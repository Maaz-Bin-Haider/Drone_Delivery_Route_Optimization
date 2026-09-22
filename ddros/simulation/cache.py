"""All-pairs route table and configuration cache (TDD sections 9.2 and 11).

Assignment must evaluate travel between many (drone position, destination)
pairs. Running a fresh search per pair costs O(P * D * (V + E) log V). Running
Dijkstra once from every node instead costs O(V * (V + E) log V) and answers
every pair by lookup, which for any realistic batch is the cheaper trade.

The table depends only on the graph and the cost model, so it survives changes
to the fleet or the delivery batch -- adding a delivery is therefore nearly free.
"""

from __future__ import annotations

from collections import OrderedDict

from ..algorithms.constrained import _best_station, recharge_minutes
from ..algorithms.dijkstra import INF, dijkstra
from ..constants import RESERVE_PCT
from ..cost.cost_model import CostModel
from ..domain.models import ChargingReroute, Route
from ..graph.graph import Graph


class RouteTable:
    """Minimum-cost route between every ordered pair of nodes.

    Built with V Dijkstra runs: O(V * (V + E) log V) time, O(V^2) space.
    """

    __slots__ = ("graph", "cost_model", "_outcomes", "searches", "nodes_expanded")

    def __init__(self, graph: Graph, cost_model: CostModel) -> None:
        self.graph = graph
        self.cost_model = cost_model
        self._outcomes = {nid: dijkstra(graph, nid, cost_model) for nid in graph.nodes}
        self.searches = len(self._outcomes)
        self.nodes_expanded = sum(o.stats.nodes_expanded for o in self._outcomes.values())

    def route(self, source: str, target: str) -> Route | None:
        return self._outcomes[source].route_to(target)

    def cost(self, source: str, target: str) -> float:
        return self._outcomes[source].cost.get(target, INF)

    def energy(self, source: str, target: str) -> float:
        return self._outcomes[source].energy_pct.get(target, INF)

    def time(self, source: str, target: str) -> float:
        return self._outcomes[source].time_min.get(target, INF)

    def reached(self, source: str, target: str) -> bool:
        return self.cost(source, target) < INF

    def leg_times(self, path: tuple[str, ...] | list[str]) -> list[float]:
        """Cumulative flight time at each node of a path, in minutes.

        Needed to place an en-route delivery on the clock: a drone crossing a
        pending destination reaches it partway through its own trip, not at the
        end of it.
        """
        out = [0.0]
        for u, v in zip(path, path[1:]):
            edge = next((e for e in self.graph.adj[u] if e.v == v), None)
            if edge is None:
                return out + [out[-1]] * (len(path) - len(out))
            out.append(out[-1] + self.cost_model.edge_time(edge))
        return out

    def charging_reroute(self, source: str, target: str, battery_pct: float,
                         reserve_pct: float = RESERVE_PCT) -> ChargingReroute | None:
        """Best station, resolved entirely from the table in O(C).

        The table already holds every source-to-station and station-to-target
        figure, so the two searches of TDD section 8.3 have effectively been
        amortised across the whole planning cycle.
        """
        cost_from = {c: self.cost(c, target) for c in self.graph.charging_stations}
        energy_from = {c: self.energy(c, target) for c in self.graph.charging_stations}
        station = _best_station(
            self.graph, self.cost_model,
            self._outcomes[source].cost, self._outcomes[source].energy_pct,
            cost_from, energy_from, battery_pct, reserve_pct,
        )
        if station is None:
            return None
        leg_one = self.route(source, station)
        leg_two = self.route(station, target)
        if leg_one is None or leg_two is None:
            return None
        arrival = battery_pct - leg_one.energy_pct
        return ChargingReroute(station, leg_one, leg_two, arrival,
                               recharge_minutes(arrival))


def combined_route(reroute: ChargingReroute) -> Route:
    """Flatten a two-leg reroute into one Route for display (FR-5.5)."""
    a, b = reroute.leg_one, reroute.leg_two
    return Route(
        path=reroute.path,
        distance_km=a.distance_km + b.distance_km,
        energy_pct=a.energy_pct + b.energy_pct,
        time_min=reroute.total_time_min,
        cost=a.cost + b.cost,
        algorithm=a.algorithm,
        stats=a.stats,
    )


class ConfigCache:
    """Bounded LRU over route tables, keyed by cost-model configuration.

    A cache hit skips the environment build, the cost-model build and the
    all-pairs precomputation, which is what keeps the dashboard sliders inside
    the 2-second replan budget of NFR-3.
    """

    def __init__(self, capacity: int = 16) -> None:
        self.capacity = capacity
        self._entries: OrderedDict[tuple, RouteTable] = OrderedDict()
        self.hits = 0
        self.misses = 0

    def get(self, graph: Graph, cost_model: CostModel) -> RouteTable:
        key = cost_model.config_key()
        if key in self._entries:
            self.hits += 1
            self._entries.move_to_end(key)
            return self._entries[key]
        self.misses += 1
        table = RouteTable(graph, cost_model)
        self._entries[key] = table
        if len(self._entries) > self.capacity:
            self._entries.popitem(last=False)
        return table

    def clear(self) -> None:
        self._entries.clear()
