"""Energy-feasible routing (TDD section 8, FR-5).

Finding a minimum-cost path subject to an additive resource budget is the
Resource Constrained Shortest Path Problem, which is NP-hard in general. Two
approaches are provided and the distinction between them is stated plainly
rather than concealed:

  * `charging_reroute` -- a two-phase heuristic. Fast and easy to analyse, but
    it tests the energy of the minimum-*cost* path to each station, so it can
    reject a station that some costlier-but-feasible path could reach.
  * `pareto_constrained_route` -- exact, by keeping the Pareto frontier of
    (cost, energy) labels. Correct, but exponential in the worst case.

Experiment 6 of TDD section 16 measures how often they disagree.
"""

from __future__ import annotations

from ..constants import CHARGE_RATE_PCT_PER_MIN, RESERVE_PCT
from ..cost.cost_model import CostModel
from ..domain.models import ChargingReroute, Route
from ..graph.graph import Graph
from ..structures.min_heap import MinHeap
from ..analysis.instrumentation import Counters
from .dijkstra import INF, SearchOutcome, dijkstra

import time as _time


def is_feasible(route: Route, battery_pct: float,
                reserve_pct: float = RESERVE_PCT) -> bool:
    """Equation (8): does this route fit within the usable charge? (FR-4.6)"""
    return route.energy_pct <= battery_pct - reserve_pct + 1e-9


def recharge_minutes(arrival_pct: float) -> float:
    """Time to restore a full charge from `arrival_pct` (FR-5.4)."""
    return max(0.0, (100.0 - arrival_pct)) / CHARGE_RATE_PCT_PER_MIN


def _assemble(graph: Graph, fwd: SearchOutcome, rev: SearchOutcome,
              station: str, battery_pct: float,
              algorithm: str) -> ChargingReroute:
    leg_one = fwd.route_to(station)
    assert leg_one is not None

    # `rev` searched the reversed graph from the destination, so its path to the
    # station runs destination -> station. Reversing it yields the forward leg,
    # and the accumulated totals already describe forward traversal.
    rev_path = rev.path_to(station)
    assert rev_path is not None
    leg_two = Route(
        path=tuple(reversed(rev_path)),
        distance_km=rev.distance_km[station],
        energy_pct=rev.energy_pct[station],
        time_min=rev.time_min[station],
        cost=rev.cost[station],
        algorithm=algorithm,
        stats=rev.stats,
    )
    arrival = battery_pct - fwd.energy_pct[station]
    return ChargingReroute(station, leg_one, leg_two, arrival,
                           recharge_minutes(arrival))


def _best_station(graph: Graph, cost_model: CostModel,
                  cost_to: dict[str, float], energy_to: dict[str, float],
                  cost_from: dict[str, float], energy_from: dict[str, float],
                  battery_pct: float, reserve_pct: float) -> str | None:
    """argmin over charging stations. O(|C|) once the tables are in hand."""
    usable = battery_pct - reserve_pct
    full = 100.0 - reserve_pct
    best, best_cost = None, INF
    for c in graph.charging_stations:
        if cost_to.get(c, INF) >= INF or cost_from.get(c, INF) >= INF:
            continue
        if energy_to[c] > usable + 1e-9:          # leg one infeasible
            continue
        if energy_from[c] > full + 1e-9:          # leg two infeasible
            continue
        arrival = battery_pct - energy_to[c]
        recharge = recharge_minutes(arrival)
        total = (cost_to[c] + cost_from[c]
                 + cost_model.weights.gamma * recharge / cost_model.t_ref)
        if total < best_cost or (total == best_cost and best is not None and c < best):
            best_cost, best = total, c
    return best


def charging_reroute(graph: Graph, source: str, target: str, battery_pct: float,
                     cost_model: CostModel,
                     reserve_pct: float = RESERVE_PCT) -> ChargingReroute | None:
    """Best two-leg route through a charging station (TDD section 8.3).

    The naive formulation runs a pair of searches per candidate station, costing
    O(C * (V + E) log V). One all-targets search from the drone and one on the
    reversed graph from the destination together yield the cost and energy to
    and from *every* station, reducing this to O((V + E) log V + C) -- two
    searches in total, however many stations exist.
    """
    fwd = dijkstra(graph, source, cost_model)
    rev = dijkstra(graph.reversed(), target, cost_model)
    station = _best_station(graph, cost_model, fwd.cost, fwd.energy_pct,
                            rev.cost, rev.energy_pct, battery_pct, reserve_pct)
    if station is None:
        return None
    return _assemble(graph, fwd, rev, station, battery_pct, "dijkstra")


def pareto_constrained_route(graph: Graph, source: str, target: str,
                             budget_pct: float, cost_model: CostModel,
                             max_labels: int = 200_000) -> Route | None:
    """Exact minimum-cost path among energy-feasible paths (TDD section 8.4).

    Label-setting over the (cost, energy) Pareto frontier: a label survives only
    if no settled label at the same node beats it on both axes. Labels exceeding
    the budget are pruned at generation.
    """
    if source not in graph.nodes or target not in graph.nodes:
        raise KeyError("unknown endpoint")

    counters = Counters()
    started = _time.perf_counter()

    # label: (cost, energy, distance, time, node, parent_index)
    labels: list[tuple[float, float, float, float, str, int]] = [
        (0.0, 0.0, 0.0, 0.0, source, -1)
    ]
    frontier: dict[str, list[tuple[float, float]]] = {source: [(0.0, 0.0)]}
    heap = MinHeap(counters)
    heap.push((0.0, 0.0), 0)

    while heap and len(labels) < max_labels:
        _, idx = heap.pop_min()
        cost, energy, dist, tmin, node, _parent = labels[idx]
        counters.nodes_expanded += 1

        if node == target:
            path = []
            cur = idx
            while cur != -1:
                path.append(labels[cur][4])
                cur = labels[cur][5]
            path.reverse()
            elapsed = (_time.perf_counter() - started) * 1000.0
            return Route(tuple(path), dist, energy, tmin, cost,
                         "pareto", counters.snapshot(elapsed))

        for e in graph.adj[node]:
            if not cost_model.is_open(e):
                continue
            ne = energy + cost_model.edge_energy(e)
            if ne > budget_pct + 1e-9:
                continue                       # prune: over budget
            nc = cost + cost_model.edge_cost(e)
            existing = frontier.setdefault(e.v, [])
            if any(c <= nc + 1e-12 and en <= ne + 1e-12 for c, en in existing):
                continue                       # dominated
            existing[:] = [(c, en) for c, en in existing
                           if not (nc <= c + 1e-12 and ne <= en + 1e-12)]
            existing.append((nc, ne))
            labels.append((nc, ne, dist + e.distance_km,
                           tmin + cost_model.edge_time(e), e.v, idx))
            heap.push((nc, ne), len(labels) - 1)

    return None
