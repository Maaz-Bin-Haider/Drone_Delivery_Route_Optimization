"""Weighted graph over city locations, stored as an adjacency list.

Space complexity O(V + E), as required by FR-1.5.
"""

from __future__ import annotations

from ..constants import ENERGY_RATE_PCT_PER_KM
from ..domain.models import Edge, Node, NodeType
from ..geo import haversine_km, initial_bearing_deg


class Graph:
    """Undirected city graph with direction-dependent traversal costs.

    The corridor between two locations is stored as two directed `Edge` records
    with opposing bearings. They share a distance and a still-air energy cost,
    but the wind model of TDD section 6.1 gives them different actual costs, so
    flying north-east up a corridor is genuinely not the same as flying
    south-west down it.
    """

    __slots__ = ("nodes", "adj", "_charging", "_origin")

    def __init__(self, nodes: dict[str, Node], adj: dict[str, list[Edge]]) -> None:
        self.nodes = nodes
        self.adj = adj
        self._charging = tuple(
            n.id for n in nodes.values() if n.type is NodeType.CHARGING_STATION
        )
        if nodes:
            self._origin = (
                sum(n.lat for n in nodes.values()) / len(nodes),
                sum(n.lon for n in nodes.values()) / len(nodes),
            )
        else:
            self._origin = (0.0, 0.0)

    # -- construction -------------------------------------------------------

    @classmethod
    def build(cls, nodes: list[Node], edge_specs: list[dict]) -> "Graph":
        """Build from node records and raw edge specifications.

        `distance_km` and `base_energy_pct` may be omitted from a specification;
        distance is then derived by haversine from the endpoint coordinates
        (FR-1.9) and energy from the nominal consumption rate. Bearings are
        always derived, never authored.
        """
        by_id = {n.id: n for n in nodes}
        adj: dict[str, list[Edge]] = {n.id: [] for n in nodes}

        for spec in edge_specs:
            u, v = spec["u"], spec["v"]
            if u not in by_id or v not in by_id:
                raise KeyError(f"edge {u}->{v} references an unknown node")
            a, b = by_id[u], by_id[v]
            dist = spec.get("distance_km")
            if dist is None:
                dist = haversine_km(a.lat, a.lon, b.lat, b.lon)
            energy = spec.get("base_energy_pct")
            if energy is None:
                energy = dist * ENERGY_RATE_PCT_PER_KM
            fwd = initial_bearing_deg(a.lat, a.lon, b.lat, b.lon)
            rev = initial_bearing_deg(b.lat, b.lon, a.lat, a.lon)
            adj[u].append(Edge(u, v, dist, energy, fwd))
            adj[v].append(Edge(v, u, dist, energy, rev))

        return cls(by_id, adj)

    # -- access -------------------------------------------------------------

    def neighbours(self, node_id: str) -> list[Edge]:
        return self.adj[node_id]

    @property
    def charging_stations(self) -> tuple[str, ...]:
        return self._charging

    @property
    def origin(self) -> tuple[float, float]:
        """Mean coordinate, used as the local projection origin for zone tests."""
        return self._origin

    @property
    def warehouse(self) -> str:
        for n in self.nodes.values():
            if n.type is NodeType.WAREHOUSE:
                return n.id
        raise LookupError("graph contains no warehouse node")

    def edge_count(self) -> int:
        """Number of undirected corridors (each stored twice in the adjacency list)."""
        return sum(len(v) for v in self.adj.values()) // 2

    def reversed(self) -> "Graph":
        """Reverse graph, used by the backward search of TDD section 8.3.

        A forward edge u->v becomes an edge out of `v` that arrives at `u` while
        retaining the *forward* bearing. Evaluating it therefore yields the cost
        of flying u->v, which is exactly what a backward search needs: the cost
        of reaching the destination from each node, not of leaving it.
        """
        radj: dict[str, list[Edge]] = {nid: [] for nid in self.nodes}
        for edges in self.adj.values():
            for e in edges:
                radj[e.v].append(Edge(e.v, e.u, e.distance_km,
                                      e.base_energy_pct, e.bearing_uv_deg))
        return Graph(self.nodes, radj)

    def __repr__(self) -> str:
        return f"<Graph V={len(self.nodes)} E={self.edge_count()}>"
