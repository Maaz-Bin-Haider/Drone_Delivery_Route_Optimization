"""Dijkstra's algorithm and the shared search core (TDD section 7.2, FR-3.1).

A* is Dijkstra with the heap key changed from g(v) to g(v) + h(v), so both are
served by one routine here; `astar.py` supplies the heuristic. Keeping them in a
single implementation is deliberate: it makes the claim that they differ only in
the priority key verifiable by inspection, and guarantees the comparison of
FR-11.2 is measuring the heuristic rather than two unrelated codebases.

Complexity: O((V + E) log V) time, O(V + E) space.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from ..analysis.instrumentation import Counters, SearchStats
from ..cost.cost_model import CostModel
from ..domain.models import Edge, Route
from ..graph.graph import Graph
from ..structures.min_heap import MinHeap

Heuristic = Callable[[str], float]

INF = float("inf")


@dataclass
class SearchOutcome:
    """Full result of one search, including every settled node.

    A single all-targets run therefore subsumes many single-pair queries, which
    is the observation the charging reroute of TDD section 8.3 exploits.
    """

    source: str
    cost: dict[str, float]
    distance_km: dict[str, float]
    energy_pct: dict[str, float]
    time_min: dict[str, float]
    prev: dict[str, tuple[str, Edge] | None]
    stats: SearchStats
    algorithm: str

    def reached(self, node_id: str) -> bool:
        return self.cost.get(node_id, INF) < INF

    def path_to(self, target: str) -> tuple[str, ...] | None:
        """Reconstruct the vertex sequence by walking `prev` backward. O(|path|)."""
        if not self.reached(target):
            return None
        out = [target]
        cur = target
        while cur != self.source:
            step = self.prev[cur]
            if step is None:
                return None
            cur = step[0]
            out.append(cur)
        out.reverse()
        return tuple(out)

    def route_to(self, target: str) -> Route | None:
        """Assemble a Route, or None when the target is unreachable (FR-3.7)."""
        path = self.path_to(target)
        if path is None:
            return None
        return Route(
            path=path,
            distance_km=self.distance_km[target],
            energy_pct=self.energy_pct[target],
            time_min=self.time_min[target],
            cost=self.cost[target],
            algorithm=self.algorithm,
            stats=self.stats,
        )


def search(graph: Graph, source: str, cost_model: CostModel,
           target: str | None = None, heuristic: Heuristic | None = None,
           algorithm: str = "dijkstra") -> SearchOutcome:
    """Shared label-setting search over non-negative edge costs.

    With `heuristic=None` this is Dijkstra; with a consistent heuristic it is A*.
    Passing a `target` enables early exit, which is what makes A*'s reduced
    expansion count visible.
    """
    if source not in graph.nodes:
        raise KeyError(f"unknown source node '{source}'")

    counters = Counters()
    started = time.perf_counter()

    cost = {nid: INF for nid in graph.nodes}
    dist_km = {nid: 0.0 for nid in graph.nodes}
    energy = {nid: 0.0 for nid in graph.nodes}
    time_min = {nid: 0.0 for nid in graph.nodes}
    prev: dict[str, tuple[str, Edge] | None] = {nid: None for nid in graph.nodes}
    settled: set[str] = set()

    cost[source] = 0.0
    heap = MinHeap(counters)
    heap.push(heuristic(source) if heuristic else 0.0, source)

    while heap:
        _, u = heap.pop_min()
        if u in settled:
            continue                       # stale entry, discarded lazily
        settled.add(u)
        counters.nodes_expanded += 1
        if target is not None and u == target:
            break                          # early exit

        base = cost[u]
        for e in graph.adj[u]:
            if not cost_model.is_open(e):  # no-fly mask, FR-8.2
                continue
            v = e.v
            if v in settled:
                continue
            nc = base + cost_model.edge_cost(e)
            if nc < cost[v]:
                cost[v] = nc
                dist_km[v] = dist_km[u] + e.distance_km
                energy[v] = energy[u] + cost_model.edge_energy(e)
                time_min[v] = time_min[u] + cost_model.edge_time(e)
                prev[v] = (u, e)
                heap.push(nc + heuristic(v) if heuristic else nc, v)

    elapsed_ms = (time.perf_counter() - started) * 1000.0
    return SearchOutcome(source, cost, dist_km, energy, time_min, prev,
                         counters.snapshot(elapsed_ms), algorithm)


def dijkstra(graph: Graph, source: str, cost_model: CostModel,
             target: str | None = None) -> SearchOutcome:
    """Single-source shortest paths. Optimal for any non-negative cost."""
    return search(graph, source, cost_model, target, None, "dijkstra")


def dijkstra_route(graph: Graph, source: str, target: str,
                   cost_model: CostModel) -> Route | None:
    return dijkstra(graph, source, cost_model, target).route_to(target)
