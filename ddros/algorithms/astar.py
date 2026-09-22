"""A* search (TDD section 7.3, FR-3.2).

A* is Dijkstra with the priority key changed from g(v) to f(v) = g(v) + h(v).
The search body is shared with `dijkstra.py`; only the key differs. Worst-case
complexity is therefore identical, O((V + E) log V); the benefit is a smaller
constant factor, which FR-11.2 exists to measure rather than assert.
"""

from __future__ import annotations

from ..cost.cost_model import CostModel
from ..domain.models import Route
from ..graph.graph import Graph
from .dijkstra import SearchOutcome, search
from .heuristics import haversine_heuristic


def astar(graph: Graph, source: str, target: str,
          cost_model: CostModel) -> SearchOutcome:
    """Goal-directed search. Requires a target; there is no all-targets A*."""
    if target not in graph.nodes:
        raise KeyError(f"unknown target node '{target}'")
    h = haversine_heuristic(graph, target, cost_model)
    return search(graph, source, cost_model, target, h, "astar")


def astar_route(graph: Graph, source: str, target: str,
                cost_model: CostModel) -> Route | None:
    return astar(graph, source, target, cost_model).route_to(target)


ROUTERS = {"dijkstra": "dijkstra", "astar": "astar"}


def route(graph: Graph, source: str, target: str, cost_model: CostModel,
          algorithm: str = "astar") -> Route | None:
    """Dispatch to the requested router. Both must return equal-cost routes."""
    from .dijkstra import dijkstra_route
    if algorithm == "astar":
        return astar_route(graph, source, target, cost_model)
    if algorithm == "dijkstra":
        return dijkstra_route(graph, source, target, cost_model)
    raise ValueError(f"unknown algorithm '{algorithm}'")
