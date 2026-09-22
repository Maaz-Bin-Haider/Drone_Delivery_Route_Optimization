"""A* heuristics (TDD section 7.3.1).

The heuristic is the straight-line distance to the goal scaled by Lambda, the
least composite cost attributable to one kilometre of travel under the most
favourable possible conditions. Because Lambda under-counts every term, the
estimate can never exceed the true remaining cost.

Admissibility and consistency are proved in TDD sections 7.3.2 and 7.3.3, and
are additionally asserted by `test_heuristic_admissible` and
`test_heuristic_consistent`.
"""

from __future__ import annotations

from typing import Callable

from ..cost.cost_model import CostModel
from ..geo import haversine_km
from ..graph.graph import Graph


def haversine_heuristic(graph: Graph, target: str,
                        cost_model: CostModel) -> Callable[[str], float]:
    """Build h(n) = Lambda * great-circle distance from n to the target.

    Lambda is computed once when the cost model is built, so each evaluation is
    a haversine call and one multiplication.
    """
    goal = graph.nodes[target]
    lam = cost_model.lambda_min
    nodes = graph.nodes

    def h(node_id: str) -> float:
        n = nodes[node_id]
        return lam * haversine_km(n.lat, n.lon, goal.lat, goal.lon)

    return h


def zero_heuristic(_graph: Graph, _target: str,
                   _cost_model: CostModel) -> Callable[[str], float]:
    """Trivially admissible heuristic; reduces A* exactly to Dijkstra.

    Used in testing to confirm the two algorithms coincide when h == 0.
    """
    return lambda _node_id: 0.0
