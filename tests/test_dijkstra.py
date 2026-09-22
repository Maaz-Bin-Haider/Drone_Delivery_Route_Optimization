"""Dijkstra correctness, including the example the project brief works through."""

import pytest

from ddros.algorithms.dijkstra import dijkstra, dijkstra_route
from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel


def test_reproduces_the_brief_worked_example(brief_graph):
    """Brief page 8: W->B->E at 4 km beats the 8 km and 9 km alternatives."""
    cm = CostModel(brief_graph, SHORTEST_DISTANCE)
    route = dijkstra_route(brief_graph, "W", "E", cm)
    assert route.path == ("W", "B", "E")
    assert route.distance_km == pytest.approx(4.0)


def test_all_alternatives_are_costed_correctly(brief_graph):
    cm = CostModel(brief_graph, SHORTEST_DISTANCE)
    out = dijkstra(brief_graph, "W", cm)
    assert out.distance_km["E"] == pytest.approx(4.0)   # W->B->E
    assert out.distance_km["A"] == pytest.approx(4.0)
    assert out.distance_km["D"] == pytest.approx(6.0)   # W->B->E->D beats W->C->D


def test_unreachable_target_returns_none_not_an_exception(brief_graph):
    """Failing loudly but gracefully (principle P6, FR-3.7)."""
    from ddros.domain.models import Node, NodeType
    from ddros.graph.graph import Graph
    nodes = list(brief_graph.nodes.values()) + [
        Node("Z", "Island", 1.0, 1.0, NodeType.CUSTOMER)
    ]
    specs = [{"u": e.u, "v": e.v, "distance_km": e.distance_km}
             for edges in brief_graph.adj.values() for e in edges if e.u < e.v]
    graph = Graph.build(nodes, specs)
    cm = CostModel(graph, SHORTEST_DISTANCE)
    assert dijkstra_route(graph, "W", "Z", cm) is None


def test_every_node_is_expanded_at_most_once(city):
    cm = CostModel(city, SHORTEST_DISTANCE)
    out = dijkstra(city, "W", cm)
    assert out.stats.nodes_expanded <= len(city.nodes)


def test_path_totals_match_the_edges_walked(city):
    cm = CostModel(city, SHORTEST_DISTANCE)
    route = dijkstra_route(city, "W", "C8", cm)
    by_pair = {(e.u, e.v): e for edges in city.adj.values() for e in edges}
    walked = sum(by_pair[(a, b)].distance_km
                 for a, b in zip(route.path, route.path[1:]))
    assert route.distance_km == pytest.approx(walked)
