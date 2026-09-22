"""A* optimality rests on an admissible, consistent heuristic. Both are asserted."""

import itertools

import pytest

from ddros.algorithms.astar import astar_route
from ddros.algorithms.dijkstra import dijkstra, dijkstra_route
from ddros.algorithms.heuristics import haversine_heuristic
from ddros.cost.cost_model import BALANCED, MINIMUM_ENERGY, SHORTEST_DISTANCE, CostModel
from ddros.environment.wind import Wind

WEIGHTS = [SHORTEST_DISTANCE, MINIMUM_ENERGY, BALANCED]
WINDS = [Wind(0, 0), Wind(9, 90), Wind(16, 225)]


@pytest.mark.parametrize("weights", WEIGHTS)
@pytest.mark.parametrize("wind", WINDS)
def test_heuristic_is_admissible(city, weights, wind):
    """h(n) must never exceed the true remaining cost (TDD 7.3.2)."""
    cm = CostModel(city, weights, wind)
    target = "C8"
    truth = dijkstra(city.reversed(), target, cm)   # true cost from every node
    h = haversine_heuristic(city, target, cm)
    for node in city.nodes:
        if truth.reached(node):
            assert h(node) <= truth.cost[node] + 1e-9, f"inadmissible at {node}"


@pytest.mark.parametrize("weights", WEIGHTS)
@pytest.mark.parametrize("wind", WINDS)
def test_heuristic_is_consistent(city, weights, wind):
    """h(u) <= c(u,v) + h(v) for every edge (TDD 7.3.3)."""
    cm = CostModel(city, weights, wind)
    h = haversine_heuristic(city, "C8", cm)
    for edges in city.adj.values():
        for e in edges:
            assert h(e.u) <= cm.edge_cost(e) + h(e.v) + 1e-9


@pytest.mark.parametrize("weights", WEIGHTS)
def test_astar_agrees_with_dijkstra_on_every_pair(city, weights):
    """FR-3.4: identical total cost, which is what makes A* a safe substitute."""
    cm = CostModel(city, weights, Wind(11, 135))
    for a, b in itertools.combinations(sorted(city.nodes), 2):
        d = dijkstra_route(city, a, b, cm)
        s = astar_route(city, a, b, cm)
        assert (d is None) == (s is None)
        if d is not None:
            assert d.cost == pytest.approx(s.cost, abs=1e-9), f"{a}->{b}"


def test_astar_expands_fewer_nodes_on_average(city):
    """The whole point of the heuristic; FR-11.2 measures it properly."""
    cm = CostModel(city, SHORTEST_DISTANCE)
    pairs = list(itertools.combinations(sorted(city.nodes), 2))
    d_total = sum(dijkstra_route(city, a, b, cm).stats.nodes_expanded for a, b in pairs)
    a_total = sum(astar_route(city, a, b, cm).stats.nodes_expanded for a, b in pairs)
    assert a_total < d_total
