"""No-fly zones (FR-8)."""

from dataclasses import replace

from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.environment.no_fly import NoFlyMask


def activate(scenario, zone_id):
    return [replace(z, active=(z.id == zone_id)) for z in scenario.zones]


def test_inactive_zones_block_nothing(scenario, city):
    assert not NoFlyMask(city, list(scenario.zones))


def test_circular_zone_blocks_edges_and_nodes(scenario, city):
    mask = NoFlyMask(city, activate(scenario, "nfz_airport"))
    assert mask
    assert mask.node_blocked_by("C5") == "Airport approach corridor"


def test_polygon_zone_blocks_its_interior(scenario, city):
    mask = NoFlyMask(city, activate(scenario, "nfz_stadium"))
    blocked = [n for n in city.nodes if mask.node_blocked_by(n)]
    assert blocked, "the stadium polygon should contain at least one node"


def test_blocked_edges_are_closed_to_the_router(scenario, city):
    mask = NoFlyMask(city, activate(scenario, "nfz_airport"))
    cm = CostModel(city, SHORTEST_DISTANCE, mask=mask)
    closed = [e for edges in city.adj.values() for e in edges if not cm.is_open(e)]
    assert closed
    assert all(mask.is_blocked(e) for e in closed)


def test_zone_imposes_a_cost_penalty_not_a_shortcut(scenario, city):
    """Routing around restricted airspace can never become cheaper (FR-8.6)."""
    from ddros.algorithms.dijkstra import dijkstra_route
    free = CostModel(city, SHORTEST_DISTANCE)
    masked = CostModel(city, SHORTEST_DISTANCE,
                       mask=NoFlyMask(city, activate(scenario, "nfz_airport")))
    for target in ("C4", "C8", "C3"):
        a = dijkstra_route(city, "W", target, free)
        b = dijkstra_route(city, "W", target, masked)
        if b is not None:
            assert b.cost >= a.cost - 1e-9
