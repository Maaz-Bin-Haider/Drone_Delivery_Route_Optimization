"""Greedy makespan assignment (FR-7)."""

import pytest

from ddros.constants import RESERVE_PCT
from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.scheduling.assignment import assign_fleet
from ddros.simulation.cache import RouteTable


@pytest.fixture(scope="module")
def table(city):
    return RouteTable(city, CostModel(city, SHORTEST_DISTANCE))


def test_every_delivery_is_accounted_for(table, scenario):
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    total = len(plan.assignments) + len(plan.unserviceable)
    assert total == len(scenario.deliveries)


def test_no_route_exceeds_usable_charge(table, scenario):
    """FR-4.6: the reserve margin is never spent."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    for a in plan.assignments:
        if a.reroute is None:
            assert a.route.energy_pct <= a.battery_before_pct - RESERVE_PCT + 1e-9
        assert a.battery_after_pct >= -1e-9


def test_work_is_spread_across_the_fleet(table, scenario):
    """FR-7.2: the point of the fleet is that no single drone carries everything."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    counts = [len(d.assigned) for d in plan.drones]
    assert sum(counts) == len(plan.assignments)
    assert max(counts) < len(plan.assignments), "one drone took the entire batch"


def test_drone_state_advances_after_each_assignment(table, scenario):
    """FR-7.5."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    for drone in plan.drones:
        if drone.assigned:
            assert drone.ready_at_min > 0.0
            assert drone.battery_pct < 100.0


def test_more_drones_never_lengthen_the_schedule(table, scenario):
    """FR-11.7: quantifies the benefit the brief claims for a fleet."""
    previous = float("inf")
    for n in (1, 2, 3):
        plan = assign_fleet(table, scenario.drones[:n], list(scenario.deliveries))
        if len(plan.unserviceable) == 0:
            assert plan.makespan_min <= previous + 1e-9
            previous = plan.makespan_min


def test_urgent_packages_are_dispatched_first(table, scenario):
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    departures = {a.delivery_id: a.depart_min for a in plan.assignments}
    urgent = [p.id for p in scenario.deliveries if p.priority.name == "URGENT"]
    normal = [p.id for p in scenario.deliveries if p.priority.name == "NORMAL"]
    assert max(departures[u] for u in urgent) <= min(departures[n] for n in normal) + 1e-9


def test_blocked_destination_is_reported_with_its_cause(table, scenario):
    """FR-8.3."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries),
                        blocked_nodes={"C5": "Airport approach corridor"})
    reasons = {u.delivery_id: u.reason for u in plan.unserviceable}
    assert any("Airport approach corridor" in r for r in reasons.values())


def test_every_assignment_explains_itself(table, scenario):
    """FR-10.7: no unexplained decisions."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    for a in plan.assignments:
        assert a.reason and a.drone_id in a.reason
