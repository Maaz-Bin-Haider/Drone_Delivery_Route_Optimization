"""Greedy makespan assignment (FR-7)."""

import pytest

from ddros.constants import RESERVE_PCT
from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.domain.models import DeliveryRequest, Drone, Priority
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


def test_the_full_fleet_substantially_beats_a_single_drone(table, scenario):
    """FR-11.7: quantifies the benefit the brief claims for a fleet."""
    one = assign_fleet(table, scenario.drones[:1], list(scenario.deliveries))
    allof = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    assert allof.makespan_min < one.makespan_min / 2


def test_the_supplied_scenario_is_monotone_in_fleet_size(table, scenario):
    """Adding a drone to the demonstration fleet never lengthens the schedule.

    This is not free. List scheduling is famously *not* monotone in the number
    of machines -- Graham (1969) -- and before the improvement pass refused
    moves that push the makespan out, this scenario showed the anomaly plainly:
    three drones finished slower than two. Pinning it here means a change that
    reintroduces the regression fails loudly.
    """
    previous = float("inf")
    for n in range(1, len(scenario.drones) + 1):
        plan = assign_fleet(table, scenario.drones[:n], list(scenario.deliveries))
        if plan.unserviceable:
            previous = float("inf")      # not comparable: it delivered less
            continue
        assert plan.makespan_min <= previous + 1e-9, f"non-monotone at {n} drones"
        previous = plan.makespan_min


def test_monotonicity_is_not_guaranteed_in_general(table, scenario, city):
    """The anomaly is suppressed on realistic batches, not eliminated.

    The makespan guard stops any single move from lengthening the schedule, but
    each fleet size is planned from its own starting point, so nothing forces
    the results to be monotone *across* sizes. A scan of random batches finds
    the anomaly in roughly 4% of adjacent comparisons; this is one of them,
    pinned so the documentation cannot quietly become false.
    """
    import random

    from ddros.domain.models import NodeType
    customers = sorted(n.id for n in city.nodes.values()
                       if n.type is NodeType.CUSTOMER)
    rng = random.Random(3)
    batch = [DeliveryRequest(f"PKG-{i + 1:03d}", rng.choice(customers),
                             rng.choice([Priority.URGENT, Priority.HIGH,
                                         Priority.NORMAL]), i)
             for i in range(12)]
    makespans = []
    for n in range(2, 8):
        plan = assign_fleet(table, scenario.drones[:n], batch)
        assert not plan.unserviceable
        makespans.append(plan.makespan_min)
    assert any(b > a + 1e-9 for a, b in zip(makespans, makespans[1:])), (
        f"expected the anomaly on this instance, got {makespans}")


def test_blocked_destination_is_reported_with_its_cause(table, scenario):
    """FR-8.3."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries),
                        blocked_nodes={"L19": "Kestrel Aerodrome approach"})
    reasons = {u.delivery_id: u.reason for u in plan.unserviceable}
    assert any("Kestrel Aerodrome approach" in r for r in reasons.values())


def test_every_assignment_explains_itself(table, scenario):
    """FR-10.7: no unexplained decisions."""
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    for a in plan.assignments:
        assert a.reason and a.drone_id in a.reason
