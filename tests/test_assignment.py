"""Greedy makespan assignment (FR-7)."""

import pytest

from ddros.constants import RESERVE_PCT
from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.domain.models import Drone
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


def test_adding_a_drone_can_lengthen_the_schedule(table, scenario):
    """Graham's timing anomaly, reproduced in this system.

    List scheduling is not monotone in the number of machines: adding one can
    make the makespan worse, because it changes every subsequent assignment
    decision. The test asserts the anomaly exists rather than pretending it does
    not; a suite demanding monotonicity would be asserting something false about
    greedy scheduling.
    """
    makespans = [
        assign_fleet(table, scenario.drones[:n], list(scenario.deliveries)).makespan_min
        for n in range(1, len(scenario.drones) + 1)
    ]
    assert any(b > a + 1e-9 for a, b in zip(makespans, makespans[1:])), (
        "expected a non-monotone step somewhere in the fleet-size sweep")


def test_the_anomaly_survives_identical_batteries(table, scenario):
    """The anomaly is the scheduler's, not the fleet's battery states.

    Giving every drone an identical full charge removes battery heterogeneity
    entirely, and removes charging detours with it. The anomaly persists, so the
    cause is the greedy assignment itself: travel times are sequence-dependent,
    because a delivery's duration depends on where its drone happens to be, and
    adding a machine changes every subsequent choice.

    Only sizes that serve the whole batch without detours are compared, since a
    makespan from a plan that delivered less is not comparable.
    """
    rows = []
    for n in range(1, len(scenario.drones) + 1):
        fleet = [Drone(f"D{i + 1}", table.graph.warehouse, 100.0) for i in range(n)]
        plan = assign_fleet(table, fleet, list(scenario.deliveries))
        if plan.unserviceable or any(a.reroute for a in plan.assignments):
            continue
        rows.append((n, plan.makespan_min))

    assert len(rows) >= 3, "not enough comparable fleet sizes"
    assert any(b > a + 1e-9 for (_, a), (_, b) in zip(rows, rows[1:])), (
        "expected the anomaly to persist with identical batteries and no detours")


def test_each_tour_is_flown_in_priority_order(table, scenario):
    """Priority governs the order a drone works through its own stops.

    It cannot govern order *across* drones: several launch at once, so a routine
    parcel on an idle drone necessarily departs alongside an urgent one on
    another. Comparing departures fleet-wide would assert something the problem
    does not permit.
    """
    plan = assign_fleet(table, scenario.drones, list(scenario.deliveries))
    for drone in plan.drones:
        legs = sorted((a for a in plan.assignments if a.drone_id == drone.id),
                      key=lambda a: a.depart_min)
        for index, leg in enumerate(legs):
            if leg.enroute:
                continue          # a free stop delays nothing but by a handover
            assert not any(x.priority < leg.priority for x in legs[index + 1:]), (
                f"{drone.id} detours to {leg.destination} ahead of more urgent work")


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
