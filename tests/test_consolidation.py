"""En-route consolidation: dropping a parcel on a destination already crossed.

The motivating defect: a drone flew directly over a pending destination while a
second drone was dispatched to that same place. These tests pin the fix and the
policy that governs it.
"""

import pytest

from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.domain.models import DeliveryRequest, Priority
from ddros.scheduling.assignment import assign_fleet
from ddros.simulation.cache import RouteTable
from ddros.simulation.orchestrator import PlanConfig, Simulator

POLICIES = ("off", "safe", "always")


@pytest.fixture(scope="module")
def table(city):
    return RouteTable(city, CostModel(city, SHORTEST_DISTANCE))


def heavy(scenario):
    """The batch that exercises long routes, where fly-overs actually occur."""
    preset = max(scenario.presets, key=lambda p: len(p["deliveries"]))
    return [
        DeliveryRequest(f"PKG-{i + 1:03d}", d["destination"],
                        Priority[d.get("priority", "NORMAL").upper()], i)
        for i, d in enumerate(preset["deliveries"])
    ]


def flyovers(plan):
    """Deliveries whose destination another drone's route passes straight through."""
    by_dest = {}
    for a in plan.assignments:
        by_dest.setdefault(a.destination, []).append(a)
    return [(a, node, other)
            for a in plan.assignments
            for node in a.route.path[1:-1]
            for other in by_dest.get(node, [])
            if other.drone_id != a.drone_id]


# -- the defect ------------------------------------------------------------

def test_without_consolidation_drones_fly_over_pending_destinations(table, scenario):
    """The behaviour that prompted the fix, pinned so it cannot creep back in."""
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate="off")
    assert flyovers(plan), "expected a fly-over with consolidation disabled"


def test_consolidation_reduces_fly_overs(table, scenario):
    orders = heavy(scenario)
    off = flyovers(assign_fleet(table, scenario.drones[:3], orders, consolidate="off"))
    on = flyovers(assign_fleet(table, scenario.drones[:3], orders, consolidate="always"))
    assert len(on) < len(off)


# -- correctness invariants -----------------------------------------------

@pytest.mark.parametrize("policy", POLICIES)
def test_no_delivery_is_assigned_twice(table, scenario, policy):
    """A parcel already flown must not also be picked up en route."""
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate=policy)
    ids = [a.delivery_id for a in plan.assignments]
    assert len(ids) == len(set(ids)), "the same parcel was delivered twice"


@pytest.mark.parametrize("policy", POLICIES)
def test_every_order_is_accounted_for(table, scenario, policy):
    orders = heavy(scenario)
    plan = assign_fleet(table, scenario.drones[:3], orders, consolidate=policy)
    assert len(plan.assignments) + len(plan.unserviceable) == len(orders)


@pytest.mark.parametrize("policy", POLICIES)
def test_a_rider_rides_the_path_it_was_dropped_on(table, scenario, policy):
    """An en-route route must be a genuine prefix of the flight it shares."""
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate=policy)
    for a in plan.assignments:
        if not a.enroute:
            continue
        assert a.route.path[-1] == a.destination
        primary = next(p for p in plan.assignments
                       if p.drone_id == a.drone_id and not p.enroute
                       and p.depart_min == a.depart_min)
        assert primary.route.path[:len(a.route.path)] == a.route.path


def test_a_rider_adds_no_energy(table, scenario):
    """It shares a flight already paid for (assumption A-3)."""
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate="always")
    riders = [a for a in plan.assignments if a.enroute]
    assert riders, "expected at least one en-route drop"
    assert all(a.route.energy_pct == 0.0 for a in riders)


def test_a_rider_lands_before_the_flight_it_shares_completes(table, scenario):
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate="always")
    for a in plan.assignments:
        if not a.enroute:
            continue
        primary = next(p for p in plan.assignments
                       if p.drone_id == a.drone_id and not p.enroute
                       and p.depart_min == a.depart_min)
        assert a.arrive_min <= primary.arrive_min + 1e-9


# -- the policy ------------------------------------------------------------

def test_safe_policy_never_delays_a_more_urgent_parcel(table, scenario):
    """A routine parcel must not ride on an urgent flight under `safe`."""
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate="safe")
    for a in plan.assignments:
        if not a.enroute:
            continue
        primary = next(p for p in plan.assignments
                       if p.drone_id == a.drone_id and not p.enroute
                       and p.depart_min == a.depart_min)
        assert a.priority <= primary.priority, (
            f"{a.delivery_id} ({a.priority.name}) delayed "
            f"{primary.delivery_id} ({primary.priority.name})")


def test_always_takes_more_than_safe(table, scenario):
    orders = heavy(scenario)
    safe = assign_fleet(table, scenario.drones[:3], orders, consolidate="safe")
    loose = assign_fleet(table, scenario.drones[:3], orders, consolidate="always")
    assert (sum(a.enroute for a in loose.assignments)
            >= sum(a.enroute for a in safe.assignments))


def test_off_takes_nothing(table, scenario):
    plan = assign_fleet(table, scenario.drones[:3], heavy(scenario), consolidate="off")
    assert not any(a.enroute for a in plan.assignments)


# -- reporting -------------------------------------------------------------

def test_totals_do_not_count_a_shared_flight_twice(scenario):
    """A rider adds no distance, so including it would inflate the totals."""
    sim = Simulator(scenario)
    plan = sim.plan(PlanConfig(fleet_size=3, consolidate="always"))
    flown = [a for a in plan["assignments"] if not a["enroute"]]
    assert plan["totals"]["enroute_drops"] == len(plan["assignments"]) - len(flown)
    assert plan["totals"]["distance_km"] == pytest.approx(
        round(sum(a["route"]["distance_km"] for a in flown), 2), abs=0.02)


def test_a_rider_explains_that_it_rode_along(scenario):
    sim = Simulator(scenario)
    plan = sim.plan(PlanConfig(fleet_size=3, consolidate="always"))
    riders = [a for a in plan["assignments"] if a["enroute"]]
    assert riders
    for a in riders:
        assert "en route" in a["reason"]
        assert a["destination"] in a["reason"]


def test_an_unknown_policy_is_rejected(scenario):
    from ddros.web.app import create_app
    from pathlib import Path
    client = create_app(Path(__file__).resolve().parent.parent / "data").test_client()
    response = client.post("/api/plan", json={"consolidate": "maybe"})
    assert response.status_code == 400
    assert response.get_json()["field"] == "consolidate"
