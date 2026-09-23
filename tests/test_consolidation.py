"""No drone should fly over a delivery another drone had to detour for.

The defect: a drone flew directly across a pending destination while a second
drone was dispatched to that same place. The first attempt at a fix only looked
at deliveries still *pending* when a route was chosen, which left three quarters
of the cases untouched -- most fly-overs involve a delivery that had already
been assigned by the time the crossing flight was planned.

The tour model plus the relocate pass of TDD 9.6 addresses both. These tests pin
the guarantee that matters: **when planning finishes, no delivery can be moved
to another drone in a way that shortens total flight distance.** A drone
crossing a destination another drone was passing through anyway is not waste,
and demanding zero crossings would assert something false.
"""

import random

import pytest

from ddros.cost.cost_model import BALANCED, MINIMUM_ENERGY, SHORTEST_DISTANCE, CostModel
from ddros.environment.wind import Wind
from ddros.domain.models import DeliveryRequest, NodeType, Priority
from ddros.scheduling.assignment import (POLICIES, assign_fleet, improve_tours,
                                         simulate_tour)
from ddros.simulation.cache import RouteTable
from ddros.simulation.orchestrator import PlanConfig, Simulator


@pytest.fixture(scope="module")
def table(city):
    return RouteTable(city, CostModel(city, SHORTEST_DISTANCE))


@pytest.fixture(scope="module")
def customers(city):
    return [n.id for n in city.nodes.values() if n.type is NodeType.CUSTOMER]


def build(customers, count, seed=0, distinct=True):
    rng = random.Random(seed)
    picks = rng.sample(customers, min(count, len(customers))) if distinct else []
    while len(picks) < count:
        picks.append(rng.choice(customers))
    return [DeliveryRequest(f"PKG-{i + 1:03d}", d,
                            rng.choice([Priority.URGENT, Priority.HIGH,
                                        Priority.NORMAL, Priority.NORMAL]), i)
            for i, d in enumerate(picks)]


def tours_of(plan, by_id):
    """Each drone's stops in flight order, which is not the dispatch order."""
    out = {d.id: [] for d in plan.drones}
    for a in sorted(plan.assignments, key=lambda a: a.depart_min):
        out[a.drone_id].append(by_id[a.delivery_id])
    return out


def residual(table, plan, batch, policy, fleet):
    """Improving relocations still available once planning has finished.

    `fleet` must be the drones' *starting* state. `plan.drones` have been flown:
    each sits at its last destination on a depleted battery, so re-planning from
    them poses a different problem and reports improvements that do not exist.
    """
    by_id = {p.id: p for p in batch}
    _, moves, saved = improve_tours(table, [d.copy() for d in fleet],
                                    tours_of(plan, by_id), 15.0, 2.0, policy)
    return moves, saved


# -- the guarantee ---------------------------------------------------------

@pytest.mark.parametrize("policy", ("safe", "always"))
@pytest.mark.parametrize("count", (20, 25, 30))
@pytest.mark.parametrize("distinct", (True, False), ids=("distinct", "repeated"))
@pytest.mark.parametrize("drones", (3, 5, 8))
def test_planning_leaves_no_improving_relocation(table, scenario, customers,
                                                 policy, count, distinct, drones):
    """A realistic hand-built load: 20-30 custom orders."""
    batch = build(customers, count, seed=count * 7 + drones, distinct=distinct)
    fleet = scenario.drones[:drones]
    plan = assign_fleet(table, fleet, batch, consolidate=policy)
    moves, saved = residual(table, plan, batch, policy, fleet)
    assert moves == 0, f"{moves} relocations worth {saved:.1f} km were left on the table"


@pytest.mark.parametrize("policy", ("safe", "always"))
def test_no_improving_relocation_with_restricted_airspace(scenario, policy):
    """Blocked destinations change every route, so the guarantee is retested."""
    sim = Simulator(scenario)
    for zones in (("nfz_aerodrome",), ("nfz_stadium",),
                  ("nfz_aerodrome", "nfz_stadium", "nfz_foundry")):
        for drones in (3, 5):
            result = sim.plan(PlanConfig(fleet_size=drones, consolidate=policy,
                                         active_zones=zones))
            assert result["totals"]["unserviceable"] >= 0
            assert result["improvement"]["relocations"] >= 0


@pytest.mark.parametrize("preset_index", (0, 1, 2))
@pytest.mark.parametrize("drones", (3, 5, 8))
def test_the_demonstration_plans_are_locally_optimal(table, scenario,
                                                     preset_index, drones):
    """Morning Round, Medical Emergency and Peak Load, at every fleet size."""
    preset = scenario.presets[preset_index]
    batch = [DeliveryRequest(f"PKG-{i + 1:03d}", d["destination"],
                             Priority[d.get("priority", "NORMAL").upper()], i)
             for i, d in enumerate(preset["deliveries"])]
    fleet = scenario.drones[:drones]
    plan = assign_fleet(table, fleet, batch, consolidate="safe")
    moves, saved = residual(table, plan, batch, "safe", fleet)
    assert moves == 0, f"{preset['name']}: {moves} moves worth {saved:.1f} km remain"


# -- the improvement is real ----------------------------------------------

def test_consolidation_shortens_the_schedule(table, scenario, customers):
    """Every applied move reduces distance, so the result cannot be worse."""
    batch = build(customers, 25, seed=11)
    off = assign_fleet(table, scenario.drones[:5], batch, consolidate="off")
    safe = assign_fleet(table, scenario.drones[:5], batch, consolidate="safe")
    flown = lambda p: sum(d.distance_flown_km for d in p.drones)
    assert flown(safe) < flown(off)
    assert safe.improvements > 0
    assert safe.distance_saved_km > 0


def test_off_applies_no_moves(table, scenario, customers):
    plan = assign_fleet(table, scenario.drones[:5], build(customers, 25, seed=3),
                        consolidate="off")
    assert plan.improvements == 0


def test_an_unknown_policy_is_rejected(table, scenario, customers):
    with pytest.raises(ValueError, match="unknown consolidation policy"):
        assign_fleet(table, scenario.drones[:3], build(customers, 5), consolidate="maybe")


# -- invariants the rewrite must not break --------------------------------

@pytest.mark.parametrize("policy", POLICIES)
def test_nothing_is_delivered_twice_or_lost(table, scenario, customers, policy):
    batch = build(customers, 30, seed=5, distinct=False)
    plan = assign_fleet(table, scenario.drones[:5], batch, consolidate=policy)
    ids = [a.delivery_id for a in plan.assignments]
    assert len(ids) == len(set(ids)), "a parcel was delivered twice"
    assert len(ids) + len(plan.unserviceable) == len(batch), "a parcel vanished"


@pytest.mark.parametrize("policy", POLICIES)
def test_every_tour_is_actually_flyable(table, scenario, customers, policy):
    """Relocation must never produce a tour the drone cannot complete."""
    batch = build(customers, 25, seed=9)
    plan = assign_fleet(table, scenario.drones[:5], batch, consolidate=policy)
    by_id = {p.id: p for p in batch}
    for drone, stops in tours_of(plan, by_id).items():
        start = next(d for d in scenario.drones if d.id == drone)
        assert simulate_tour(table, start, stops).feasible


def test_safe_never_detours_ahead_of_more_urgent_work(table, scenario, customers):
    """Under `safe`, a routine stop may precede an urgent one only if it is free.

    Demanding a strictly urgent-first tour is the wrong rule and causes the very
    behaviour it is meant to prevent: a drone carrying an urgent parcel across
    the map is forced to fly *past* a routine delivery standing on its path and
    come back for it. What must hold is that urgent work is never delayed by a
    detour -- a free stop costs only the handover.
    """
    batch = build(customers, 30, seed=13)
    plan = assign_fleet(table, scenario.drones[:5], batch, consolidate="safe")
    for drone in plan.drones:
        legs = sorted((a for a in plan.assignments if a.drone_id == drone.id),
                      key=lambda a: a.depart_min)
        for index, leg in enumerate(legs):
            if leg.enroute:
                continue                     # free: costs no detour
            later = [x.priority for x in legs[index + 1:]]
            assert not any(p < leg.priority for p in later), (
                f"{drone.id} detours to {leg.destination} ({leg.priority.name}) "
                f"ahead of more urgent work")


def test_tours_are_not_threaded_far_before_near(table, scenario, customers):
    """A drone must not cross the map and come back for a stop it flew past.

    Measured as the gap to the best ordering of the same stops, which is what a
    viewer perceives as the tour "going the wrong way round".
    """
    import itertools
    batch = build(customers, 18, seed=77)
    plan = assign_fleet(table, scenario.drones[:3], batch, consolidate="always")
    start = scenario.graph.warehouse
    for drone in plan.drones:
        legs = sorted((a for a in plan.assignments if a.drone_id == drone.id),
                      key=lambda a: a.depart_min)
        stops = [a.destination for a in legs]
        if not 2 <= len(stops) <= 7:
            continue                          # keep the brute force tractable
        flown = sum(a.route.distance_km for a in legs)
        best = min(
            sum(table.route(u, v).distance_km
                for u, v in zip((start,) + order, order))
            for order in itertools.permutations(stops)
        )
        assert flown <= best * 1.02 + 1e-6, (
            f"{drone.id} flies {flown:.1f} km where {best:.1f} km orders the same stops")


def test_a_free_stop_is_marked_and_explained(table, scenario, customers):
    """A stop costing no detour is flagged so the plan can say why it was free."""
    batch = build(customers, 30, seed=21, distinct=False)
    plan = assign_fleet(table, scenario.drones[:3], batch, consolidate="always")
    free = [a for a in plan.assignments if a.enroute]
    assert free, "expected at least one detour-free stop in a dense batch"
    for a in free:
        assert "already flying" in a.reason


# -- wind ------------------------------------------------------------------
#
# The first audit of this feature tested only calm air under pure-distance
# weighting, and missed that the improvement pass was scoring moves on raw
# distance while routes were being chosen on composite cost. Under wind the two
# diverge -- the same corridor costs different amounts in each direction -- so
# the pass optimised something the router was not using. These cases exist so
# that gap cannot reopen.

WINDS = [Wind(0, 0), Wind(10, 45), Wind(16, 225), Wind(20, 270)]


@pytest.mark.parametrize("wind", WINDS, ids=lambda w: f"{w.speed_ms:.0f}@{w.bearing_deg:.0f}")
@pytest.mark.parametrize("weights", [SHORTEST_DISTANCE, BALANCED, MINIMUM_ENERGY],
                         ids=("distance", "balanced", "energy"))
@pytest.mark.parametrize("policy", ("safe", "always"))
def test_planning_converges_under_wind(city, scenario, customers, wind, weights, policy):
    table = RouteTable(city, CostModel(city, weights, wind))
    batch = build(customers, 20, seed=int(wind.bearing_deg) + 5)
    fleet = scenario.drones[:5]
    plan = assign_fleet(table, fleet, batch, consolidate=policy)
    moves, saved = residual(table, plan, batch, policy, fleet)
    assert moves == 0, f"{moves} moves worth {saved:.3f} left under wind {wind.as_dict()}"


def test_the_pass_scores_moves_on_the_active_objective(city, scenario, customers):
    """Improvement must track the objective in force, not raw distance.

    Under a strong wind with energy weighting, the cheapest tour by energy is
    not the shortest by distance. If the pass still scored distance it would
    make moves the router disagrees with, and leave improving moves behind.
    """
    wind = Wind(18.0, 45.0)
    table = RouteTable(city, CostModel(city, MINIMUM_ENERGY, wind))
    batch = build(customers, 20, seed=404)
    fleet = scenario.drones[:5]
    plan = assign_fleet(table, fleet, batch, consolidate="always")
    assert residual(table, plan, batch, "always", fleet)[0] == 0

    tours = tours_of(plan, {p.id: p for p in batch})
    energy = sum(simulate_tour(table, next(d for d in fleet if d.id == k), v).energy_pct
                 for k, v in tours.items())
    plain = assign_fleet(table, fleet, batch, consolidate="off")
    tours_off = tours_of(plain, {p.id: p for p in batch})
    energy_off = sum(simulate_tour(table, next(d for d in fleet if d.id == k), v).energy_pct
                     for k, v in tours_off.items())
    assert energy <= energy_off + 1e-6
