"""End-to-end planning cycle."""

import pytest

from ddros.cost.cost_model import BALANCED, SHORTEST_DISTANCE, Weights
from ddros.environment.wind import Wind
from ddros.simulation.orchestrator import PlanConfig, Simulator


@pytest.fixture(scope="module")
def sim(scenario):
    return Simulator(scenario)


def test_plan_delivers_the_whole_batch(sim, scenario):
    result = sim.plan(PlanConfig())
    assert result["totals"]["delivered"] == len(scenario.deliveries)
    assert result["totals"]["unserviceable"] == 0
    assert result["makespan_min"] > 0


def strip_volatile(obj):
    """Remove wall-clock and cache figures.

    NFR-11 constrains the decisions the system makes, not the telemetry that
    describes how long they took; measured runtimes can never be byte-identical.
    """
    if isinstance(obj, dict):
        return {k: strip_volatile(v) for k, v in obj.items()
                if k not in ("runtime_ms", "telemetry")}
    if isinstance(obj, list):
        return [strip_volatile(v) for v in obj]
    return obj


def test_identical_inputs_give_identical_output(sim):
    """NFR-11: determinism, which the tie-breaking rules exist to guarantee."""
    config = PlanConfig(weights=BALANCED, wind=Wind(7, 120))
    first = strip_volatile(sim.plan(config))
    second = strip_volatile(sim.plan(config))
    assert first == second


def test_planning_does_not_mutate_the_stored_scenario(sim, scenario):
    before = [(d.id, d.battery_pct, d.current_node) for d in scenario.drones]
    sim.plan(PlanConfig())
    sim.plan(PlanConfig(weights=BALANCED))
    after = [(d.id, d.battery_pct, d.current_node) for d in scenario.drones]
    assert before == after
    assert all(p.status.name == "PENDING" for p in scenario.deliveries)


def test_cache_is_reused_for_a_repeated_configuration(scenario):
    from ddros.simulation.orchestrator import Simulator as S
    local = S(scenario)
    config = PlanConfig(weights=BALANCED)
    local.plan(config)
    hits_before = local.cache.hits
    local.plan(config)
    assert local.cache.hits == hits_before + 1


def test_active_zone_makes_its_interior_unserviceable(sim):
    result = sim.plan(PlanConfig(active_zones=("nfz_airport",)))
    assert result["totals"]["unserviceable"] >= 1
    assert any("no-fly zone" in u["reason"] for u in result["unserviceable"])


def test_comparison_endpoint_confirms_the_two_routers_agree(sim):
    """FR-11.6: a live admissibility check on every comparison."""
    out = sim.compare("W", "C8", PlanConfig(weights=BALANCED, wind=Wind(13, 200)))
    assert out["agree"]
    assert out["expansion_ratio"] <= 1.0


def test_energy_weighting_reduces_energy_on_the_same_journey(sim, city):
    """FR-4.7: for a fixed origin and destination, weighting energy cannot cost more.

    The claim is deliberately scoped to a single journey. At batch level it does
    NOT hold: energy-optimal routes are slower, which delays drones and can
    trigger an extra charging detour costing a whole additional leg. Experiment 3
    measures that effect rather than assuming it away.
    """
    wind = Wind(16, 225)
    shortest = PlanConfig(weights=SHORTEST_DISTANCE, wind=wind)
    greenest = PlanConfig(weights=Weights(0.0, 1.0, 0.0), wind=wind)
    for target in ("C3", "C4", "C8", "C10"):
        a = sim.route("W", target, shortest)
        b = sim.route("W", target, greenest)
        assert b.energy_pct <= a.energy_pct + 1e-9, target


def test_alternatives_can_diverge_under_wind(sim):
    """The map must actually exhibit the Route A / Route B case (TDD 10.2)."""
    out = sim.alternatives("C3", "C4", PlanConfig(wind=Wind(16, 225)))
    assert out["differ"], "expected the shortest and most efficient routes to differ"
    assert "less energy" in out["annotation"]


def test_wind_alone_can_change_the_chosen_route(sim):
    """FR-9.6."""
    paths = set()
    for bearing in range(0, 360, 45):
        route = sim.route("C11", "C12", PlanConfig(weights=Weights(0.0, 1.0, 0.0),
                                                   wind=Wind(16, bearing)))
        paths.add(route.path)
    assert len(paths) > 1, "wind bearing had no effect on the optimal route"
