"""The harness produces the evidence for every complexity claim, so it needs
checking as carefully as the algorithms it measures."""

import itertools

import pytest

from ddros.analysis import benchmark as bench
from ddros.analysis.generators import random_city, random_pairs
from ddros.constants import SERVICE_TIME_MIN
from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.graph.validation import validate_graph
from ddros.simulation.cache import RouteTable


# -- generators ------------------------------------------------------------

@pytest.mark.parametrize("size", [3, 10, 50, 200])
def test_generated_cities_are_valid_and_connected(size):
    graph = random_city(size, seed=7)
    validate_graph(graph)                      # raises if disconnected or malformed
    assert len(graph.nodes) == size


def test_generation_is_reproducible():
    """Determinism (principle P5) must hold for the harness too."""
    a, b = random_city(40, seed=3), random_city(40, seed=3)
    assert sorted(a.nodes) == sorted(b.nodes)
    assert a.edge_count() == b.edge_count()
    assert random_city(40, seed=4).edge_count() != 0


def test_generated_distances_come_from_coordinates():
    """What keeps the A* heuristic admissible on synthetic graphs."""
    from ddros.geo import haversine_km
    graph = random_city(30, seed=11)
    for edges in graph.adj.values():
        for e in edges:
            u, v = graph.nodes[e.u], graph.nodes[e.v]
            assert e.distance_km == pytest.approx(
                haversine_km(u.lat, u.lon, v.lat, v.lon))


def test_random_pairs_are_distinct_and_valid():
    graph = random_city(30, seed=5)
    pairs = random_pairs(graph, 20, seed=5)
    assert len(pairs) == len(set(pairs)) == 20
    assert all(a != b and a in graph.nodes and b in graph.nodes for a, b in pairs)


# -- statistics ------------------------------------------------------------

def test_summarise_reports_spread_not_just_a_mean():
    """FR-11.4: a single noisy run must not be mistaken for a trend."""
    assert bench.summarise([2.0, 4.0, 6.0])["mean"] == pytest.approx(4.0)
    assert bench.summarise([2.0, 4.0, 6.0])["stdev"] == pytest.approx(2.0)
    assert bench.summarise([])["n"] == 0
    assert bench.summarise([5.0])["stdev"] == 0.0


def test_linear_fit_recovers_a_known_line():
    fit = bench.linear_fit([1, 2, 3, 4], [3, 5, 7, 9])      # y = 2x + 1
    assert fit["slope"] == pytest.approx(2.0)
    assert fit["intercept"] == pytest.approx(1.0)
    assert fit["r_squared"] == pytest.approx(1.0)


def test_linear_fit_reports_poor_agreement_honestly():
    fit = bench.linear_fit([1, 2, 3, 4], [5, 1, 6, 2])
    assert fit["r_squared"] < 0.5


# -- the optimal-makespan reference ---------------------------------------

def _brute_force_makespan(table, warehouse, destinations, drone_count):
    """Enumerate every assignment and every ordering. Exponential, but exact.

    This is the reference the dynamic-programming optimum is checked against:
    if Held-Karp has a bug, the greedy quality figure of Experiment 5 is
    meaningless, so it cannot be left unverified.
    """
    best = float("inf")
    for assignment in itertools.product(range(drone_count), repeat=len(destinations)):
        groups = [[] for _ in range(drone_count)]
        for pkg, drone in enumerate(assignment):
            groups[drone].append(destinations[pkg])
        makespan = 0.0
        for group in groups:
            if not group:
                continue
            shortest = min(
                sum(table.time(a, b) + SERVICE_TIME_MIN
                    for a, b in zip((warehouse,) + order, order))
                for order in itertools.permutations(group)
            )
            makespan = max(makespan, shortest)
        best = min(best, makespan)
    return best


@pytest.mark.parametrize("packages,drones", [(3, 2), (4, 2), (4, 3)])
def test_held_karp_matches_exhaustive_enumeration(city, packages, drones):
    table = RouteTable(city, CostModel(city, SHORTEST_DISTANCE))
    destinations = ["C1", "C3", "C5", "C7"][:packages]
    fast = bench._optimal_makespan(table, city.warehouse, destinations, drones)
    slow = _brute_force_makespan(table, city.warehouse, destinations, drones)
    assert fast == pytest.approx(slow)


# -- experiments -----------------------------------------------------------

def test_experiment_1_confirms_the_two_routers_agree():
    """A cost disagreement would raise, so reaching the assertions is the check."""
    out = bench.experiment_1_expansion(sizes=(10, 25), pairs=8, repeats=2)
    assert out["costs_agreed_everywhere"]
    for row in out["series"]:
        w = row["weights"]["distance"]
        assert w["astar_expanded"]["mean"] <= w["dijkstra_expanded"]["mean"]
        assert 0 < w["expansion_ratio"]["mean"] <= 1.0


def test_experiment_1_heuristic_weakens_as_the_objective_leaves_distance():
    """The prediction made in the TDD design note of section 7.3."""
    out = bench.experiment_1_expansion(sizes=(50, 100), pairs=12, repeats=1)
    ratios = {
        label: sum(r["weights"][label]["expansion_ratio"]["mean"] for r in out["series"])
        for label, _ in bench.WEIGHT_SETTINGS
    }
    assert ratios["distance"] < ratios["energy"]


def test_experiment_2_runtime_tracks_the_derived_bound():
    data = bench.experiment_1_expansion(sizes=(25, 50, 100, 250), pairs=10, repeats=3)
    fit = bench.experiment_2_growth(data)
    assert fit["dijkstra_fit"]["r_squared"] > 0.8
    assert fit["dijkstra_fit"]["slope"] > 0


def test_experiment_3_separates_journey_from_batch(scenario):
    out = bench.experiment_3_energy(scenario, betas=(0.0, 0.5, 1.0))
    assert out["journey_energy_monotonic"], "per-journey claim should hold"
    assert len(out["per_batch"]) == 3
    assert all("charging_detours" in row for row in out["per_batch"])


def test_experiment_4_exposes_the_cause_of_superlinearity(scenario):
    out = bench.experiment_4_fleet(scenario, max_drones=3)
    assert out["rows"][0]["speedup"] == pytest.approx(1.0)
    for row in out["rows"]:
        assert "charging_detours" in row and "flight_min" in row
    assert out["rows"][-1]["makespan_min"] <= out["rows"][0]["makespan_min"]


def test_experiment_5_greedy_never_beats_optimal(scenario):
    """A ratio below 1 would mean the reference optimum is wrong."""
    out = bench.experiment_5_greedy_quality(scenario, instances=15, packages=5)
    assert out["ratio"]["mean"] >= 1.0 - 1e-9
    assert out["worst_case"]["ratio"] >= 1.0 - 1e-9


def test_experiment_6_approximation_is_conservative_never_unsafe(scenario):
    """It may refuse a possible delivery; it must never promise an impossible one."""
    out = bench.experiment_6_constrained(scenario, instances=40)
    assert out["disagreements"] == out["false_infeasible_count"], (
        "a disagreement in the other direction would mean the cheap test "
        "reported a route as feasible when none exists")


def test_experiment_7_detects_wind_sensitivity(scenario):
    out = bench.experiment_7_environment(scenario, bearing_step=45)
    assert out["wind_changes_route"], "FR-9.6 requires a demonstrable case"
    assert out["no_fly_penalties"]


def test_full_quick_sweep_runs_within_budget(scenario):
    """NFR-5 budgets the sweep at five minutes; the quick form is far smaller."""
    out = bench.run_all(scenario, quick=True)
    assert set(out) >= {f"experiment_{i}" for i in range(1, 8)}
    assert out["elapsed_s"] < 300
