"""Energy feasibility and the charging reroute (TDD section 8)."""

import pytest

from ddros.algorithms.constrained import (charging_reroute, is_feasible,
                                          pareto_constrained_route, recharge_minutes)
from ddros.algorithms.dijkstra import dijkstra, dijkstra_route
from ddros.constants import RESERVE_PCT
from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel


def test_reserve_margin_is_respected(city):
    cm = CostModel(city, SHORTEST_DISTANCE)
    route = dijkstra_route(city, city.warehouse, "L16", cm)
    exactly_enough = route.energy_pct + RESERVE_PCT
    assert is_feasible(route, exactly_enough)
    assert not is_feasible(route, exactly_enough - 0.5)


def test_reroute_matches_brute_force_over_all_stations(city):
    """The two-search optimisation must give the same answer as the naive form."""
    cm = CostModel(city, SHORTEST_DISTANCE)
    source, target, battery = city.warehouse, "L16", 40.0
    fast = charging_reroute(city, source, target, battery, cm)
    assert fast is not None

    usable, full = battery - RESERVE_PCT, 100.0 - RESERVE_PCT
    best, best_cost = None, float("inf")
    for c in city.charging_stations:                       # naive: two runs per station
        leg1 = dijkstra_route(city, source, c, cm)
        leg2 = dijkstra_route(city, c, target, cm)
        if leg1 is None or leg2 is None:
            continue
        if leg1.energy_pct > usable or leg2.energy_pct > full:
            continue
        recharge = recharge_minutes(battery - leg1.energy_pct)
        total = leg1.cost + leg2.cost + cm.weights.gamma * recharge / cm.t_ref
        if total < best_cost:
            best_cost, best = total, c
    assert fast.station_id == best


def test_reroute_legs_are_individually_feasible(city):
    cm = CostModel(city, SHORTEST_DISTANCE)
    r = charging_reroute(city, city.warehouse, "L16", 40.0, cm)
    assert r.leg_one.energy_pct <= 40.0 - RESERVE_PCT + 1e-9
    assert r.leg_two.energy_pct <= 100.0 - RESERVE_PCT + 1e-9
    assert r.path[0] == city.warehouse and r.path[-1] == "L16" and r.station_id in r.path


def test_reroute_returns_none_when_nothing_can_be_reached(city):
    """FR-5.6: an impossible delivery is reported, not silently dropped."""
    cm = CostModel(city, SHORTEST_DISTANCE)
    assert charging_reroute(city, city.warehouse, "L16", RESERVE_PCT + 0.1, cm) is None


def test_exact_variant_never_returns_a_worse_cost(city):
    """The Pareto search is exact, so it can only match or beat the heuristic."""
    cm = CostModel(city, SHORTEST_DISTANCE)
    for target in ("L17", "L16", "L18", "L19"):
        budget = 60.0
        exact = pareto_constrained_route(city, city.warehouse, target, budget, cm)
        direct = dijkstra_route(city, city.warehouse, target, cm)
        if exact is None:
            continue
        assert exact.energy_pct <= budget + 1e-9
        if direct is not None and direct.energy_pct <= budget:
            assert exact.cost <= direct.cost + 1e-9


def test_exact_variant_honours_a_tight_budget(city):
    cm = CostModel(city, SHORTEST_DISTANCE)
    tight = pareto_constrained_route(city, city.warehouse, "L16", 25.0, cm)
    assert tight is None or tight.energy_pct <= 25.0 + 1e-9
