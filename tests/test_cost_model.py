"""The cost model underpins Dijkstra's correctness, so its invariants matter."""

import itertools

import pytest

from ddros.constants import MU_MAX, MU_MIN, V_AIR_MS
from ddros.cost.cost_model import BALANCED, SHORTEST_DISTANCE, CostModel, Weights
from ddros.environment.wind import Wind


def test_weights_must_lie_on_the_unit_simplex():
    with pytest.raises(ValueError, match="must equal 1"):
        Weights(0.5, 0.3, 0.3)
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        Weights(-0.2, 0.7, 0.5)


@pytest.mark.parametrize("speed,bearing", [(0, 0), (8, 45), (16, 180), (20, 300)])
def test_edge_cost_is_never_negative(city, speed, bearing):
    """Dijkstra's precondition (TDD 6.4). Without this the routing layer is unsound."""
    wind = Wind(speed, bearing)
    for a in (0.0, 0.25, 0.5, 1.0):
        for b in (0.0, 0.25, 0.5, 1.0):
            if a + b > 1.0:
                continue
            cm = CostModel(city, Weights(a, b, 1.0 - a - b), wind)
            for edges in city.adj.values():
                for e in edges:
                    assert cm.edge_cost(e) >= 0.0


def test_tailwind_costs_less_energy_than_headwind(city):
    """The direction-dependence that makes the wind model meaningful (FR-9.3)."""
    wind = Wind(12.0, 90.0)                     # blowing toward the east
    assert wind.energy_multiplier(90) < 1.0     # flying east: tailwind
    assert wind.energy_multiplier(270) > 1.0    # flying west: headwind
    assert wind.energy_multiplier(0) == pytest.approx(1.0)   # crosswind


def test_wind_is_symmetric_under_reversal():
    """Reversing the bearing swaps head- and tailwind (FR-9.2)."""
    a, b = Wind(10, 0), Wind(10, 180)
    assert a.along_track_ms(0) == pytest.approx(-b.along_track_ms(0))


def test_energy_multiplier_stays_clamped(city):
    for bearing in range(0, 360, 15):
        mu = Wind(20.0, 0.0).energy_multiplier(bearing)
        assert MU_MIN <= mu <= MU_MAX


def test_calm_air_leaves_base_values_untouched(city):
    cm = CostModel(city, SHORTEST_DISTANCE)
    e = next(iter(city.adj["W"]))
    assert cm.edge_energy(e) == pytest.approx(e.base_energy_pct)


def test_config_key_distinguishes_configurations(city):
    base = CostModel(city, SHORTEST_DISTANCE, Wind(5, 90)).config_key()
    assert CostModel(city, BALANCED, Wind(5, 90)).config_key() != base
    assert CostModel(city, SHORTEST_DISTANCE, Wind(6, 90)).config_key() != base
    assert CostModel(city, SHORTEST_DISTANCE, Wind(5, 90)).config_key() == base
