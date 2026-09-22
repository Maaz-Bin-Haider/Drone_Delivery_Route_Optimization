"""Geometry helpers. The A* admissibility proof depends on these properties."""

import math

import pytest

from ddros.geo import (haversine_km, initial_bearing_deg, point_in_polygon,
                       point_segment_distance, project_km, segments_intersect)


def test_haversine_is_zero_for_identical_points():
    assert haversine_km(24.86, 67.00, 24.86, 67.00) == pytest.approx(0.0)


def test_haversine_is_symmetric():
    a = haversine_km(24.83, 66.96, 24.92, 67.06)
    b = haversine_km(24.92, 67.06, 24.83, 66.96)
    assert a == pytest.approx(b)


def test_haversine_satisfies_the_triangle_inequality():
    """This is what makes the heuristic consistent (TDD 7.3.3)."""
    p, q, r = (24.83, 66.96), (24.88, 67.01), (24.92, 67.06)
    assert haversine_km(*p, *r) <= haversine_km(*p, *q) + haversine_km(*q, *r) + 1e-9


def test_bearing_points_the_expected_way():
    assert initial_bearing_deg(0, 0, 1, 0) == pytest.approx(0.0, abs=0.1)     # north
    assert initial_bearing_deg(0, 0, 0, 1) == pytest.approx(90.0, abs=0.1)    # east
    assert initial_bearing_deg(0, 0, -1, 0) == pytest.approx(180.0, abs=0.1)  # south


def test_projection_preserves_short_distances():
    lat0, lon0 = 24.86, 67.00
    a = project_km(24.86, 67.00, lat0, lon0)
    b = project_km(24.87, 67.01, lat0, lon0)
    planar = math.hypot(b[0] - a[0], b[1] - a[1])
    assert planar == pytest.approx(haversine_km(24.86, 67.00, 24.87, 67.01), rel=1e-3)


def test_point_to_segment_distance_clamps_to_the_endpoints():
    assert point_segment_distance((5, 0), (0, 0), (1, 0)) == pytest.approx(4.0)
    assert point_segment_distance((0.5, 3), (0, 0), (1, 0)) == pytest.approx(3.0)


def test_point_in_polygon():
    square = [(0, 0), (2, 0), (2, 2), (0, 2)]
    assert point_in_polygon((1, 1), square)
    assert not point_in_polygon((3, 1), square)
    assert not point_in_polygon((-1, -1), square)


def test_segment_intersection():
    assert segments_intersect((0, 0), (2, 2), (0, 2), (2, 0))
    assert not segments_intersect((0, 0), (1, 0), (0, 1), (1, 1))
