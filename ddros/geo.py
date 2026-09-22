"""Spherical and planar geometry helpers.

These are implemented directly rather than imported because the A* admissibility
argument in TDD section 7.3.2 depends on the exact properties of `haversine`,
and because the no-fly-zone tests need planar geometry in a local projection.
"""

from __future__ import annotations

import math

from .constants import EARTH_RADIUS_KM, EPS

Point = tuple[float, float]


def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in kilometres.

    This is a metric: it is symmetric, non-negative, and satisfies the triangle
    inequality. The A* heuristic relies on all three.
    """
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_KM * math.asin(math.sqrt(min(1.0, a)))


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2, degrees clockwise from true north."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def project_km(lat: float, lon: float, lat0: float, lon0: float) -> Point:
    """Equirectangular projection about (lat0, lon0), returning kilometres.

    Accurate to well under a metre at city scale, and far cheaper than exact
    spherical geometry for the segment-intersection tests of TDD section 10.1.
    """
    x = EARTH_RADIUS_KM * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_RADIUS_KM * math.radians(lat - lat0)
    return x, y


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Shortest distance from point `p` to segment `ab`, in the units supplied."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom < EPS:                       # degenerate segment
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))             # clamp to the segment
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_in_polygon(p: Point, polygon: list[Point]) -> bool:
    """Ray-casting parity test. O(n) in the vertex count."""
    px, py = p
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            x_cross = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < x_cross:
                inside = not inside
    return inside


def _orientation(a: Point, b: Point, c: Point) -> int:
    v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if v > EPS:
        return 1
    if v < -EPS:
        return -1
    return 0


def _on_segment(a: Point, b: Point, c: Point) -> bool:
    return (min(a[0], b[0]) - EPS <= c[0] <= max(a[0], b[0]) + EPS
            and min(a[1], b[1]) - EPS <= c[1] <= max(a[1], b[1]) + EPS)


def segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    """True if segment p1p2 intersects segment p3p4, including collinear touching."""
    o1 = _orientation(p1, p2, p3)
    o2 = _orientation(p1, p2, p4)
    o3 = _orientation(p3, p4, p1)
    o4 = _orientation(p3, p4, p2)
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and _on_segment(p1, p2, p3):
        return True
    if o2 == 0 and _on_segment(p1, p2, p4):
        return True
    if o3 == 0 and _on_segment(p3, p4, p1):
        return True
    if o4 == 0 and _on_segment(p3, p4, p2):
        return True
    return False
