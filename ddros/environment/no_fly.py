"""No-fly zones (TDD section 10.1, FR-8).

A zone never mutates the graph. It produces a mask that the cost model consults,
so the search algorithms remain entirely unaware that restricted airspace exists.
Mask construction is O(E * Z) and is performed once per configuration change.
"""

from __future__ import annotations

from ..domain.models import Edge, NoFlyZone
from ..geo import point_in_polygon, point_segment_distance, project_km, segments_intersect
from ..graph.graph import Graph


class NoFlyMask:
    """Precomputed set of edges and nodes made unusable by active zones."""

    __slots__ = ("_blocked_edges", "_blocked_nodes", "_reasons", "active_ids")

    def __init__(self, graph: Graph, zones: list[NoFlyZone]) -> None:
        self._blocked_edges: set[tuple[str, str]] = set()
        self._blocked_nodes: dict[str, str] = {}
        self._reasons: dict[tuple[str, str], str] = {}
        self.active_ids = tuple(sorted(z.id for z in zones if z.active))

        lat0, lon0 = graph.origin
        for zone in zones:
            if not zone.active:
                continue
            self._apply(graph, zone, lat0, lon0)

    # -- construction -------------------------------------------------------

    def _apply(self, graph: Graph, zone: NoFlyZone, lat0: float, lon0: float) -> None:
        pts = {
            nid: project_km(n.lat, n.lon, lat0, lon0) for nid, n in graph.nodes.items()
        }

        if zone.shape == "circle":
            assert zone.centre is not None and zone.radius_km is not None
            centre = project_km(zone.centre[0], zone.centre[1], lat0, lon0)
            radius = zone.radius_km
            inside = lambda p: (p[0] - centre[0]) ** 2 + (p[1] - centre[1]) ** 2 < radius ** 2
            crosses = lambda a, b: point_segment_distance(centre, a, b) < radius
        elif zone.shape == "polygon":
            assert zone.vertices is not None
            poly = [project_km(la, lo, lat0, lon0) for la, lo in zone.vertices]
            inside = lambda p: point_in_polygon(p, poly)

            def crosses(a, b, _poly=poly):
                if point_in_polygon(a, _poly) or point_in_polygon(b, _poly):
                    return True
                n = len(_poly)
                return any(
                    segments_intersect(a, b, _poly[i], _poly[(i + 1) % n])
                    for i in range(n)
                )
        else:
            raise ValueError(f"zone '{zone.id}' has unknown shape '{zone.shape}'")

        for nid, p in pts.items():
            if inside(p):
                self._blocked_nodes.setdefault(nid, zone.name)

        for edges in graph.adj.values():
            for e in edges:
                key = (e.u, e.v)
                if key in self._blocked_edges:
                    continue
                if crosses(pts[e.u], pts[e.v]):
                    self._blocked_edges.add(key)
                    self._reasons[key] = zone.name

    # -- queries ------------------------------------------------------------

    def is_blocked(self, edge: Edge) -> bool:
        return (edge.u, edge.v) in self._blocked_edges

    def node_blocked_by(self, node_id: str) -> str | None:
        """Name of the zone containing this node, or None."""
        return self._blocked_nodes.get(node_id)

    def __bool__(self) -> bool:
        return bool(self._blocked_edges or self._blocked_nodes)


EMPTY_MASK_ZONES: list[NoFlyZone] = []
