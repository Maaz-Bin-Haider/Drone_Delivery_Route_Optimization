"""Synthetic graph generation for the benchmark harness (TDD section 14 note).

The application itself always uses the fixed demonstration map. These generators
exist only so the harness can vary V and E as FR-11.3 requires; they are never
surfaced in the user interface.

Edge distances are always derived from the generated coordinates by haversine,
never authored. That keeps the A* heuristic admissible on synthetic graphs for
exactly the same reason it is admissible on the demonstration map, so the
comparison in Experiment 1 measures the heuristic rather than an artefact.
"""

from __future__ import annotations

import random
from collections import deque

from ..domain.models import Node, NodeType
from ..geo import haversine_km
from ..graph.graph import Graph

# A bounding box roughly 10 km on a side, matching the demonstration city so
# that edge lengths and therefore energy costs stay in a comparable range.
LAT_MIN, LAT_MAX = 24.830, 24.920
LON_MIN, LON_MAX = 66.960, 67.060


def random_city(node_count: int, seed: int = 0, neighbours: int = 4) -> Graph:
    """A connected geographic graph of `node_count` nodes.

    Nodes are scattered uniformly and joined to their `neighbours` nearest
    peers, which produces the sparse, locally-clustered structure of a real
    road or corridor network rather than a uniform random graph. Any components
    the nearest-neighbour step leaves disconnected are then bridged, so the
    result always satisfies the connectivity precondition of the router.
    """
    if node_count < 3:
        raise ValueError("a city needs at least three nodes")

    rng = random.Random(seed)
    nodes: list[Node] = []
    for i in range(node_count):
        lat = rng.uniform(LAT_MIN, LAT_MAX)
        lon = rng.uniform(LON_MIN, LON_MAX)
        if i == 0:
            kind = NodeType.WAREHOUSE
        elif i == 1 or i % 9 == 0:
            # Node 1 is always a charger, so even the smallest graph satisfies
            # the validator's requirement of at least one charging station.
            kind = NodeType.CHARGING_STATION
        else:
            kind = NodeType.CUSTOMER
        nodes.append(Node(f"N{i}", f"Node {i}", lat, lon, kind))

    coords = [(n.lat, n.lon) for n in nodes]
    pairs: set[tuple[str, str]] = set()
    k = min(neighbours, node_count - 1)

    for i in range(node_count):
        lat_i, lon_i = coords[i]
        distances = sorted(
            ((haversine_km(lat_i, lon_i, coords[j][0], coords[j][1]), j)
             for j in range(node_count) if j != i),
            key=lambda t: t[0],
        )
        for _d, j in distances[:k]:
            a, b = sorted((nodes[i].id, nodes[j].id))
            pairs.add((a, b))

    _bridge_components(nodes, coords, pairs)
    return Graph.build(nodes, [{"u": a, "v": b} for a, b in sorted(pairs)])


def _bridge_components(nodes: list[Node], coords: list[tuple[float, float]],
                       pairs: set[tuple[str, str]]) -> None:
    """Join disconnected components by their closest pair, until one remains."""
    index = {n.id: i for i, n in enumerate(nodes)}
    while True:
        adj: dict[str, list[str]] = {n.id: [] for n in nodes}
        for a, b in pairs:
            adj[a].append(b)
            adj[b].append(a)

        seen: set[str] = set()
        components: list[list[str]] = []
        for n in nodes:
            if n.id in seen:
                continue
            group, queue = [], deque([n.id])
            seen.add(n.id)
            while queue:
                u = queue.popleft()
                group.append(u)
                for v in adj[u]:
                    if v not in seen:
                        seen.add(v)
                        queue.append(v)
            components.append(group)

        if len(components) == 1:
            return

        head, rest = components[0], [x for g in components[1:] for x in g]
        best = min(
            ((haversine_km(*coords[index[a]], *coords[index[b]]), a, b)
             for a in head for b in rest),
            key=lambda t: t[0],
        )
        pairs.add(tuple(sorted((best[1], best[2]))))


def random_pairs(graph: Graph, count: int, seed: int = 0) -> list[tuple[str, str]]:
    """Distinct source/target pairs, sampled reproducibly."""
    rng = random.Random(seed)
    ids = sorted(graph.nodes)
    out: list[tuple[str, str]] = []
    guard = 0
    while len(out) < count and guard < count * 50:
        guard += 1
        a, b = rng.choice(ids), rng.choice(ids)
        if a != b and (a, b) not in out:
            out.append((a, b))
    return out
