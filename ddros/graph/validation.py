"""Scenario validation (TDD section 5.5, FR-1.7).

Every failure names the offending element, so a malformed scenario file produces
a diagnostic the author can act on rather than a stack trace.
"""

from __future__ import annotations

from collections import deque

from ..domain.models import DeliveryRequest, Drone, NodeType
from .graph import Graph


class MapValidationError(ValueError):
    """The city map is structurally invalid."""


class ScenarioValidationError(ValueError):
    """The fleet or delivery batch is inconsistent with the map."""


def validate_graph(graph: Graph) -> None:
    if not graph.nodes:
        raise MapValidationError("map contains no nodes")

    warehouses = [n.id for n in graph.nodes.values() if n.type is NodeType.WAREHOUSE]
    if len(warehouses) != 1:
        raise MapValidationError(
            f"map must contain exactly one warehouse, found {len(warehouses)}: {warehouses}"
        )
    if not graph.charging_stations:
        raise MapValidationError("map must contain at least one charging station")

    for node_id, edges in graph.adj.items():
        for e in edges:
            if e.u == e.v:
                raise MapValidationError(f"self-loop at node '{e.u}'")
            if e.v not in graph.nodes:
                raise MapValidationError(f"edge {e.u}->{e.v} references an unknown node")
            if e.distance_km <= 0:
                raise MapValidationError(
                    f"edge {e.u}->{e.v} has non-positive distance {e.distance_km}"
                )
            if e.base_energy_pct < 0:
                raise MapValidationError(
                    f"edge {e.u}->{e.v} has negative energy {e.base_energy_pct}"
                )
        if node_id not in graph.nodes:
            raise MapValidationError(f"adjacency list references unknown node '{node_id}'")

    _require_connected(graph)


def _require_connected(graph: Graph) -> None:
    """Breadth-first reachability check in O(V + E)."""
    start = next(iter(graph.nodes))
    seen = {start}
    queue = deque([start])
    while queue:
        u = queue.popleft()
        for e in graph.adj[u]:
            if e.v not in seen:
                seen.add(e.v)
                queue.append(e.v)
    if len(seen) != len(graph.nodes):
        unreachable = sorted(set(graph.nodes) - seen)
        raise MapValidationError(
            f"map is not connected; unreachable from '{start}': {unreachable}"
        )


def validate_scenario(graph: Graph, drones: list[Drone],
                      deliveries: list[DeliveryRequest]) -> None:
    if not drones:
        raise ScenarioValidationError("fleet is empty")

    seen_drones: set[str] = set()
    for d in drones:
        if d.id in seen_drones:
            raise ScenarioValidationError(f"duplicate drone id '{d.id}'")
        seen_drones.add(d.id)
        if d.current_node not in graph.nodes:
            raise ScenarioValidationError(
                f"drone '{d.id}' starts at unknown node '{d.current_node}'"
            )
        if not 0.0 <= d.battery_pct <= 100.0:
            raise ScenarioValidationError(
                f"drone '{d.id}' has out-of-range battery {d.battery_pct}"
            )

    seen_deliveries: set[str] = set()
    for p in deliveries:
        if p.id in seen_deliveries:
            raise ScenarioValidationError(f"duplicate delivery id '{p.id}'")
        seen_deliveries.add(p.id)
        if p.destination not in graph.nodes:
            raise ScenarioValidationError(
                f"delivery '{p.id}' targets unknown destination '{p.destination}'"
            )
