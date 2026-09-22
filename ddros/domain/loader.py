"""Scenario loading (FR-1.6, FR-2.4, SI-2).

Every failure names the offending record, so a malformed scenario file produces
an actionable diagnostic rather than a stack trace.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from ..graph.graph import Graph
from ..graph.validation import ScenarioValidationError, validate_graph, validate_scenario
from .models import (DeliveryRequest, Drone, Node, NodeType, NoFlyZone, Priority)


@dataclass
class Scenario:
    """Everything a planning cycle needs, validated and ready to use."""

    name: str
    graph: Graph
    drones: list[Drone]
    deliveries: list[DeliveryRequest]
    zones: list[NoFlyZone]
    backdrop: dict = field(default_factory=dict)
    presets: list[dict] = field(default_factory=list)

    def replace_deliveries(self, specs: list[dict]) -> list[DeliveryRequest]:
        """Swap the batch for a new one, renumbering ids and sequence.

        The sequence number is the priority queue's tie-breaker, so it must be
        rebuilt from scratch rather than continued: a loaded batch should
        dispatch in the order it was written, not in the order of whatever it
        replaced.
        """
        batch = []
        for i, spec in enumerate(specs):
            destination = spec["destination"]
            if destination not in self.graph.nodes:
                raise ScenarioValidationError(
                    f"delivery #{i + 1} targets unknown destination '{destination}'")
            name = str(spec.get("priority", "NORMAL")).upper()
            if name not in Priority.__members__:
                raise ScenarioValidationError(
                    f"delivery #{i + 1} has unknown priority '{name}'")
            batch.append(DeliveryRequest(
                spec.get("id") or f"PKG-{i + 1:03d}", destination, Priority[name], i))
        self.deliveries[:] = batch
        return batch

    def preset(self, preset_id: str) -> dict:
        for p in self.presets:
            if p["id"] == preset_id:
                return p
        raise KeyError(f"unknown preset '{preset_id}'")

    def zone(self, zone_id: str) -> NoFlyZone:
        for z in self.zones:
            if z.id == zone_id:
                return z
        raise KeyError(f"unknown no-fly zone '{zone_id}'")


def _read(path: Path) -> dict:
    try:
        with path.open(encoding="utf-8") as fh:
            return json.load(fh)
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"scenario file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path.name} is not valid JSON: {exc}") from exc


def load_graph(path: Path) -> tuple[str, Graph]:
    raw = _read(path)
    nodes = []
    for i, n in enumerate(raw.get("nodes", [])):
        try:
            nodes.append(Node(n["id"], n["name"], float(n["lat"]),
                              float(n["lon"]), NodeType(n["type"]),
                              n.get("district", "")))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path.name}: node #{i} is malformed: {exc}") from exc
    graph = Graph.build(nodes, raw.get("edges", []))
    validate_graph(graph)
    return raw.get("name", "Unnamed"), graph


def load_fleet(path: Path) -> list[Drone]:
    raw = _read(path)
    out = []
    for i, d in enumerate(raw.get("drones", [])):
        try:
            out.append(Drone(d["id"], d["current_node"], float(d["battery_pct"])))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path.name}: drone #{i} is malformed: {exc}") from exc
    return out


def load_deliveries(path: Path) -> list[DeliveryRequest]:
    raw = _read(path)
    out = []
    for i, p in enumerate(raw.get("deliveries", [])):
        try:
            out.append(DeliveryRequest(p["id"], p["destination"],
                                       Priority[p["priority"].upper()], i))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path.name}: delivery #{i} is malformed: {exc}") from exc
    return out


def load_zones(path: Path) -> list[NoFlyZone]:
    if not path.exists():
        return []
    raw = _read(path)
    out = []
    for i, z in enumerate(raw.get("no_fly_zones", [])):
        try:
            verts = z.get("vertices")
            out.append(NoFlyZone(
                id=z["id"], name=z["name"], shape=z["shape"],
                active=bool(z.get("active", False)),
                centre=tuple(z["centre"]) if z.get("centre") else None,
                radius_km=z.get("radius_km"),
                vertices=tuple(tuple(v) for v in verts) if verts else None,
            ))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path.name}: zone #{i} is malformed: {exc}") from exc
    return out


def load_json_if_present(path: Path, key: str, default):
    """Optional scenario files degrade to a default rather than failing."""
    if not path.exists():
        return default
    return _read(path).get(key, default)


def load_scenario(data_dir: str | Path = "data") -> Scenario:
    d = Path(data_dir)
    name, graph = load_graph(d / "city_map.json")
    drones = load_fleet(d / "fleet.json")
    deliveries = load_deliveries(d / "deliveries.json")
    zones = load_zones(d / "no_fly_zones.json")
    validate_scenario(graph, drones, deliveries)

    backdrop = _read(d / "city_backdrop.json") if (d / "city_backdrop.json").exists() else {}
    presets = load_json_if_present(d / "presets.json", "presets", [])
    return Scenario(name, graph, drones, deliveries, zones, backdrop, presets)
