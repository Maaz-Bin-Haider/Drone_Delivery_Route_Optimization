"""JSON API (TDD section 12, SI-1).

This layer is deliberately thin: it validates payloads, calls the simulator and
serialises the result. All decision-making lives in the algorithm tier, which
has no knowledge that a web server exists (design principle P1, NFR-7).
"""

from __future__ import annotations

import csv
import io

from flask import Blueprint, current_app, jsonify, request

from ..analysis import benchmark as bench
from ..cost.cost_model import Weights
from ..domain.models import DeliveryRequest, Priority
from ..environment.wind import Wind
from ..simulation.orchestrator import HeuristicViolation, PlanConfig

api = Blueprint("api", __name__, url_prefix="/api")


class BadRequest(Exception):
    """Invalid input, reported with the offending field named (TDD 12.2)."""

    def __init__(self, message: str, field: str | None = None) -> None:
        super().__init__(message)
        self.message = message
        self.field = field


def simulator():
    return current_app.config["SIMULATOR"]


# --------------------------------------------------------------------------
# payload parsing
# --------------------------------------------------------------------------

def _config_from(payload: dict) -> PlanConfig:
    raw = payload.get("weights") or {}
    try:
        alpha = float(raw.get("alpha", 1.0))
        beta = float(raw.get("beta", 0.0))
        gamma = float(raw.get("gamma", 0.0))
    except (TypeError, ValueError):
        raise BadRequest("weights must be numeric", "weights")

    total = alpha + beta + gamma
    if total <= 0:
        raise BadRequest("weights must not all be zero", "weights")
    # Normalise rather than reject: the sliders can only ever produce a positive
    # triple, and rejecting a rounding error would make the UI feel broken.
    try:
        weights = Weights(alpha / total, beta / total, gamma / total)
    except ValueError as exc:
        raise BadRequest(str(exc), "weights") from exc

    raw_wind = payload.get("wind") or {}
    try:
        wind = Wind(float(raw_wind.get("speed_ms", 0.0)),
                    float(raw_wind.get("bearing_deg", 0.0)))
    except (TypeError, ValueError) as exc:
        raise BadRequest(str(exc), "wind") from exc

    known = {z.id for z in simulator().scenario.zones}
    zones = tuple(payload.get("active_no_fly_zones") or ())
    for zone_id in zones:
        if zone_id not in known:
            raise BadRequest(f"unknown no-fly zone '{zone_id}'", "active_no_fly_zones")

    algorithm = payload.get("algorithm", "astar")
    if algorithm not in ("astar", "dijkstra"):
        raise BadRequest(f"unknown algorithm '{algorithm}'", "algorithm")

    try:
        reserve = float(payload.get("reserve_pct", PlanConfig.reserve_pct))
    except (TypeError, ValueError):
        raise BadRequest("reserve_pct must be numeric", "reserve_pct")
    if not 0.0 <= reserve < 100.0:
        raise BadRequest("reserve_pct must lie in [0, 100)", "reserve_pct")

    return PlanConfig(weights=weights, wind=wind, active_zones=zones,
                      algorithm=algorithm, reserve_pct=reserve)


def _node_or_400(node_id: str, field: str) -> str:
    if node_id not in simulator().scenario.graph.nodes:
        raise BadRequest(f"unknown node '{node_id}'", field)
    return node_id


# --------------------------------------------------------------------------
# endpoints
# --------------------------------------------------------------------------

@api.get("/scenario")
def get_scenario():
    """Everything the client needs to draw the city (FR-1.6, FR-2.4)."""
    scenario = simulator().scenario
    graph = scenario.graph
    seen: set[tuple[str, str]] = set()
    edges = []
    for group in graph.adj.values():
        for e in group:
            key = tuple(sorted((e.u, e.v)))
            if key in seen:
                continue
            seen.add(key)
            edges.append({"u": key[0], "v": key[1],
                          "distance_km": round(e.distance_km, 3)})

    return jsonify({
        "name": scenario.name,
        "nodes": [
            {"id": n.id, "name": n.name, "lat": n.lat, "lon": n.lon,
             "type": n.type.value}
            for n in graph.nodes.values()
        ],
        "edges": edges,
        "drones": [
            {"id": d.id, "current_node": d.current_node, "battery_pct": d.battery_pct}
            for d in scenario.drones
        ],
        "deliveries": [
            {"id": p.id, "destination": p.destination, "priority": p.priority.name}
            for p in scenario.deliveries
        ],
        "zones": [
            {"id": z.id, "name": z.name, "shape": z.shape, "active": z.active,
             "centre": list(z.centre) if z.centre else None,
             "radius_km": z.radius_km,
             "vertices": [list(v) for v in z.vertices] if z.vertices else None}
            for z in scenario.zones
        ],
    })


@api.post("/plan")
def post_plan():
    """Run a full planning cycle (FR-7, FR-10)."""
    config = _config_from(request.get_json(silent=True) or {})
    result = simulator().plan(config)
    current_app.config["LAST_PLAN"] = result
    return jsonify(result)


@api.post("/route")
def post_route():
    """A single point-to-point route (FR-3)."""
    payload = request.get_json(silent=True) or {}
    source = _node_or_400(payload.get("source", ""), "source")
    target = _node_or_400(payload.get("target", ""), "target")
    route = simulator().route(source, target, _config_from(payload))
    if route is None:
        # Unreachable is a valid outcome, not an error (principle P6, FR-3.7).
        return jsonify({"unreachable": True, "source": source, "target": target})
    return jsonify(route.as_dict())


@api.post("/compare")
def post_compare():
    """Dijkstra against A* on one pair (FR-3.4, FR-11.2)."""
    payload = request.get_json(silent=True) or {}
    source = _node_or_400(payload.get("source", ""), "source")
    target = _node_or_400(payload.get("target", ""), "target")
    return jsonify(simulator().compare(source, target, _config_from(payload)))


@api.post("/route/alternatives")
def post_alternatives():
    """Minimum-distance against minimum-energy, side by side (FR-4.7)."""
    payload = request.get_json(silent=True) or {}
    source = _node_or_400(payload.get("source", ""), "source")
    target = _node_or_400(payload.get("target", ""), "target")
    return jsonify(simulator().alternatives(source, target, _config_from(payload)))


@api.post("/deliveries")
def post_delivery():
    """Add a delivery request to the batch (FR-2.4)."""
    payload = request.get_json(silent=True) or {}
    scenario = simulator().scenario
    destination = _node_or_400(payload.get("destination", ""), "destination")

    name = str(payload.get("priority", "NORMAL")).upper()
    if name not in Priority.__members__:
        raise BadRequest(f"unknown priority '{name}'", "priority")

    identifier = payload.get("id") or f"PKG-{len(scenario.deliveries) + 1:03d}"
    if any(p.id == identifier for p in scenario.deliveries):
        raise BadRequest(f"delivery '{identifier}' already exists", "id")

    scenario.deliveries.append(DeliveryRequest(
        identifier, destination, Priority[name], len(scenario.deliveries)))
    return jsonify({"id": identifier, "destination": destination,
                    "priority": name, "count": len(scenario.deliveries)}), 201


@api.post("/benchmark")
def post_benchmark():
    """Run the experimental sweep (FR-11.3, FR-11.5)."""
    payload = request.get_json(silent=True) or {}
    experiment = payload.get("experiment")
    quick = bool(payload.get("quick", True))
    scenario = simulator().scenario

    if experiment is None:
        return jsonify(bench.run_all(scenario, quick=quick))

    try:
        number = int(experiment)
    except (TypeError, ValueError):
        raise BadRequest("experiment must be an integer 1-7", "experiment")
    sizes = (10, 25, 50, 100) if quick else bench.DEFAULT_SIZES
    runners = {
        1: lambda: bench.experiment_1_expansion(sizes=sizes),
        2: lambda: bench.experiment_2_growth(sizes=sizes),
        3: lambda: bench.experiment_3_energy(scenario),
        4: lambda: bench.experiment_4_fleet(scenario),
        5: lambda: bench.experiment_5_greedy_quality(scenario),
        6: lambda: bench.experiment_6_constrained(scenario),
        7: lambda: bench.experiment_7_environment(scenario),
    }
    if number not in runners:
        raise BadRequest("experiment must be an integer 1-7", "experiment")
    return jsonify({f"experiment_{number}": runners[number]()})


EXPORT_COLUMNS = [
    "delivery_id", "priority", "drone_id", "destination", "route",
    "distance_km", "energy_pct", "depart_min", "arrive_min",
    "battery_before_pct", "battery_after_pct", "charging_station",
    "algorithm", "reason",
]


@api.get("/export")
def get_export():
    """Export the most recent plan (FR-10.6, SI-3)."""
    plan = current_app.config.get("LAST_PLAN")
    if plan is None:
        raise BadRequest("no plan has been computed yet", "plan")

    fmt = request.args.get("format", "json").lower()
    if fmt == "json":
        return jsonify(plan)
    if fmt != "csv":
        raise BadRequest(f"unknown export format '{fmt}'", "format")

    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=EXPORT_COLUMNS)
    writer.writeheader()
    for a in plan["assignments"]:
        stop = a.get("charging_stop")
        writer.writerow({
            "delivery_id": a["delivery_id"],
            "priority": a["priority"],
            "drone_id": a["drone_id"],
            "destination": a["destination"],
            "route": " > ".join(a["route"]["path"]),
            "distance_km": a["route"]["distance_km"],
            "energy_pct": a["route"]["energy_pct"],
            "depart_min": a["depart_min"],
            "arrive_min": a["arrive_min"],
            "battery_before_pct": a["battery_before_pct"],
            "battery_after_pct": a["battery_after_pct"],
            "charging_station": stop["station_id"] if stop else "",
            "algorithm": a["route"]["algorithm"],
            "reason": a["reason"],
        })
    return buffer.getvalue(), 200, {
        "Content-Type": "text/csv; charset=utf-8",
        "Content-Disposition": 'attachment; filename="delivery_plan.csv"',
    }


# --------------------------------------------------------------------------
# error handling (TDD 12.2)
# --------------------------------------------------------------------------

@api.errorhandler(BadRequest)
def handle_bad_request(exc: BadRequest):
    body = {"error": exc.message}
    if exc.field:
        body["field"] = exc.field
    return jsonify(body), 400


@api.errorhandler(HeuristicViolation)
def handle_heuristic_violation(exc: HeuristicViolation):
    """Never swallowed: an inadmissible heuristic degrades routes silently."""
    return jsonify({"error": f"heuristic admissibility violated: {exc}"}), 500


@api.errorhandler(KeyError)
def handle_key_error(exc: KeyError):
    return jsonify({"error": str(exc).strip("'\"")}), 400


@api.errorhandler(ValueError)
def handle_value_error(exc: ValueError):
    return jsonify({"error": str(exc)}), 400
