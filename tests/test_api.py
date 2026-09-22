"""API contract tests (TDD section 12).

The web tier owns no decision-making, so these check the contract: payloads are
validated, errors name the offending field, and outcomes that are valid but
negative -- an unreachable target, a blocked destination -- are reported as
results rather than as failures.
"""

from pathlib import Path

import pytest

from ddros.web.app import create_app

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture()
def client():
    """A fresh app per test: some endpoints mutate the delivery batch."""
    app = create_app(ROOT / "data")
    app.config["TESTING"] = True
    return app.test_client()


# -- scenario --------------------------------------------------------------

def test_scenario_describes_the_whole_city(client):
    body = client.get("/api/scenario").get_json()
    assert len(body["nodes"]) >= 20                       # FR-1.8
    assert body["edges"] and body["drones"] and body["deliveries"]
    assert {n["type"] for n in body["nodes"]} >= {"warehouse", "charging_station", "customer"}


def test_each_corridor_is_listed_once(client):
    """The graph stores both directions; the client should draw one line."""
    edges = client.get("/api/scenario").get_json()["edges"]
    keys = [tuple(sorted((e["u"], e["v"]))) for e in edges]
    assert len(keys) == len(set(keys))


# -- planning --------------------------------------------------------------

def test_plan_returns_a_complete_schedule(client):
    body = client.post("/api/plan", json={}).get_json()
    assert body["totals"]["delivered"] > 0
    assert body["makespan_min"] > 0
    for a in body["assignments"]:
        assert a["reason"], "every decision must explain itself (FR-10.7)"
        assert a["route"]["path"][-1] == a["destination"]


def test_weights_are_normalised_not_rejected(client):
    """The sliders can only produce a positive triple; rounding must not 400."""
    body = client.post("/api/plan",
                       json={"weights": {"alpha": 3, "beta": 1, "gamma": 1}}).get_json()
    w = body["config"]["weights"]
    assert w["alpha"] + w["beta"] + w["gamma"] == pytest.approx(1.0)
    assert w["alpha"] == pytest.approx(0.6)


def test_activating_a_zone_changes_the_plan(client):
    free = client.post("/api/plan", json={}).get_json()
    masked = client.post("/api/plan",
                         json={"active_no_fly_zones": ["nfz_airport"]}).get_json()
    assert masked["totals"]["unserviceable"] > free["totals"]["unserviceable"]
    assert any("no-fly zone" in u["reason"] for u in masked["unserviceable"])


# -- error handling (TDD 12.2) --------------------------------------------

def test_zero_weights_are_rejected_with_the_field_named(client):
    response = client.post("/api/plan",
                           json={"weights": {"alpha": 0, "beta": 0, "gamma": 0}})
    assert response.status_code == 400
    assert response.get_json()["field"] == "weights"


def test_unknown_node_is_rejected_with_the_field_named(client):
    response = client.post("/api/route", json={"source": "W", "target": "ghost"})
    assert response.status_code == 400
    body = response.get_json()
    assert body["field"] == "target" and "ghost" in body["error"]


def test_unknown_zone_is_rejected(client):
    response = client.post("/api/plan", json={"active_no_fly_zones": ["nope"]})
    assert response.status_code == 400
    assert response.get_json()["field"] == "active_no_fly_zones"


def test_unknown_algorithm_is_rejected(client):
    response = client.post("/api/plan", json={"algorithm": "bogosort"})
    assert response.status_code == 400
    assert response.get_json()["field"] == "algorithm"


def test_out_of_range_wind_is_rejected(client):
    response = client.post("/api/plan", json={"wind": {"speed_ms": 500}})
    assert response.status_code == 400
    assert response.get_json()["field"] == "wind"


def test_reserve_outside_range_is_rejected(client):
    assert client.post("/api/plan", json={"reserve_pct": 120}).status_code == 400


# -- routes ----------------------------------------------------------------

def test_compare_confirms_the_routers_agree(client):
    body = client.post("/api/compare", json={"source": "W", "target": "C8"}).get_json()
    assert body["agree"] is True
    assert body["astar"]["stats"]["nodes_expanded"] <= body["dijkstra"]["stats"]["nodes_expanded"]


def test_alternatives_report_both_routes(client):
    body = client.post("/api/route/alternatives",
                       json={"source": "C3", "target": "C4",
                             "wind": {"speed_ms": 16, "bearing_deg": 225}}).get_json()
    assert body["min_distance"] and body["min_energy"]
    assert body["differ"] is True
    assert "less energy" in body["annotation"]


def test_single_route_honours_the_selected_algorithm(client):
    body = client.post("/api/route", json={"source": "W", "target": "C8",
                                           "algorithm": "dijkstra"}).get_json()
    assert body["algorithm"] == "dijkstra"


# -- deliveries ------------------------------------------------------------

def test_a_delivery_can_be_added(client):
    before = len(client.get("/api/scenario").get_json()["deliveries"])
    response = client.post("/api/deliveries",
                           json={"destination": "C6", "priority": "urgent"})
    assert response.status_code == 201
    assert response.get_json()["count"] == before + 1
    assert len(client.get("/api/scenario").get_json()["deliveries"]) == before + 1


def test_duplicate_delivery_id_is_rejected(client):
    client.post("/api/deliveries", json={"id": "PKG-X", "destination": "C6"})
    assert client.post("/api/deliveries",
                       json={"id": "PKG-X", "destination": "C6"}).status_code == 400


def test_unknown_priority_is_rejected(client):
    response = client.post("/api/deliveries",
                           json={"destination": "C6", "priority": "whenever"})
    assert response.status_code == 400
    assert response.get_json()["field"] == "priority"


# -- export ----------------------------------------------------------------

def test_export_requires_a_plan_first(client):
    assert client.get("/api/export?format=csv").status_code == 400


def test_csv_export_carries_every_assignment(client):
    plan = client.post("/api/plan", json={}).get_json()
    response = client.get("/api/export?format=csv")
    assert response.status_code == 200
    assert "text/csv" in response.headers["Content-Type"]
    lines = response.get_data(as_text=True).strip().splitlines()
    assert len(lines) == len(plan["assignments"]) + 1          # header
    assert lines[0].startswith("delivery_id,priority,drone_id")


def test_unknown_export_format_is_rejected(client):
    client.post("/api/plan", json={})
    assert client.get("/api/export?format=pdf").status_code == 400


# -- benchmark and page ----------------------------------------------------

def test_benchmark_runs_a_single_experiment(client):
    body = client.post("/api/benchmark", json={"experiment": 4}).get_json()
    assert "experiment_4" in body
    assert body["experiment_4"]["rows"]


def test_benchmark_rejects_an_out_of_range_experiment(client):
    assert client.post("/api/benchmark", json={"experiment": 99}).status_code == 400


def test_index_page_loads_the_dashboard(client):
    html = client.get("/").get_data(as_text=True)
    assert "Drone Delivery Route Optimization" in html
    for asset in ("map.js", "controls.js", "results.js", "charts.js",
                  "animation.js", "app.js", "style.css"):
        assert asset in html
