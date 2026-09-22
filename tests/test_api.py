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
    assert len(body["nodes"]) >= 30                       # FR-1.8
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
                         json={"active_no_fly_zones": ["nfz_aerodrome"]}).get_json()
    assert masked["totals"]["unserviceable"] > free["totals"]["unserviceable"]
    assert any("no-fly zone" in u["reason"] for u in masked["unserviceable"])


# -- error handling (TDD 12.2) --------------------------------------------

def test_zero_weights_are_rejected_with_the_field_named(client):
    response = client.post("/api/plan",
                           json={"weights": {"alpha": 0, "beta": 0, "gamma": 0}})
    assert response.status_code == 400
    assert response.get_json()["field"] == "weights"


def test_unknown_node_is_rejected_with_the_field_named(client):
    response = client.post("/api/route", json={"source": "DEP", "target": "ghost"})
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
    body = client.post("/api/compare", json={"source": "DEP", "target": "L16"}).get_json()
    assert body["agree"] is True
    assert body["astar"]["stats"]["nodes_expanded"] <= body["dijkstra"]["stats"]["nodes_expanded"]


def test_alternatives_report_both_routes(client):
    body = client.post("/api/route/alternatives",
                       json={"source": "L06", "target": "L21",
                             "wind": {"speed_ms": 16, "bearing_deg": 225}}).get_json()
    assert body["min_distance"] and body["min_energy"]
    assert body["differ"] is True
    assert "less energy" in body["annotation"]


def test_single_route_honours_the_selected_algorithm(client):
    body = client.post("/api/route", json={"source": "DEP", "target": "L16",
                                           "algorithm": "dijkstra"}).get_json()
    assert body["algorithm"] == "dijkstra"


# -- deliveries ------------------------------------------------------------

def test_a_delivery_can_be_added(client):
    before = len(client.get("/api/scenario").get_json()["deliveries"])
    response = client.post("/api/deliveries",
                           json={"destination": "L06", "priority": "urgent"})
    assert response.status_code == 201
    assert response.get_json()["count"] == before + 1
    assert len(client.get("/api/scenario").get_json()["deliveries"]) == before + 1


def test_duplicate_delivery_id_is_rejected(client):
    client.post("/api/deliveries", json={"id": "PKG-X", "destination": "L06"})
    assert client.post("/api/deliveries",
                       json={"id": "PKG-X", "destination": "L06"}).status_code == 400


def test_unknown_priority_is_rejected(client):
    response = client.post("/api/deliveries",
                           json={"destination": "L06", "priority": "whenever"})
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


# -- demonstration plans and order management ------------------------------

def test_presets_are_listed_with_their_sizes(client):
    presets = client.get("/api/presets").get_json()["presets"]
    assert len(presets) >= 3
    for p in presets:
        assert p["id"] and p["name"] and p["summary"] and p["count"] > 0


def test_loading_a_preset_replaces_the_whole_batch(client):
    before = client.get("/api/scenario").get_json()["deliveries"]
    out = client.post("/api/presets/peak_load").get_json()
    assert out["count"] == 18
    assert out["count"] != len(before)
    assert [d["id"] for d in out["deliveries"]] == [f"PKG-{i:03d}" for i in range(1, 19)]


def test_a_loaded_preset_dispatches_in_its_own_order(client):
    """Sequence numbers must be rebuilt, not continued from the previous batch."""
    client.post("/api/presets/peak_load")
    client.post("/api/presets/morning_round")
    sequences = [d["sequence"] for d in client.get("/api/deliveries").get_json()["deliveries"]]
    assert sequences == list(range(len(sequences)))


def test_unknown_preset_is_rejected_with_a_clean_message(client):
    response = client.post("/api/presets/nonesuch")
    assert response.status_code == 400
    body = response.get_json()
    assert body["field"] == "preset_id"
    assert body["error"] == "unknown preset 'nonesuch'"


def test_an_order_can_be_removed(client):
    before = client.get("/api/deliveries").get_json()["deliveries"]
    target = before[2]["id"]
    out = client.delete(f"/api/deliveries/{target}").get_json()
    assert out["count"] == len(before) - 1
    assert target not in [d["id"] for d in out["deliveries"]]


def test_removing_an_unknown_order_is_rejected(client):
    assert client.delete("/api/deliveries/PKG-999").status_code == 400


def test_the_batch_can_be_cleared_and_rebuilt_by_hand(client):
    assert client.post("/api/deliveries/clear").get_json()["count"] == 0
    plan = client.post("/api/plan", json={}).get_json()
    assert plan["totals"]["delivered"] == 0          # an empty batch is valid

    client.post("/api/deliveries", json={"destination": "L16", "priority": "URGENT"})
    client.post("/api/deliveries", json={"destination": "L01", "priority": "NORMAL"})
    plan = client.post("/api/plan", json={}).get_json()
    assert plan["totals"]["delivered"] == 2
    assert plan["assignments"][0]["priority"] == "URGENT"


def test_ids_do_not_collide_after_a_removal(client):
    """Sequential numbering must skip ids already in use."""
    client.post("/api/deliveries/clear")
    for dest in ("L01", "L02", "L03"):
        client.post("/api/deliveries", json={"destination": dest})
    client.delete("/api/deliveries/PKG-002")
    out = client.post("/api/deliveries", json={"destination": "L04"}).get_json()
    ids = [d["id"] for d in out["deliveries"]]
    assert len(ids) == len(set(ids)), f"duplicate id generated: {ids}"


def test_scenario_carries_the_fictional_backdrop(client):
    body = client.get("/api/scenario").get_json()
    assert body["name"] == "Kestrel Bay"
    backdrop = body["backdrop"]
    assert backdrop["water"] and backdrop["parks"] and backdrop["districts"]
    assert backdrop["river"]["points"]


def test_every_location_names_its_district(client):
    nodes = client.get("/api/scenario").get_json()["nodes"]
    assert all(n["district"] for n in nodes)


def test_the_fleet_has_five_drones(client):
    assert len(client.get("/api/scenario").get_json()["drones"]) == 5
