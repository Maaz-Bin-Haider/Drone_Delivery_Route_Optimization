"""Scenario validation must name what is wrong (FR-1.7)."""

import pytest

from ddros.domain.models import DeliveryRequest, Drone, Node, NodeType, Priority
from ddros.graph.graph import Graph
from ddros.graph.validation import (MapValidationError, ScenarioValidationError,
                                    validate_graph, validate_scenario)


def nodes(*specs):
    return [Node(i, i, la, lo, t) for i, la, lo, t in specs]


def test_disconnected_map_is_rejected():
    g = Graph.build(
        nodes(("W", 0, 0, NodeType.WAREHOUSE), ("S", 0.01, 0, NodeType.CHARGING_STATION),
              ("X", 5, 5, NodeType.CUSTOMER)),
        [{"u": "W", "v": "S"}])
    with pytest.raises(MapValidationError, match="not connected"):
        validate_graph(g)


def test_missing_warehouse_is_rejected():
    g = Graph.build(
        nodes(("A", 0, 0, NodeType.CUSTOMER), ("S", 0.01, 0, NodeType.CHARGING_STATION)),
        [{"u": "A", "v": "S"}])
    with pytest.raises(MapValidationError, match="exactly one warehouse"):
        validate_graph(g)


def test_missing_charging_station_is_rejected():
    g = Graph.build(
        nodes(("W", 0, 0, NodeType.WAREHOUSE), ("A", 0.01, 0, NodeType.CUSTOMER)),
        [{"u": "W", "v": "A"}])
    with pytest.raises(MapValidationError, match="charging station"):
        validate_graph(g)


def test_edge_to_an_unknown_node_is_rejected():
    with pytest.raises(KeyError, match="unknown node"):
        Graph.build(nodes(("W", 0, 0, NodeType.WAREHOUSE)), [{"u": "W", "v": "ghost"}])


def test_delivery_to_an_unknown_destination_is_named(city, scenario):
    bad = [DeliveryRequest("PKG-X", "nowhere", Priority.NORMAL, 0)]
    with pytest.raises(ScenarioValidationError, match="PKG-X"):
        validate_scenario(city, scenario.drones, bad)


def test_drone_with_impossible_battery_is_named(city, scenario):
    with pytest.raises(ScenarioValidationError, match="D9"):
        validate_scenario(city, [Drone("D9", "W", 150.0)], list(scenario.deliveries))


def test_the_shipped_map_is_valid(city, scenario):
    validate_graph(city)
    validate_scenario(city, scenario.drones, scenario.deliveries)
    assert len(city.nodes) >= 20                       # FR-1.8
    assert len(city.charging_stations) >= 2
