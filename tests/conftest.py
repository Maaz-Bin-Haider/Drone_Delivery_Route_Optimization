"""Shared fixtures."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ddros.domain.models import Node, NodeType          # noqa: E402
from ddros.domain.loader import load_scenario           # noqa: E402
from ddros.graph.graph import Graph                     # noqa: E402


@pytest.fixture(scope="session")
def scenario():
    return load_scenario(ROOT / "data")


@pytest.fixture(scope="session")
def city(scenario):
    return scenario.graph


@pytest.fixture(scope="session")
def brief_graph():
    """The worked example from page 8 of the project brief.

    Coordinates are tightly clustered so the straight-line heuristic stays well
    below the stated edge distances, keeping it admissible against weights that
    were authored rather than derived.
    """
    coords = {"W": (0, 0), "A": (0.001, 0.002), "B": (0.003, 0),
              "C": (0, -0.002), "D": (0.002, -0.004), "E": (0.004, 0)}
    types = {"W": NodeType.WAREHOUSE, "E": NodeType.CUSTOMER,
             "C": NodeType.CHARGING_STATION}
    nodes = [Node(k, k, la, lo, types.get(k, NodeType.WAYPOINT))
             for k, (la, lo) in coords.items()]
    edges = [{"u": "W", "v": "A", "distance_km": 4}, {"u": "W", "v": "B", "distance_km": 3},
             {"u": "W", "v": "C", "distance_km": 2}, {"u": "A", "v": "B", "distance_km": 2},
             {"u": "A", "v": "E", "distance_km": 4}, {"u": "B", "v": "E", "distance_km": 1},
             {"u": "C", "v": "D", "distance_km": 5}, {"u": "D", "v": "E", "distance_km": 2}]
    return Graph.build(nodes, edges)
