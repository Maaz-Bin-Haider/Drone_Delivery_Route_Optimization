"""The dependency rule of TDD section 3.2, enforced rather than trusted.

The algorithm tier must stay free of web-framework imports so it remains
testable and benchmarkable without starting a server (NFR-7).
"""

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent / "ddros"
ALGORITHM_TIER = ("algorithms", "graph", "cost", "scheduling", "structures",
                  "environment", "domain", "analysis", "simulation")
FORBIDDEN = {"flask", "werkzeug", "jinja2"}


def sources():
    for package in ALGORITHM_TIER:
        yield from (ROOT / package).rglob("*.py")


@pytest.mark.parametrize("path", list(sources()), ids=lambda p: p.name)
def test_module_does_not_import_the_web_layer(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [a.name.split(".")[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [(node.module or "").split(".")[0]]
        else:
            continue
        for name in names:
            assert name not in FORBIDDEN, f"{path.name} imports '{name}'"
            assert "web" not in name, f"{path.name} imports the web tier"


def test_algorithm_tier_avoids_heavy_numeric_dependencies():
    """Every operation should be explicit Python, not hidden in a numeric library."""
    for path in sources():
        text = path.read_text(encoding="utf-8")
        assert "import numpy" not in text, f"{path.name} imports numpy"


def test_shortest_path_is_not_delegated_to_a_library():
    """Constraint C-1: the algorithms are the substance of the work."""
    for path in sources():
        text = path.read_text(encoding="utf-8")
        assert "networkx" not in text, f"{path.name} references networkx"
        assert "scipy" not in text, f"{path.name} references scipy"
        if path.name != "min_heap.py" and "test" not in path.name:
            assert "import heapq" not in text, f"{path.name} uses the stdlib heap"
