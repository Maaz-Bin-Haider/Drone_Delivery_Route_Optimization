"""Search instrumentation (TDD section 7.4).

The counters live inside the search routines themselves, so the numbers the
benchmark harness reports and the numbers a normal run reports are produced by
identical code paths.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Counters:
    """Mutable tally, threaded through a single search."""

    nodes_expanded: int = 0
    heap_operations: int = 0

    def snapshot(self, runtime_ms: float) -> "SearchStats":
        return SearchStats(self.nodes_expanded, self.heap_operations, runtime_ms)


@dataclass(frozen=True)
class SearchStats:
    """Immutable record of one completed search."""

    nodes_expanded: int
    heap_operations: int
    runtime_ms: float

    def as_dict(self) -> dict:
        return {
            "nodes_expanded": self.nodes_expanded,
            "heap_operations": self.heap_operations,
            "runtime_ms": round(self.runtime_ms, 4),
        }
