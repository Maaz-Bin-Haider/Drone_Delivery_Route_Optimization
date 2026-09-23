"""Binary min-heap, implemented from first principles.

Constraint C-1 of the SRS requires the priority queue to be written for this
project rather than imported, since it is one of the data structures the work
is about. `heapq` appears only in the test suite, as a cross-checking oracle.

Complexity: push O(log n), pop_min O(log n), peek O(1), heapify O(n).
"""

from __future__ import annotations

from typing import Any, Iterable

from ..analysis.instrumentation import Counters


class MinHeap:
    """Min-heap over (key, sequence, payload) triples.

    `sequence` is a strictly increasing insertion counter. It makes the ordering
    total, so equal keys break ties deterministically by insertion order and the
    whole system's output is reproducible (design principle P5).
    """

    __slots__ = ("_data", "_seq", "_counters")

    def __init__(self, counters: Counters | None = None) -> None:
        self._data: list[tuple[Any, int, Any]] = []
        self._seq = 0
        self._counters = counters

    # -- internal -----------------------------------------------------------

    def _tick(self) -> None:
        if self._counters is not None:
            self._counters.heap_operations += 1

    def _sift_up(self, i: int) -> None:
        data = self._data
        item = data[i]
        while i > 0:
            parent = (i - 1) >> 1
            if item < data[parent]:
                data[i] = data[parent]
                i = parent
            else:
                break
        data[i] = item

    def _sift_down(self, i: int) -> None:
        data = self._data
        n = len(data)
        item = data[i]
        while True:
            left = 2 * i + 1
            if left >= n:
                break
            right = left + 1
            child = left
            if right < n and data[right] < data[left]:
                child = right
            if data[child] < item:
                data[i] = data[child]
                i = child
            else:
                break
        data[i] = item

    # -- public API ---------------------------------------------------------

    def push(self, key: Any, payload: Any) -> None:
        self._tick()
        self._data.append((key, self._seq, payload))
        self._seq += 1
        self._sift_up(len(self._data) - 1)

    def pop_min(self) -> tuple[Any, Any]:
        if not self._data:
            raise IndexError("pop_min from an empty heap")
        self._tick()
        top = self._data[0]
        last = self._data.pop()
        if self._data:
            self._data[0] = last
            self._sift_down(0)
        return top[0], top[2]

    def peek(self) -> tuple[Any, Any]:
        if not self._data:
            raise IndexError("peek at an empty heap")
        top = self._data[0]
        return top[0], top[2]

    @classmethod
    def heapify(cls, items: Iterable[tuple[Any, Any]],
                counters: Counters | None = None) -> "MinHeap":
        """Build in O(n) by Floyd's method, rather than O(n log n) pushes."""
        heap = cls(counters)
        for key, payload in items:
            heap._data.append((key, heap._seq, payload))
            heap._seq += 1
        for i in range(len(heap._data) // 2 - 1, -1, -1):
            heap._sift_down(i)
        return heap

    def __len__(self) -> int:
        return len(self._data)

    def __bool__(self) -> bool:
        return bool(self._data)
