"""The hand-written heap must behave exactly like a reference implementation."""

import heapq
import random

from ddros.structures.min_heap import MinHeap


def test_matches_heapq_under_random_operations():
    """Cross-check against the standard library, used here only as an oracle."""
    random.seed(20260922)
    heap, reference = MinHeap(), []
    for _ in range(10_000):
        if reference and random.random() < 0.4:
            assert heap.pop_min()[0] == heapq.heappop(reference)
        else:
            key = random.randint(-1000, 1000)
            heap.push(key, f"item{key}")
            heapq.heappush(reference, key)
    while reference:
        assert heap.pop_min()[0] == heapq.heappop(reference)
    assert len(heap) == 0


def test_heapify_builds_valid_heap():
    items = [(random.randint(0, 500), i) for i in range(500)]
    heap = MinHeap.heapify(items)
    out = [heap.pop_min()[0] for _ in range(len(items))]
    assert out == sorted(k for k, _ in items)


def test_equal_keys_break_ties_by_insertion_order():
    """Determinism (principle P5) depends on this."""
    heap = MinHeap()
    for name in ("first", "second", "third"):
        heap.push(7, name)
    assert [heap.pop_min()[1] for _ in range(3)] == ["first", "second", "third"]


def test_tuple_keys_compare_lexicographically():
    heap = MinHeap()
    for key in [(2, 0), (1, 5), (1, 2), (0, 9)]:
        heap.push(key, key)
    assert [heap.pop_min()[0] for _ in range(4)] == [(0, 9), (1, 2), (1, 5), (2, 0)]
