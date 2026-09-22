"""Delivery priority queue (TDD section 9.1, FR-6).

Ordering is by the tuple (priority class, submission sequence), so urgent work
overtakes routine work while equal-priority requests keep strict arrival order.
Because the sequence number is unique the ordering is total, which is what makes
the resulting schedule reproducible (design principle P5).
"""

from __future__ import annotations

from typing import Iterable, Iterator

from ..domain.models import DeliveryRequest, Priority
from ..structures.min_heap import MinHeap


class DeliveryQueue:
    """Min-heap of pending deliveries. Push and pop are O(log n) (FR-6.4)."""

    __slots__ = ("_heap",)

    def __init__(self, requests: Iterable[DeliveryRequest] = ()) -> None:
        self._heap = MinHeap.heapify(
            ((self.key(r), r) for r in requests)
        )

    @staticmethod
    def key(request: DeliveryRequest) -> tuple[int, int]:
        """Equation (9). Priority first, then arrival order within a class."""
        return (int(request.priority), request.sequence)

    def push(self, request: DeliveryRequest) -> None:
        self._heap.push(self.key(request), request)

    def pop(self) -> DeliveryRequest:
        return self._heap.pop_min()[1]

    def peek(self) -> DeliveryRequest:
        return self._heap.peek()[1]

    def drain(self) -> Iterator[DeliveryRequest]:
        """Yield every request in dispatch order, emptying the queue."""
        while self._heap:
            yield self.pop()

    def order(self) -> list[DeliveryRequest]:
        """Dispatch order without consuming the queue (used by the UI, FR-6.5)."""
        snapshot = list(self._heap._data)
        out = [payload for _key, _seq, payload in snapshot]
        out.sort(key=self.key)
        return out

    def __len__(self) -> int:
        return len(self._heap)

    def __bool__(self) -> bool:
        return bool(self._heap)


def age_request(request: DeliveryRequest) -> DeliveryRequest:
    """Promote a request one priority class (optional aging policy, FR-6.6).

    Disabled by default, so the baseline behaviour is a pure priority queue.
    """
    if request.priority is Priority.NORMAL:
        request.priority = Priority.HIGH
    elif request.priority is Priority.HIGH:
        request.priority = Priority.URGENT
    return request
