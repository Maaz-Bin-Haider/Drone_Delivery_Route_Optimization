"""Dispatch ordering (FR-6)."""

from ddros.domain.models import DeliveryRequest, Priority
from ddros.scheduling.priority_queue import DeliveryQueue


def make(idx, priority):
    return DeliveryRequest(f"PKG-{idx:03d}", "L01", priority, idx)


def test_urgent_precedes_high_precedes_normal():
    q = DeliveryQueue([make(0, Priority.NORMAL), make(1, Priority.URGENT),
                       make(2, Priority.HIGH)])
    assert [r.priority for r in q.drain()] == [
        Priority.URGENT, Priority.HIGH, Priority.NORMAL]


def test_equal_priority_keeps_arrival_order():
    q = DeliveryQueue([make(i, Priority.NORMAL) for i in range(6)])
    assert [r.id for r in q.drain()] == [f"PKG-{i:03d}" for i in range(6)]


def test_order_preview_does_not_consume_the_queue():
    """FR-6.5: the interface shows the dispatch order without destroying it."""
    q = DeliveryQueue([make(i, Priority.HIGH) for i in range(4)])
    preview = [r.id for r in q.order()]
    assert len(q) == 4
    assert preview == [r.id for r in q.drain()]


def test_real_batch_dispatches_urgent_first(scenario):
    order = [r.id for r in DeliveryQueue(scenario.deliveries).drain()]
    assert order[:2] == ["PKG-002", "PKG-007"]          # the two URGENT packages
