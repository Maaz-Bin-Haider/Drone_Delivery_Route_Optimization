"""Choosing how many drones to launch (FR-7.9 – FR-7.12, TDD §9.5)."""

import pytest

from ddros.cost.cost_model import SHORTEST_DISTANCE, CostModel
from ddros.domain.models import DeliveryRequest, Drone, NodeType, Priority
from ddros.scheduling.assignment import assign_fleet
from ddros.scheduling.fleet_sizing import launch_order, size_fleet
from ddros.simulation.cache import RouteTable


@pytest.fixture(scope="module")
def table(city):
    return RouteTable(city, CostModel(city, SHORTEST_DISTANCE))


@pytest.fixture(scope="module")
def customers(city):
    return [n.id for n in city.nodes.values() if n.type is NodeType.CUSTOMER]


def batch(customers, count):
    return [DeliveryRequest(f"P{i}", customers[i % len(customers)], Priority.NORMAL, i)
            for i in range(count)]


def test_best_charged_aircraft_launch_first(scenario):
    order = launch_order(scenario.drones)
    charges = [d.battery_pct for d in order]
    assert charges == sorted(charges, reverse=True)


def test_launch_order_is_total_so_the_choice_is_reproducible():
    tied = [Drone("D3", "DEP", 80.0), Drone("D1", "DEP", 80.0), Drone("D2", "DEP", 80.0)]
    assert [d.id for d in launch_order(tied)] == ["D1", "D2", "D3"]


def test_a_single_order_launches_a_single_drone(table, scenario, customers):
    assert size_fleet(table, scenario.drones, batch(customers, 1)).chosen == 1


def test_small_batches_do_not_launch_the_whole_roster(table, scenario, customers):
    """The point of FR-7.9: aircraft that would sit idle stay on the ground."""
    chosen = size_fleet(table, scenario.drones, batch(customers, 3)).chosen
    assert chosen < len(scenario.drones)


def test_fleet_size_grows_with_the_batch(table, scenario, customers):
    small = size_fleet(table, scenario.drones, batch(customers, 2)).chosen
    large = size_fleet(table, scenario.drones, batch(customers, 18)).chosen
    assert small < large


def test_an_empty_batch_launches_nothing(table, scenario):
    sizing = size_fleet(table, scenario.drones, [])
    assert sizing.chosen == 0
    assert len(sizing.reserve) == len(scenario.drones)
    assert sizing.plan.assignments == []


def test_service_level_is_filtered_before_speed(table, scenario, customers):
    """A fleet that abandons half the batch posts a short makespan.

    Selecting on speed alone would pick exactly that plan, so this guards the
    filter in TDD §9.5 line 5 rather than the arithmetic around it.
    """
    # Consolidation is disabled here on purpose: it lets small fleets finish
    # batches they otherwise could not, which removes the very condition this
    # test needs. The guard still has to hold when it is off.
    orders = batch(customers, 18)
    sizing = size_fleet(table, scenario.drones, orders, consolidate="off")
    fewest = min(o.unserviceable for o in sizing.options)
    complete = [o for o in sizing.options if o.unserviceable == fewest]
    incomplete = [o for o in sizing.options if o.unserviceable > fewest]

    assert incomplete, "expected some fleet size to be too small for this batch"
    # The trap: an abandoning plan outruns a plan that actually finishes the
    # batch. A speed-first selector would take it.
    assert any(bad.makespan_min < good.makespan_min
               for bad in incomplete for good in complete), (
        "the guard is only meaningful if an incomplete plan outruns a complete one")

    chosen = next(o for o in sizing.options if o.drones == sizing.chosen)
    assert chosen.unserviceable == fewest, "selected a plan that abandons orders"
    assert "undelivered" in sizing.reason


def test_a_forced_size_is_honoured(table, scenario, customers):
    sizing = size_fleet(table, scenario.drones, batch(customers, 10), fixed=3)
    assert sizing.chosen == 3
    assert "fixed" in sizing.reason
    # `chosen` records the decision; `launched` records what flew, and the
    # improvement pass may have consolidated a tour away entirely.
    assert sizing.launched <= sizing.chosen


def test_a_forced_size_is_clamped_to_the_roster(table, scenario, customers):
    sizing = size_fleet(table, scenario.drones, batch(customers, 5), fixed=99)
    assert sizing.chosen == len(scenario.drones)


def test_zero_tolerance_minimises_makespan_outright(table, scenario, customers):
    orders = batch(customers, 10)
    strict = size_fleet(table, scenario.drones, orders, tolerance=0.0)
    chosen = next(o for o in strict.options if o.drones == strict.chosen)
    viable = [o for o in strict.options if o.unserviceable == chosen.unserviceable]
    assert chosen.makespan_min == pytest.approx(min(o.makespan_min for o in viable))


def test_a_generous_tolerance_launches_fewer_aircraft(table, scenario, customers):
    orders = batch(customers, 10)
    strict = size_fleet(table, scenario.drones, orders, tolerance=0.0).chosen
    relaxed = size_fleet(table, scenario.drones, orders, tolerance=0.9).chosen
    assert relaxed <= strict


def test_dispatched_and_reserve_partition_the_roster(table, scenario, customers):
    sizing = size_fleet(table, scenario.drones, batch(customers, 6))
    assert len(sizing.dispatched) == sizing.launched
    assert set(sizing.dispatched) | set(sizing.reserve) == {d.id for d in scenario.drones}
    assert not set(sizing.dispatched) & set(sizing.reserve)


def test_a_drone_left_empty_by_the_improvement_pass_is_not_reported_as_flying(
        table, scenario, customers):
    """The sizer decides before the improvement pass consolidates tours.

    If a selected drone ends up carrying nothing, reporting it as launched
    overstates the fleet -- and the interface would show a drone with 0 pkg
    sitting in the key.
    """
    for count in (10, 18, 25):
        sizing = size_fleet(table, scenario.drones, batch(customers, count))
        flying = {d.id for d in sizing.plan.drones if d.assigned}
        assert set(sizing.dispatched) == flying
        assert not (set(sizing.reserve) & flying)


def test_the_returned_plan_matches_the_chosen_size(table, scenario, customers):
    orders = batch(customers, 8)
    sizing = size_fleet(table, scenario.drones, orders)
    direct = assign_fleet(table, launch_order(scenario.drones)[:sizing.chosen], orders)
    assert sizing.plan.makespan_min == pytest.approx(direct.makespan_min)


def test_the_decision_explains_itself(table, scenario, customers):
    """FR-7.12: no unexplained decisions."""
    sizing = size_fleet(table, scenario.drones, batch(customers, 10))
    assert sizing.reason
    assert len(sizing.options) == len(scenario.drones)
    assert all(o.drones and o.makespan_min >= 0 for o in sizing.options)


def test_sizing_is_deterministic(table, scenario, customers):
    orders = batch(customers, 12)
    a = size_fleet(table, scenario.drones, orders)
    b = size_fleet(table, scenario.drones, orders)
    assert (a.chosen, a.reason, a.dispatched) == (b.chosen, b.reason, b.dispatched)
