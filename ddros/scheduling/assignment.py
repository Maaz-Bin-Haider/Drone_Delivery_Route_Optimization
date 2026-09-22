"""Greedy multi-drone assignment (TDD section 9.3, FR-7).

The objective is minimum makespan -- the completion time of the *last* delivery
-- because that is what the project brief means by completing the batch faster.
Each delivery, taken in priority order, goes to the eligible drone that can
finish it soonest.

This is list scheduling. Graham (1969) proves a (2 - 1/m) bound for that family
on identical parallel machines, but the bound does not transfer here: a
delivery's duration depends on where the drone currently is, and battery state
makes the eligible set change over time. The strategy is therefore a
well-motivated heuristic with a known analogue, not an algorithm carrying a
proven ratio -- and Experiment 5 measures its actual quality against brute force.

Complexity: O(P * D + P log P) once the route table is in hand.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from ..algorithms.constrained import is_feasible
from ..constants import RESERVE_PCT, SERVICE_TIME_MIN
from ..domain.models import (Assignment, DeliveryRequest, DeliveryStatus, Drone,
                             Unserviceable)
from ..simulation.cache import RouteTable, combined_route
from .priority_queue import DeliveryQueue

_EPS = 1e-9


@dataclass
class FleetPlan:
    assignments: list[Assignment]
    unserviceable: list[Unserviceable]
    drones: list[Drone]

    @property
    def makespan_min(self) -> float:
        return max((d.ready_at_min for d in self.drones), default=0.0)


def assign_fleet(table: RouteTable, drones: list[Drone],
                 deliveries: list[DeliveryRequest],
                 reserve_pct: float = RESERVE_PCT,
                 service_min: float = SERVICE_TIME_MIN,
                 blocked_nodes: dict[str, str] | None = None) -> FleetPlan:
    fleet = [d.copy() for d in drones]
    fleet.sort(key=lambda d: d.id)
    # Copy the requests too: planning records status on them, and a caller's
    # batch must survive a planning cycle unchanged so repeated plans from the
    # same scenario stay independent.
    queue = DeliveryQueue(replace(p) for p in deliveries)
    assignments: list[Assignment] = []
    unserviceable: list[Unserviceable] = []
    blocked = blocked_nodes or {}

    for request in queue.drain():
        zone = blocked.get(request.destination)
        if zone is not None:                                    # FR-8.3
            request.status = DeliveryStatus.UNSERVICEABLE
            unserviceable.append(Unserviceable(
                request.id, request.destination, request.priority,
                f"Destination {request.destination} lies inside active no-fly zone '{zone}'",
            ))
            continue

        best: Drone | None = None
        best_finish = float("inf")
        best_route = None
        best_reroute = None
        considered: list[tuple[str, float]] = []
        rejections: list[str] = []

        for drone in fleet:
            direct = table.route(drone.current_node, request.destination)
            if direct is None:
                rejections.append(f"{drone.id}: no route from {drone.current_node}")
                continue

            reroute = None
            if is_feasible(direct, drone.battery_pct, reserve_pct):
                route, travel = direct, direct.time_min
            else:
                reroute = table.charging_reroute(
                    drone.current_node, request.destination,
                    drone.battery_pct, reserve_pct)
                if reroute is None:
                    rejections.append(
                        f"{drone.id}: needs {direct.energy_pct:.1f}% but has "
                        f"{drone.battery_pct - reserve_pct:.1f}% usable, and no "
                        f"charging station makes the trip feasible")
                    continue
                route, travel = combined_route(reroute), reroute.total_time_min

            finish = drone.ready_at_min + travel + service_min
            considered.append((drone.id, finish))
            if finish < best_finish - _EPS or (
                    abs(finish - best_finish) <= _EPS and best is not None
                    and drone.id < best.id):
                best, best_finish = drone, finish
                best_route, best_reroute = route, reroute

        if best is None or best_route is None:
            request.status = DeliveryStatus.UNSERVICEABLE
            reason = "; ".join(rejections) or "no drone available"
            unserviceable.append(Unserviceable(
                request.id, request.destination, request.priority, reason))
            continue

        battery_before = best.battery_pct
        if best_reroute is not None:
            battery_after = 100.0 - best_reroute.leg_two.energy_pct
        else:
            battery_after = best.battery_pct - best_route.energy_pct

        depart = best.ready_at_min
        best.ready_at_min = best_finish                          # FR-7.5
        best.current_node = request.destination
        best.battery_pct = battery_after
        best.assigned.append(request.id)
        best.distance_flown_km += best_route.distance_km
        best.energy_used_pct += best_route.energy_pct
        request.status = DeliveryStatus.ASSIGNED

        assignments.append(Assignment(
            delivery_id=request.id,
            drone_id=best.id,
            destination=request.destination,
            priority=request.priority,
            route=best_route,
            reroute=best_reroute,
            depart_min=depart,
            arrive_min=best_finish - service_min,
            battery_before_pct=battery_before,
            battery_after_pct=battery_after,
            reason=_explain(best.id, best_finish, considered, best_reroute),
        ))

    return FleetPlan(assignments, unserviceable, fleet)


def _explain(chosen: str, finish: float, considered: list[tuple[str, float]],
             reroute) -> str:
    """Human-readable justification for the choice (FR-10.7)."""
    others = [f"{d} at {f:.1f} min" for d, f in considered if d != chosen]
    base = f"{chosen} completes at {finish:.1f} min"
    if others:
        base += " vs " + ", ".join(others)
    else:
        base += " (only eligible drone)"
    if reroute is not None:
        base += (f"; routed via charging station {reroute.station_id}, "
                 f"arriving on {reroute.arrival_battery_pct:.1f}% and "
                 f"recharging for {reroute.recharge_min:.1f} min")
    return base
