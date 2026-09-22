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

**En-route consolidation.** The loop above plans each delivery as a point-to-point
trip from the drone's current position, which throws away a useful fact: the
chosen path frequently crosses *other* pending destinations. Left alone, the
system sends a second drone to a location the first one flew straight over. So
after a route is chosen, any pending delivery sitting on an intermediate node of
that path is handed to the same drone, costing one service stop instead of an
entire separate flight.

Whether a *less* urgent parcel may ride along is a policy, not a fact, because
each extra stop delays everything behind it by the service time:

  ``off``     no consolidation; every delivery is its own flight.
  ``safe``    (default) a parcel rides along only if it is at least as urgent as
              the delivery whose route it is on. Priority order is never
              inverted, at the cost of leaving some fly-overs unserved.
  ``always``  any parcel on the path is taken. Measured over the three
              demonstration plans this flies about 3% less distance, but pushes
              the last urgent arrival out by one service stop, and because the
              greedy is not monotone it is not uniformly better -- one scenario
              got worse. Offered, measured, and not made the default.

Complexity: O(P * D + P log P) once the route table is in hand; consolidation
adds O(L) per assignment for a path of L nodes, so the bound is unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from ..algorithms.constrained import is_feasible
from ..constants import RESERVE_PCT, SERVICE_TIME_MIN
from ..domain.models import (Assignment, DeliveryRequest, DeliveryStatus, Drone,
                             Route, Unserviceable)
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
                 blocked_nodes: dict[str, str] | None = None,
                 consolidate: str = "safe") -> FleetPlan:
    fleet = [d.copy() for d in drones]
    fleet.sort(key=lambda d: d.id)
    # Copy the requests too: planning records status on them, and a caller's
    # batch must survive a planning cycle unchanged so repeated plans from the
    # same scenario stay independent.
    batch = [replace(p) for p in deliveries]
    queue = DeliveryQueue(batch)
    assignments: list[Assignment] = []
    unserviceable: list[Unserviceable] = []
    blocked = blocked_nodes or {}

    # Pending deliveries indexed by destination, so a chosen route can be
    # checked against them in one pass over its path. `resolved` holds every
    # request that is no longer pending by ANY route -- assigned normally,
    # dropped en route, or found unserviceable. Tracking only the en-route
    # drops would let a delivery already flown be picked up a second time.
    waiting: dict[str, list[DeliveryRequest]] = {}
    for p in batch:
        waiting.setdefault(p.destination, []).append(p)
    resolved: set[str] = set()

    for request in queue.drain():
        if request.id in resolved:
            continue                      # already dropped en route
        zone = blocked.get(request.destination)
        if zone is not None:                                    # FR-8.3
            resolved.add(request.id)
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
            resolved.add(request.id)
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
        # Pick up anything sitting on the path before committing the clock.
        resolved.add(request.id)
        riders = _enroute_riders(table, best_route, request, waiting, resolved,
                                 service_min, consolidate)
        best_finish += len(riders) * service_min        # each stop delays the rest
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
            reason=_explain(best.id, best_finish, considered, best_reroute,
                            len(riders)),
        ))

        for rider, arrive_at, prefix in riders:
            rider.status = DeliveryStatus.ASSIGNED
            best.assigned.append(rider.id)
            assignments.append(Assignment(
                delivery_id=rider.id,
                drone_id=best.id,
                destination=rider.destination,
                priority=rider.priority,
                route=prefix,
                reroute=None,
                depart_min=depart,
                arrive_min=arrive_at,
                battery_before_pct=battery_before,
                battery_after_pct=battery_after,
                reason=(f"dropped en route: {best.id} was already flying "
                        f"{' > '.join(best_route.path)} to deliver {request.id}, "
                        f"and {rider.destination} lies on that path. One service "
                        f"stop instead of a separate flight"),
                enroute=True,
            ))

    # Left in dispatch order deliberately: that is the order the priority queue
    # produced, and each en-route drop already follows the flight it rode on.
    # Sorting by arrival would read chronologically but hide the prioritisation.
    return FleetPlan(assignments, unserviceable, fleet)


def _enroute_riders(table: RouteTable, route, primary: DeliveryRequest,
                    waiting: dict[str, list[DeliveryRequest]], resolved: set[str],
                    service_min: float, policy: str = "safe"):
    """Pending deliveries sitting on the intermediate nodes of `route`.

    Returns (request, arrival time, route prefix) for each. Under the ``safe``
    policy a parcel less urgent than `primary` is left behind rather than
    delaying it; under ``always`` it is taken.

    Energy is unchanged: the drone flies the same path either way, and hovering
    to release a parcel is not modelled (assumption A-3). The only cost is time.
    """
    riders = []
    if policy == "off":
        return riders
    path = route.path
    if len(path) < 3:
        return riders

    times = table.leg_times(path)
    for index in range(1, len(path) - 1):
        node = path[index]
        for candidate in waiting.get(node, []):
            if candidate.id in resolved:
                continue                  # already flown, or being flown now
            if policy == "safe" and candidate.priority > primary.priority:
                continue                  # would delay more urgent work
            prefix = Route(
                path=path[:index + 1],
                distance_km=_prefix_distance(table, path, index),
                energy_pct=0.0,           # shares the primary flight; no extra energy
                time_min=times[index],
                cost=0.0,
                algorithm=route.algorithm,
                stats=route.stats,
            )
            arrive = times[index] + len(riders) * service_min + service_min
            riders.append((candidate, arrive, prefix))
            resolved.add(candidate.id)
            break                         # one parcel per stop
    return riders


def _prefix_distance(table: RouteTable, path, index: int) -> float:
    total = 0.0
    for u, v in zip(path[:index + 1], path[1:index + 1]):
        edge = next((e for e in table.graph.adj[u] if e.v == v), None)
        if edge is not None:
            total += edge.distance_km
    return total


def _explain(chosen: str, finish: float, considered: list[tuple[str, float]],
             reroute, riders: int = 0) -> str:
    """Human-readable justification for the choice (FR-10.7)."""
    others = [f"{d} at {f:.1f} min" for d, f in considered if d != chosen]
    base = f"{chosen} completes at {finish:.1f} min"
    if riders:
        base += (f" (including {riders} parcel{'s' if riders > 1 else ''} "
                 f"dropped en route)")
    if others:
        base += " vs " + ", ".join(others)
    else:
        base += " (only eligible drone)"
    if reroute is not None:
        base += (f"; routed via charging station {reroute.station_id}, "
                 f"arriving on {reroute.arrival_battery_pct:.1f}% and "
                 f"recharging for {reroute.recharge_min:.1f} min")
    return base
