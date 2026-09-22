"""Assigning deliveries to drones (TDD section 9.3, FR-7).

A drone's plan is a **tour**: an ordered list of destinations it visits. Its
route is the concatenation of the shortest paths between consecutive stops, so
the cost of a tour falls out of the route table with no special cases. This
matters for one reason in particular: if a stop happens to lie on the path
between the two stops around it, inserting it costs only the service time,
because `d(a,b) + d(b,c) = d(a,c)` exactly when b is on the shortest path from
a to c. Delivering something "in passing" is therefore not a special mode -- it
is an ordinary tour stop that happens to be free.

Planning runs in three phases.

**Phase 1 -- greedy assignment.** Deliveries are taken in priority order and
appended to whichever drone can complete them soonest. This is list scheduling:
Graham (1969) proves a (2 - 1/m) bound for that family on identical parallel
machines, but the bound does not transfer here, because a delivery's duration
depends on where its drone currently is and battery state changes which drones
are eligible. It is a well-motivated heuristic, not one carrying a proven ratio,
and Experiment 5 measures its actual quality against brute force.

**Phase 2 -- improvement.** Greedy commits each delivery before it knows what
the rest of the schedule will look like, and never revisits the decision. The
visible consequence was a drone flying directly over a pending destination while
a second drone was dispatched to that same place. Phase 2 repairs this with
relocate moves: a delivery is moved to another drone's tour whenever doing so
reduces total flight distance. Where the destination already lies on that
drone's path the move is nearly free, which is exactly the fly-over case.

**Phase 3 -- simulation.** Each tour is flown: routes, arrival times, battery
state and any charging detours are computed from the finished tours, so there is
a single place where a plan turns into timings.

Complexity: phase 1 is O(P·D) lookups, phase 3 O(P), and phase 2 is O(K·P·D·S)
for K passes over tours of length S -- bounded by `MAX_IMPROVEMENT_PASSES`.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import math

from ..algorithms.constrained import is_feasible
from ..constants import RESERVE_PCT, SERVICE_TIME_MIN
from ..domain.models import (Assignment, ChargingReroute, DeliveryRequest,
                             DeliveryStatus, Drone, Priority, Route, Unserviceable)
from ..simulation.cache import RouteTable, combined_route
from .priority_queue import DeliveryQueue

_EPS = 1e-9

# Each pass applies one relocation, so the budget must scale with the batch: a
# fixed ceiling silently stops improving part-way through a large plan, leaving
# exactly the fly-overs the pass exists to remove.
MIN_IMPROVEMENT_MOVES = 60
MOVES_PER_DELIVERY = 4

# How much extra flying is worth a shorter finish. At zero the pass never
# rebalances a drone unless rebalancing also happens to be shorter; unbounded,
# it scatters the batch across the whole roster to shave seconds.
BALANCE_COST_TOLERANCE = 0.02

# Policies for phase 2. "safe" refuses any move that would put a less urgent
# parcel ahead of a more urgent one; "always" allows it; "off" skips phase 2.
POLICIES = ("off", "safe", "always")


@dataclass
class FleetPlan:
    assignments: list[Assignment]
    unserviceable: list[Unserviceable]
    drones: list[Drone]
    improvements: int = 0
    distance_saved_km: float = 0.0

    @property
    def makespan_min(self) -> float:
        return max((d.ready_at_min for d in self.drones), default=0.0)


@dataclass
class _Leg:
    """One hop of a tour, already costed."""

    request: DeliveryRequest
    route: Route
    reroute: ChargingReroute | None
    depart_min: float
    arrive_min: float
    battery_before: float
    battery_after: float
    free: bool           # the stop lay on the path between its neighbours


@dataclass
class _TourResult:
    legs: list[_Leg] = field(default_factory=list)
    finish_min: float = 0.0
    distance_km: float = 0.0
    cost: float = 0.0
    """Total composite cost: what the router actually minimises.

    The improvement pass scores moves on this rather than on raw distance.
    Under pure-distance weighting the two coincide, but once energy or time
    carries weight -- or a wind makes the same corridor cost different amounts
    in each direction -- optimising distance while routing by cost leaves the
    pass working against the objective the operator selected.
    """
    energy_pct: float = 0.0
    end_node: str = ""
    end_battery: float = 0.0
    feasible: bool = True
    blocker: str = ""


# --------------------------------------------------------------------------
# phase 3 -- flying a tour
# --------------------------------------------------------------------------

def simulate_tour(table: RouteTable, drone: Drone, stops: list[DeliveryRequest],
                  reserve_pct: float = RESERVE_PCT,
                  service_min: float = SERVICE_TIME_MIN) -> _TourResult:
    """Fly an ordered tour and cost it. Returns `feasible=False` if it cannot be flown."""
    result = _TourResult(end_node=drone.current_node,
                         end_battery=drone.battery_pct,
                         finish_min=drone.ready_at_min)
    position = drone.current_node
    clock = drone.ready_at_min
    battery = drone.battery_pct

    for index, request in enumerate(stops):
        direct = table.route(position, request.destination)
        if direct is None:
            result.feasible = False
            result.blocker = f"no route from {position} to {request.destination}"
            return result

        reroute = None
        if is_feasible(direct, battery, reserve_pct):
            route, travel = direct, direct.time_min
        else:
            reroute = table.charging_reroute(position, request.destination,
                                             battery, reserve_pct)
            if reroute is None:
                result.feasible = False
                result.blocker = (
                    f"needs {direct.energy_pct:.1f}% but has "
                    f"{battery - reserve_pct:.1f}% usable, and no charging station "
                    f"makes the trip feasible")
                return result
            route, travel = combined_route(reroute), reroute.total_time_min

        depart = clock
        arrive = clock + travel
        before = battery
        battery = (100.0 - reroute.leg_two.energy_pct if reroute
                   else battery - route.energy_pct)

        # A stop is "free" when it sits on the path its neighbours would have
        # flown anyway: the detour it causes is nil and only the service stop
        # is paid for.
        free = False
        if reroute is None and index + 1 < len(stops):
            onward = table.route(request.destination, stops[index + 1].destination)
            straight = table.route(position, stops[index + 1].destination)
            if onward and straight:
                detour = direct.distance_km + onward.distance_km - straight.distance_km
                free = detour < 1e-6

        result.legs.append(_Leg(request, route, reroute, depart, arrive,
                                before, battery, free))
        result.distance_km += route.distance_km
        result.cost += route.cost
        result.energy_pct += route.energy_pct
        clock = arrive + service_min
        position = request.destination

    result.finish_min = clock
    result.distance_km = round(result.distance_km, 9)
    result.end_node = position
    result.end_battery = battery
    return result


# --------------------------------------------------------------------------
# a second construction: the polar sweep
# --------------------------------------------------------------------------

def _bearing_from(graph, origin: str, node: str) -> float:
    """Compass bearing of `node` seen from `origin`, degrees."""
    a, b = graph.nodes[origin], graph.nodes[node]
    dy = b.lat - a.lat
    dx = (b.lon - a.lon) * math.cos(math.radians(a.lat))
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return 0.0
    return math.degrees(math.atan2(dx, dy)) % 360.0


def sweep_construction(table: RouteTable, fleet: list[Drone],
                       requests: list[DeliveryRequest],
                       reserve_pct: float, service_min: float
                       ) -> dict[str, list[DeliveryRequest]] | None:
    """Sort destinations by polar angle about the depot and cut into equal groups.

    The classical sweep heuristic for vehicle routing. It gives each drone a
    contiguous wedge of the map, which produces two properties the greedy of
    phase 1 has no reason to produce: the loads are balanced by construction,
    and each drone's stops lie in one direction rather than scattered across
    the city.

    Greedy scores candidates on completion time alone, so it will happily give
    one drone a delivery on the far side of the map because that drone happened
    to be free. Measured on the demonstration plans, sweeping first and then
    polishing is usually both shorter *and* tidier -- Peak Load at five drones
    goes from a 135-degree scatter to at most 59 degrees, and from 56.8 km to
    51.3. It is kept as a second starting point rather than a replacement,
    because on some batches greedy still wins.

    Returns None when no feasible assignment of the wedges can be found, in
    which case the caller simply keeps the greedy construction.
    """
    if not fleet or not requests:
        return None
    origin = fleet[0].current_node
    ordered = sorted(requests,
                     key=lambda r: (_bearing_from(table.graph, origin, r.destination),
                                    r.sequence))
    count = len(fleet)
    per = math.ceil(len(ordered) / count)
    groups = [ordered[i * per:(i + 1) * per] for i in range(count)]

    # Within a wedge, fly urgent work first. That keeps the tour safe by
    # construction, so the improvement pass has a legal starting point and can
    # relax the order later wherever a stop turns out to be free.
    groups = [sorted(g, key=lambda r: (r.priority, r.sequence)) for g in groups]

    # Repair: a wedge may be more than its drone can carry. Push the overflow
    # to the next drone round the sweep rather than abandoning the whole
    # construction, since the wedges are adjacent and the spill is local.
    for _ in range(len(ordered) + 1):
        trouble = None
        for index, (drone, group) in enumerate(zip(fleet, groups)):
            if group and not simulate_tour(table, drone, group,
                                           reserve_pct, service_min).feasible:
                trouble = index
                break
        if trouble is None:
            return {d.id: list(g) for d, g in zip(fleet, groups)}
        spill = groups[trouble].pop()
        target = (trouble + 1) % count
        if target == trouble:
            return None
        groups[target] = sorted(groups[target] + [spill],
                                key=lambda r: (r.priority, r.sequence))
    return None


# --------------------------------------------------------------------------
# phase 2 -- relocate moves
# --------------------------------------------------------------------------

def safe_ordering(result: _TourResult) -> bool:
    """Does this tour respect priority in the way that actually matters?

    The obvious reading -- "a tour must be sorted urgent-first" -- is too
    strict, and produces the behaviour it is meant to prevent. A drone carrying
    an urgent parcel to the far side of the map is forced to fly *past* a
    routine delivery standing directly on its path, deliver the urgent one, and
    come back. Nobody is better off: the urgent parcel arrives no sooner, and
    the fleet flies a pointless detour.

    What matters is that urgent work is never delayed by a **detour**. So a less
    urgent stop may precede a more urgent one only when it costs no detour --
    when the drone was flying through that point anyway and pays only the
    service stop. That is the `free` flag computed while flying the tour.
    """
    for index, leg in enumerate(result.legs):
        if leg.free:
            continue                  # costs nothing but the handover
        if any(later.request.priority < leg.request.priority
               for later in result.legs[index + 1:]):
            return False              # a detour ahead of more urgent work
    return True


def improve_tours(table: RouteTable, fleet: list[Drone],
                  tours: dict[str, list[DeliveryRequest]],
                  reserve_pct: float, service_min: float,
                  policy: str) -> tuple[dict[str, list[DeliveryRequest]], int, float]:
    """Relocate deliveries between tours while it reduces total distance.

    The move that matters is relocating a delivery onto a tour that already
    flies through its destination: the receiving drone pays only a service stop,
    and the donating drone drops an entire detour. That is precisely the
    fly-over the greedy leaves behind, and unlike the forward-only consolidation
    it replaces, this runs over the finished schedule -- so it repairs cases
    where the crossed delivery had already been assigned.

    **Moves are judged on makespan first, then cost.** Total distance and
    makespan are different objectives, and optimising distance alone attacks
    the schedule: the cheapest way to serve a batch is to pile it onto one
    drone and leave the rest idle. Judged on distance this improves; as a
    delivery service it is plainly worse, and it is the imbalance an operator
    notices first -- thirty parcels split 15/9/6 rather than 10/10/10.

    Judging on makespan alone is no better. Shortening the finish is always
    possible by launching another aircraft for one parcel, so a purely
    makespan-led pass spreads the batch thinly across the whole roster and
    doubles the depot round-trips: on Peak Load it cut the finish from 20.2 to
    15.7 minutes while pushing total flying from 28.7 km to 56.5.

    The rule adopted is therefore asymmetric, and reflects which way the
    trade-off is worth taking:

      * a move that lowers cost is accepted whenever it does not delay the
        finish; and
      * a move that shortens the finish is accepted when it costs at most
        `BALANCE_COST_TOLERANCE` more flying.

    The second clause is what rebalances an overloaded drone -- relieving the
    busiest aircraft is what shortens the makespan, so balance follows from the
    objective rather than from a quota on parcels per drone. The tolerance
    bounds how much detour that is worth: without it, thirty parcels split
    15/9/6 stay that way because rebalancing them costs a little distance.
    """
    if policy == "off":
        return tours, 0, 0.0

    by_id = {d.id: d for d in fleet}
    costs = {d.id: simulate_tour(table, d, tours[d.id], reserve_pct, service_min)
             for d in fleet}

    def score(overrides: dict[str, _TourResult] | None = None) -> tuple[float, float]:
        """(makespan, total cost) -- compared lexicographically."""
        merged = dict(costs)
        merged.update(overrides or {})
        return (max((r.finish_min for r in merged.values()), default=0.0),
                sum(r.cost for r in merged.values()))

    def better(after: dict[str, _TourResult]) -> bool:
        m1, c1 = score(after)
        m0, c0 = score()
        if m1 > m0 + _EPS:
            return False                      # finishes later: never worth it
        if m1 < m0 - _EPS:                    # finishes sooner
            return c1 <= c0 * (1 + BALANCE_COST_TOLERANCE) + _EPS
        return c1 < c0 - _EPS                 # same finish, less flying
    budget = max(MIN_IMPROVEMENT_MOVES,
                 MOVES_PER_DELIVERY * sum(len(t) for t in tours.values()))
    moves = 0
    saved = 0.0

    # First-improvement local search: take the first move that lowers the total
    # and restart, rather than scanning for the very best one each time. It
    # converges in far fewer simulations and reaches the same fixpoint -- the
    # loop only ends when no improving move exists at all.
    improving = True
    while improving and moves < budget:
        improving = False

        # --- relocate: move one stop to a better place --------------------
        for donor_id in list(tours):
            donor_stops = tours[donor_id]
            for position, request in enumerate(donor_stops):
                trimmed = donor_stops[:position] + donor_stops[position + 1:]
                donor_after = simulate_tour(table, by_id[donor_id], trimmed,
                                            reserve_pct, service_min)
                if not donor_after.feasible:
                    continue
                gain = costs[donor_id].cost - donor_after.cost
                relieves = (costs[donor_id].finish_min
                            > donor_after.finish_min + _EPS)

                for taker_id in list(tours):
                    # taker == donor is allowed and matters: it reorders a
                    # drone's own stops. Without it a drone keeps whatever
                    # sequence the priority queue happened to produce, which is
                    # how a tour ends up visiting a far customer before a near
                    # one standing on the way.
                    same = taker_id == donor_id
                    base_stops = trimmed if same else tours[taker_id]
                    base_cost = donor_after.cost if same else costs[taker_id].cost
                    if not same and gain <= _EPS and not relieves:
                        continue        # frees neither distance nor time

                    for slot in range(len(base_stops) + 1):
                        if same and slot == position:
                            continue    # putting it back where it was
                        extended = base_stops[:slot] + [request] + base_stops[slot:]
                        taker_after = simulate_tour(table, by_id[taker_id], extended,
                                                    reserve_pct, service_min)
                        if not taker_after.feasible:
                            continue
                        if policy == "safe" and not safe_ordering(taker_after):
                            continue
                        after = ({donor_id: taker_after} if same else
                                 {donor_id: donor_after, taker_id: taker_after})
                        if not better(after):
                            continue
                        net = (costs[donor_id].cost - taker_after.cost if same
                               else gain - (taker_after.cost - base_cost))
                        saved += max(net, 0.0)
                        if same:
                            tours[donor_id] = extended
                            costs[donor_id] = taker_after
                        else:
                            tours[donor_id] = trimmed
                            tours[taker_id] = extended
                            costs[donor_id] = donor_after
                            costs[taker_id] = taker_after
                        moves += 1
                        improving = True
                        break
                    if improving:
                        break
                if improving:
                    break
            if improving:
                break
        if improving:
            continue

        # --- 2-opt: reverse a run of stops within one tour ----------------
        # Relocation moves one stop at a time and can stall on a tour that is
        # simply threaded in the wrong order; reversing a segment escapes that.
        for drone_id in list(tours):
            stops = tours[drone_id]
            for i in range(len(stops) - 1):
                for j in range(i + 2, len(stops) + 1):
                    candidate = stops[:i] + stops[i:j][::-1] + stops[j:]
                    after = simulate_tour(table, by_id[drone_id], candidate,
                                          reserve_pct, service_min)
                    if not after.feasible:
                        continue
                    if policy == "safe" and not safe_ordering(after):
                        continue
                    if not better({drone_id: after}):
                        continue
                    saved += max(costs[drone_id].cost - after.cost, 0.0)
                    tours[drone_id] = candidate
                    costs[drone_id] = after
                    moves += 1
                    improving = True
                    break
                if improving:
                    break
            if improving:
                break

    return tours, moves, saved


# --------------------------------------------------------------------------
# phases 1 and 3 -- the public entry point
# --------------------------------------------------------------------------

def assign_fleet(table: RouteTable, drones: list[Drone],
                 deliveries: list[DeliveryRequest],
                 reserve_pct: float = RESERVE_PCT,
                 service_min: float = SERVICE_TIME_MIN,
                 blocked_nodes: dict[str, str] | None = None,
                 consolidate: str = "safe") -> FleetPlan:
    if consolidate not in POLICIES:
        raise ValueError(f"unknown consolidation policy '{consolidate}'")

    fleet = sorted((d.copy() for d in drones), key=lambda d: d.id)
    by_id = {d.id: d for d in fleet}
    # Copy the requests: planning records status on them, and a caller's batch
    # must survive a planning cycle unchanged.
    batch = [replace(p) for p in deliveries]
    blocked = blocked_nodes or {}

    unserviceable: list[Unserviceable] = []
    # Dispatch order and the phase-1 comparison, kept for the explanations.
    # Local to this call: a module-level store would not survive two plans
    # running against the same process.
    choices: dict[str, tuple[int, str]] = {}
    tours: dict[str, list[DeliveryRequest]] = {d.id: [] for d in fleet}
    costs: dict[str, _TourResult] = {
        d.id: simulate_tour(table, d, [], reserve_pct, service_min) for d in fleet
    }

    # ---- phase 1: greedy, in dispatch order ------------------------------
    for request in DeliveryQueue(batch).drain():
        zone = blocked.get(request.destination)
        if zone is not None:                                    # FR-8.3
            request.status = DeliveryStatus.UNSERVICEABLE
            unserviceable.append(Unserviceable(
                request.id, request.destination, request.priority,
                f"Destination {request.destination} lies inside active no-fly zone '{zone}'"))
            continue

        best_id, best_finish, best_result = None, float("inf"), None
        considered: list[tuple[str, float]] = []
        rejections: list[str] = []

        for drone in fleet:
            trial = simulate_tour(table, drone, tours[drone.id] + [request],
                                  reserve_pct, service_min)
            if not trial.feasible:
                rejections.append(f"{drone.id}: {trial.blocker}")
                continue
            considered.append((drone.id, trial.finish_min))
            if trial.finish_min < best_finish - _EPS or (
                    abs(trial.finish_min - best_finish) <= _EPS
                    and best_id is not None and drone.id < best_id):
                best_id, best_finish, best_result = drone.id, trial.finish_min, trial

        if best_id is None:
            request.status = DeliveryStatus.UNSERVICEABLE
            unserviceable.append(Unserviceable(
                request.id, request.destination, request.priority,
                "; ".join(rejections) or "no drone available"))
            continue

        tours[best_id].append(request)
        costs[best_id] = best_result
        request.status = DeliveryStatus.ASSIGNED
        others = [f"{d} at {f:.1f} min" for d, f in considered if d != best_id]
        choices[request.id] = (
            len(choices),
            f"{best_id} completes at {best_finish:.1f} min"
            + (" vs " + ", ".join(others) if others else " (only eligible drone)"))

    # ---- phase 2: improve, from both constructions ------------------------
    # Greedy scores candidates on completion time alone and has no reason to
    # keep a drone's work in one part of the map. A polar sweep does, and is
    # often the better starting point -- but not always, so both are polished
    # and the cheaper result wins. Local search is sensitive to where it starts;
    # running it twice costs milliseconds and removes that sensitivity.
    assigned = [r for stops in tours.values() for r in stops]
    starts = [tours]
    alternative = sweep_construction(table, fleet, assigned, reserve_pct, service_min)
    if alternative is not None:
        starts.append(alternative)

    best_tours, moves, saved, best_cost = tours, 0, 0.0, None
    for start in starts:
        polished, applied, gained = improve_tours(
            table, fleet, {k: list(v) for k, v in start.items()},
            reserve_pct, service_min, consolidate)
        total = sum(simulate_tour(table, d, polished[d.id],
                                  reserve_pct, service_min).cost for d in fleet)
        if best_cost is None or total < best_cost - _EPS:
            best_tours, moves, saved, best_cost = polished, applied, gained, total
    tours = best_tours

    # ---- phase 3: fly the finished tours ---------------------------------
    assignments: list[Assignment] = []
    for drone in fleet:
        flown = simulate_tour(table, drone, tours[drone.id], reserve_pct, service_min)
        for leg in flown.legs:
            assignments.append(Assignment(
                delivery_id=leg.request.id,
                drone_id=drone.id,
                destination=leg.request.destination,
                priority=leg.request.priority,
                route=leg.route,
                reroute=leg.reroute,
                depart_min=leg.depart_min,
                arrive_min=leg.arrive_min,
                battery_before_pct=leg.battery_before,
                battery_after_pct=leg.battery_after,
                reason=_explain(drone.id, leg, choices),
                enroute=leg.free,
            ))
        drone.assigned = [leg.request.id for leg in flown.legs]
        drone.ready_at_min = flown.finish_min
        drone.current_node = flown.end_node
        drone.battery_pct = flown.end_battery
        drone.distance_flown_km = flown.distance_km
        drone.energy_used_pct = flown.energy_pct

    # Reported in dispatch order: that is what the priority queue established,
    # and sorting by arrival would read chronologically but hide it.
    assignments.sort(key=lambda a: (choices.get(a.delivery_id, (0, ""))[0],
                                    a.delivery_id))
    return FleetPlan(assignments, unserviceable, fleet, moves, round(saved, 3))


def _explain(drone_id: str, leg: _Leg, choices: dict[str, tuple[int, str]]) -> str:
    """Human-readable justification for the decision (FR-10.7)."""
    _seq, original = choices.get(leg.request.id,
                                 (0, f"{drone_id} was assigned this"))
    parts = []
    if original.split()[0] == drone_id:
        parts.append(original)
    else:
        parts.append(f"reassigned to {drone_id} by the improvement pass, which "
                     f"shortened total flight distance ({original} before)")
    if leg.free:
        parts.append(f"{leg.request.destination} lies on the path {drone_id} was "
                     f"already flying, so this cost one service stop and no detour")
    if leg.reroute is not None:
        parts.append(f"routed via charging station {leg.reroute.station_id}, "
                     f"arriving on {leg.reroute.arrival_battery_pct:.1f}% and "
                     f"recharging for {leg.reroute.recharge_min:.1f} min")
    return "; ".join(parts)
