"""Choosing how many drones to dispatch (FR-7.9).

The fleet is a roster, not a launch order. Sending every aircraft is not always
right, for two reasons the measurements bear out:

  * Beyond some point extra drones buy nothing. Once the batch is spread thinly
    the makespan is set by the single longest delivery, and further aircraft sit
    idle -- launched, tracked and charged for no gain.
  * Adding a drone can make the schedule actively *worse*. List scheduling is
    not monotone in the number of machines (TDD section 9.4): a delivery's
    duration depends on where its drone currently is, so one more aircraft
    changes every subsequent choice and can leave some drone with a worse tour.
    This holds even with identical batteries and no charging detours, so it is
    the scheduler's behaviour rather than an artefact of the fleet's state.

So the size is chosen rather than assumed. Each candidate size is planned and
scored, and the smallest fleet that comes within a tolerance of the best
achievable makespan wins -- launching a sixth aircraft to save four seconds is
not a trade an operator would make.

Complexity: O(N) assignments over a cached route table, so
O(N * (P * N + P log P)). For a roster of eight and a batch of twenty this is
a few milliseconds, which is why an exhaustive sweep is affordable at all.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..constants import RESERVE_PCT, SERVICE_TIME_MIN
from ..domain.models import DeliveryRequest, Drone
from ..simulation.cache import RouteTable
from .assignment import FleetPlan, assign_fleet

DEFAULT_TOLERANCE = 0.03      # a fleet within 3% of the best makespan is "as good"


@dataclass(frozen=True)
class SizingOption:
    """One candidate fleet size and what it achieved."""

    drones: int
    makespan_min: float
    unserviceable: int
    charging_detours: int
    idle_drones: int

    def as_dict(self) -> dict:
        return {
            "drones": self.drones,
            "makespan_min": round(self.makespan_min, 2),
            "unserviceable": self.unserviceable,
            "charging_detours": self.charging_detours,
            "idle_drones": self.idle_drones,
        }


@dataclass
class FleetSizing:
    chosen: int
    """The size the sweep selected. Indexes into `options`."""
    reason: str
    options: list[SizingOption]
    plan: FleetPlan
    dispatched: list[str]
    reserve: list[str]

    @property
    def launched(self) -> int:
        """Drones that actually flew.

        Not always the size selected: the improvement pass runs inside the
        assignment and can consolidate a tour away entirely, leaving a selected
        drone with nothing to carry. `chosen` records the decision, `launched`
        records the outcome, and reporting them separately keeps both honest.
        """
        return len(self.dispatched)

    def as_dict(self) -> dict:
        return {
            "chosen": self.chosen,
            "launched": self.launched,
            "reason": self.reason,
            "dispatched": list(self.dispatched),
            "reserve": list(self.reserve),
            "options": [o.as_dict() for o in self.options],
        }


def launch_order(roster: list[Drone]) -> list[Drone]:
    """Best-charged aircraft first, ties broken by id so the order is total.

    Charge is the only thing that distinguishes otherwise identical drones, and
    a fuller battery is strictly more useful, so it is the natural launch order.
    """
    return sorted(roster, key=lambda d: (-d.battery_pct, d.id))


def size_fleet(table: RouteTable, roster: list[Drone],
               deliveries: list[DeliveryRequest],
               reserve_pct: float = RESERVE_PCT,
               service_min: float = SERVICE_TIME_MIN,
               blocked_nodes: dict[str, str] | None = None,
               tolerance: float = DEFAULT_TOLERANCE,
               fixed: int | None = None,
               consolidate: str = "safe") -> FleetSizing:
    """Plan with every fleet size and keep the smallest one that is good enough.

    `fixed` overrides the choice, so an operator can force a size and see what
    it costs -- which is how the anomaly above is demonstrated rather than just
    asserted.
    """
    ordered = launch_order(roster)
    if not ordered:
        raise ValueError("the roster is empty")

    if not deliveries:
        empty = assign_fleet(table, ordered[:1], [], reserve_pct,
                             service_min, blocked_nodes, consolidate)
        return FleetSizing(0, "no orders to deliver", [], empty, [],
                           [d.id for d in ordered])

    sizes = [fixed] if fixed else range(1, len(ordered) + 1)
    options: list[SizingOption] = []
    plans: dict[int, FleetPlan] = {}

    for k in sizes:
        k = max(1, min(int(k), len(ordered)))
        plan = assign_fleet(table, ordered[:k], deliveries, reserve_pct,
                            service_min, blocked_nodes, consolidate)
        plans[k] = plan
        options.append(SizingOption(
            drones=k,
            makespan_min=plan.makespan_min,
            unserviceable=len(plan.unserviceable),
            charging_detours=sum(1 for a in plan.assignments if a.reroute),
            idle_drones=sum(1 for d in plan.drones if not d.assigned),
        ))

    if fixed:
        k = options[0].drones
        chosen, reason = k, f"fleet size fixed at {k} by the operator"
    else:
        chosen, reason = _select(options, tolerance)

    plan = plans[chosen]
    # The improvement pass runs inside the assignment and can consolidate a
    # tour away entirely, leaving a selected drone with nothing to carry.
    # Reporting the size the sizer picked would then overstate the fleet, so
    # what is reported is what actually flew.
    flew = [d.id for d in plan.drones if d.assigned]
    idle_but_selected = [d.id for d in ordered[:chosen] if d.id not in flew]
    reserve = [d.id for d in ordered[chosen:]] + idle_but_selected
    if idle_but_selected:
        reason += (f". The improvement pass then consolidated onto {len(flew)}, "
                   f"leaving {', '.join(idle_but_selected)} on the ground")
    return FleetSizing(chosen, reason, options, plan, flew, reserve)


def _select(options: list[SizingOption], tolerance: float) -> tuple[int, str]:
    """Fewest unserviceable first, then the smallest fleet within tolerance."""
    # Makespan is only comparable between plans that serve the same number of
    # orders: a fleet too small to finish the batch posts a *short* makespan
    # simply because it delivered less. Service level is therefore the first
    # criterion, never speed.
    fewest_unserved = min(o.unserviceable for o in options)
    viable = [o for o in options if o.unserviceable == fewest_unserved]
    excluded = [o for o in options if o.unserviceable > fewest_unserved]

    best = min(viable, key=lambda o: o.makespan_min)
    threshold = best.makespan_min * (1 + tolerance)
    chosen = min((o for o in viable if o.makespan_min <= threshold),
                 key=lambda o: o.drones)

    if chosen.drones == len(options):
        reason = (f"every drone earns its place: {chosen.drones} gives "
                  f"{chosen.makespan_min:.1f} min")
    elif abs(chosen.makespan_min - best.makespan_min) < 1e-9:
        reason = (f"{chosen.drones} drone{'s' if chosen.drones > 1 else ''} "
                  f"reaches the best makespan of {chosen.makespan_min:.1f} min; "
                  f"the remaining {len(options) - chosen.drones} would sit idle")
    else:
        saving = chosen.makespan_min - best.makespan_min
        reason = (f"{chosen.drones} drone{'s' if chosen.drones > 1 else ''} at "
                  f"{chosen.makespan_min:.1f} min is within {tolerance:.0%} of the "
                  f"best {best.makespan_min:.1f} min from {best.drones}; the extra "
                  f"aircraft would save only {saving:.1f} min")

    if excluded:
        smallest_viable = min(o.drones for o in viable)
        reason += (f". Fleets below {smallest_viable} were ruled out for leaving "
                   f"orders undelivered, not for being slow")

    worse = [o for o in options if o.drones > chosen.drones
             and o.makespan_min > chosen.makespan_min + 1e-9
             and o.unserviceable == chosen.unserviceable]
    if worse:
        reason += (f". Note {worse[0].drones} drones would be *slower* "
                   f"({worse[0].makespan_min:.1f} min) -- see TDD section 9.4")
    return chosen.drones, reason
