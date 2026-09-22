"""The planning cycle (TDD section 3.3).

Steps 2-4 -- environment, cost model, all-pairs table -- are pure functions of
the configuration and are cached. Steps 5-9 are cheap lookups over that table.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from ..algorithms.astar import astar_route
from ..algorithms.dijkstra import dijkstra_route
from ..constants import RESERVE_PCT, SERVICE_TIME_MIN
from ..cost.cost_model import (BALANCED, MINIMUM_ENERGY, SHORTEST_DISTANCE,
                               CostModel, Weights)
from ..domain.loader import Scenario
from ..domain.models import Route
from ..environment.no_fly import NoFlyMask
from ..environment.wind import CALM, Wind
from ..scheduling.assignment import FleetPlan
from ..scheduling.fleet_sizing import DEFAULT_TOLERANCE, FleetSizing, size_fleet
from .cache import ConfigCache, RouteTable

_COST_EQUALITY_TOLERANCE = 1e-9


@dataclass(frozen=True)
class PlanConfig:
    """One complete routing configuration."""

    weights: Weights = SHORTEST_DISTANCE
    wind: Wind = CALM
    active_zones: tuple[str, ...] = ()
    algorithm: str = "astar"
    reserve_pct: float = RESERVE_PCT
    service_min: float = SERVICE_TIME_MIN
    fleet_size: int | None = None            # None selects the size automatically
    fleet_tolerance: float = DEFAULT_TOLERANCE

    def as_dict(self) -> dict:
        return {
            "weights": self.weights.as_dict(),
            "wind": self.wind.as_dict(),
            "active_no_fly_zones": list(self.active_zones),
            "algorithm": self.algorithm,
            "reserve_pct": self.reserve_pct,
            "fleet_size": self.fleet_size,
            "fleet_tolerance": self.fleet_tolerance,
        }


class HeuristicViolation(AssertionError):
    """Dijkstra and A* disagreed, which falsifies heuristic admissibility.

    Raised rather than swallowed: an inadmissible heuristic silently returns
    sub-optimal routes, so this must fail loudly (design principle P6, FR-11.6).
    """


@dataclass
class Simulator:
    scenario: Scenario
    cache: ConfigCache = field(default_factory=ConfigCache)

    # -- configuration ------------------------------------------------------

    def cost_model(self, config: PlanConfig) -> CostModel:
        zones = [
            replace(z, active=(z.id in config.active_zones)) for z in self.scenario.zones
        ]
        mask = NoFlyMask(self.scenario.graph, zones)
        return CostModel(self.scenario.graph, config.weights, config.wind, mask)

    def table(self, config: PlanConfig) -> RouteTable:
        return self.cache.get(self.scenario.graph, self.cost_model(config))

    # -- single routes ------------------------------------------------------

    def route(self, source: str, target: str, config: PlanConfig) -> Route | None:
        cm = self.cost_model(config)
        if config.algorithm == "dijkstra":
            return dijkstra_route(self.scenario.graph, source, target, cm)
        return astar_route(self.scenario.graph, source, target, cm)

    def compare(self, source: str, target: str, config: PlanConfig) -> dict:
        """Run both routers on one pair and check they agree (FR-3.4, FR-11.2)."""
        cm = self.cost_model(config)
        d = dijkstra_route(self.scenario.graph, source, target, cm)
        a = astar_route(self.scenario.graph, source, target, cm)
        if d is None or a is None:
            return {"unreachable": True, "source": source, "target": target}
        agree = abs(d.cost - a.cost) <= _COST_EQUALITY_TOLERANCE
        if not agree:
            raise HeuristicViolation(
                f"{source}->{target}: Dijkstra cost {d.cost!r} != A* cost {a.cost!r}"
            )
        expanded_ratio = (a.stats.nodes_expanded / d.stats.nodes_expanded
                          if d.stats.nodes_expanded else 1.0)
        return {
            "source": source, "target": target, "agree": agree,
            "dijkstra": d.as_dict(), "astar": a.as_dict(),
            "expansion_ratio": round(expanded_ratio, 4),
        }

    def alternatives(self, source: str, target: str, config: PlanConfig) -> dict:
        """Minimum-distance vs minimum-energy route, side by side (FR-4.7)."""
        shortest = self.route(source, target, replace(config, weights=SHORTEST_DISTANCE))
        greenest = self.route(source, target, replace(config, weights=MINIMUM_ENERGY))
        out = {
            "source": source, "target": target,
            "min_distance": shortest.as_dict() if shortest else None,
            "min_energy": greenest.as_dict() if greenest else None,
        }
        if shortest and greenest:
            differ = shortest.path != greenest.path
            out["differ"] = differ
            if differ and shortest.distance_km > 0 and greenest.energy_pct > 0:
                out["annotation"] = (
                    f"{(greenest.distance_km / shortest.distance_km - 1) * 100:.1f}% longer, "
                    f"{(1 - greenest.energy_pct / shortest.energy_pct) * 100:.1f}% less energy"
                )
            elif not differ:
                out["annotation"] = "shortest route is also the most energy-efficient"
        return out

    # -- full plan ----------------------------------------------------------

    def plan(self, config: PlanConfig = PlanConfig()) -> dict:
        cm = self.cost_model(config)
        table = self.cache.get(self.scenario.graph, cm)
        blocked = {
            nid: name
            for nid, name in (
                (n, cm.mask.node_blocked_by(n)) for n in self.scenario.graph.nodes
            )
            if name is not None
        } if cm.mask else {}

        deliveries = _fresh(self.scenario.deliveries)
        # The roster is not a launch order: how many aircraft to dispatch is
        # itself a decision, and on this problem a larger fleet is not always a
        # faster one (TDD section 9.4).
        sizing = size_fleet(table, self.scenario.drones, deliveries,
                            config.reserve_pct, config.service_min, blocked,
                            config.fleet_tolerance, config.fleet_size)
        fleet_plan = sizing.plan

        if config.algorithm == "astar":
            fleet_plan = self._restate_with_astar(fleet_plan, cm)

        return self._render(fleet_plan, config, table, cm, sizing)

    def _restate_with_astar(self, plan: FleetPlan, cm: CostModel) -> FleetPlan:
        """Recompute each direct route with A* and verify it matches Dijkstra.

        The all-pairs table is necessarily built with Dijkstra -- there is no
        all-targets A* -- so when the operator selects A* the reported route is
        recomputed per assignment. That makes the 'algorithm used' column honest
        and turns every planning cycle into a live admissibility check.
        """
        out = []
        for a in plan.assignments:
            if a.reroute is not None:
                out.append(a)
                continue
            source = a.route.path[0]
            alt = astar_route(self.scenario.graph, source, a.destination, cm)
            if alt is None:
                out.append(a)
                continue
            if abs(alt.cost - a.route.cost) > _COST_EQUALITY_TOLERANCE:
                raise HeuristicViolation(
                    f"{source}->{a.destination}: Dijkstra {a.route.cost!r} "
                    f"!= A* {alt.cost!r}"
                )
            out.append(replace(a, route=alt))
        return FleetPlan(out, plan.unserviceable, plan.drones)

    def _render(self, plan: FleetPlan, config: PlanConfig, table: RouteTable,
                cm: CostModel, sizing: FleetSizing) -> dict:
        total_distance = sum(a.route.distance_km for a in plan.assignments)
        total_energy = sum(a.route.energy_pct for a in plan.assignments)
        return {
            "scenario": self.scenario.name,
            "config": config.as_dict(),
            "makespan_min": round(plan.makespan_min, 2),
            "totals": {
                "distance_km": round(total_distance, 2),
                "energy_pct": round(total_energy, 2),
                "delivered": len(plan.assignments),
                "unserviceable": len(plan.unserviceable),
            },
            "fleet": sizing.as_dict(),
            "assignments": [a.as_dict() for a in plan.assignments],
            "unserviceable": [u.as_dict() for u in plan.unserviceable],
            "drones": [
                {
                    "id": d.id,
                    "deliveries": len(d.assigned),
                    "packages": list(d.assigned),
                    "distance_km": round(d.distance_flown_km, 2),
                    "energy_used_pct": round(d.energy_used_pct, 2),
                    "battery_pct": round(d.battery_pct, 2),
                    "busy_min": round(d.ready_at_min, 2),
                    "idle_min": round(max(0.0, plan.makespan_min - d.ready_at_min), 2),
                }
                for d in plan.drones
            ],
            # Deterministic: the same configuration always produces these counts.
            "search": {
                "all_pairs_searches": table.searches,
                "all_pairs_nodes_expanded": table.nodes_expanded,
            },
            # Volatile: wall-clock and cache figures depend on execution history,
            # not on the inputs, so they are kept out of the plan proper. NFR-11
            # constrains the decisions, not the telemetry describing them.
            "telemetry": {
                "cache_hits": self.cache.hits,
                "cache_misses": self.cache.misses,
            },
        }


def _fresh(deliveries):
    """Copies, so a planning cycle never mutates the stored batch."""
    return [replace(p) for p in deliveries]
