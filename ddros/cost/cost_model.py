"""Composite edge cost (TDD section 6, FR-4, FR-9).

This module is the only place that knows about weights, wind and restricted
airspace. The search algorithms consume it as an opaque oracle, which is what
allows environmental features to be added without touching Dijkstra or A*
(design principle P7).
"""

from __future__ import annotations

from dataclasses import dataclass

from ..constants import EPS, MU_MAX, MU_MIN, V_AIR_MS, V_MIN_MS, W_MAX_MS
from ..domain.models import Edge
from ..environment.no_fly import NoFlyMask
from ..environment.wind import CALM, Wind
from ..graph.graph import Graph

_SEC_PER_MIN = 60.0
_M_PER_KM = 1000.0


@dataclass(frozen=True)
class Weights:
    """Routing objective weights, constrained to the unit simplex (FR-4.3)."""

    alpha: float = 1.0   # distance
    beta: float = 0.0    # energy
    gamma: float = 0.0   # time

    def __post_init__(self) -> None:
        for name, w in (("alpha", self.alpha), ("beta", self.beta), ("gamma", self.gamma)):
            if not 0.0 <= w <= 1.0:
                raise ValueError(f"{name} must lie in [0, 1], got {w}")
        total = self.alpha + self.beta + self.gamma
        if abs(total - 1.0) > 1e-6:
            raise ValueError(f"alpha + beta + gamma must equal 1, got {total}")

    def as_dict(self) -> dict:
        return {"alpha": self.alpha, "beta": self.beta, "gamma": self.gamma}


SHORTEST_DISTANCE = Weights(1.0, 0.0, 0.0)
MINIMUM_ENERGY = Weights(0.0, 1.0, 0.0)
BALANCED = Weights(0.5, 0.3, 0.2)


def time_min_for(distance_km: float, ground_speed_ms: float) -> float:
    """Equation (3), converting to minutes."""
    return distance_km * _M_PER_KM / ground_speed_ms / _SEC_PER_MIN


class CostModel:
    """Evaluates edge cost, energy and time under a fixed configuration.

    All normalisation references are worst-case constants derived from the graph
    alone. They do not depend on the wind or the weights, which is precisely the
    property the consistency proof of TDD section 7.3.3 relies on.
    """

    __slots__ = ("graph", "weights", "wind", "mask",
                 "d_ref", "g_ref", "t_ref", "_lambda")

    def __init__(self, graph: Graph, weights: Weights = SHORTEST_DISTANCE,
                 wind: Wind = CALM, mask: NoFlyMask | None = None) -> None:
        self.graph = graph
        self.weights = weights
        self.wind = wind
        self.mask = mask

        d_ref = g_ref = 0.0
        rate_min = float("inf")
        for edges in graph.adj.values():
            for e in edges:
                d_ref = max(d_ref, e.distance_km)
                g_ref = max(g_ref, e.base_energy_pct * MU_MAX)
                rate_min = min(rate_min, e.base_energy_pct / e.distance_km)

        self.d_ref = max(d_ref, EPS)
        self.g_ref = max(g_ref, EPS)
        self.t_ref = max(time_min_for(d_ref, V_MIN_MS), EPS)

        # Lambda of equation (7): the least composite cost attributable to one
        # kilometre of travel, under the most favourable possible conditions.
        g_dot_min = (0.0 if rate_min == float("inf") else rate_min) * MU_MIN
        t_dot_min = time_min_for(1.0, V_AIR_MS + W_MAX_MS)
        self._lambda = (
            weights.alpha / self.d_ref
            + weights.beta * g_dot_min / self.g_ref
            + weights.gamma * t_dot_min / self.t_ref
        )

    # -- per-edge quantities ------------------------------------------------

    def edge_energy(self, e: Edge) -> float:
        """Equation (5): wind-adjusted energy consumption, percent of charge."""
        return e.base_energy_pct * self.wind.energy_multiplier(e.bearing_uv_deg)

    def edge_time(self, e: Edge) -> float:
        """Equation (3): wind-adjusted flight time, minutes."""
        return time_min_for(e.distance_km, self.wind.ground_speed_ms(e.bearing_uv_deg))

    def edge_cost(self, e: Edge) -> float:
        """Equation (6): the dimensionless composite cost. Always non-negative."""
        w = self.weights
        return (
            w.alpha * e.distance_km / self.d_ref
            + w.beta * self.edge_energy(e) / self.g_ref
            + w.gamma * self.edge_time(e) / self.t_ref
        )

    def is_open(self, e: Edge) -> bool:
        """False when an active no-fly zone blocks this corridor (FR-8.2)."""
        return self.mask is None or not self.mask.is_blocked(e)

    # -- heuristic support --------------------------------------------------

    @property
    def lambda_min(self) -> float:
        """Least composite cost per kilometre; the A* heuristic's scale factor."""
        return self._lambda

    def config_key(self) -> tuple:
        """Cache key for TDD section 11. Weights are rounded so slider jitter
        does not defeat the cache."""
        w = self.weights
        return (
            round(w.alpha, 3), round(w.beta, 3), round(w.gamma, 3),
            round(self.wind.speed_ms, 2), round(self.wind.bearing_deg, 1),
            self.mask.active_ids if self.mask else (),
        )
