"""Uniform wind field (TDD section 6.1, FR-9).

One wind vector drives both flight time and energy consumption, rather than two
unrelated correction factors: modelling the drone as drawing roughly constant
power in cruise, energy is proportional to time aloft, so a tailwind that raises
ground speed lowers both.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..constants import MU_MAX, MU_MIN, V_AIR_MS, V_MIN_MS, W_MAX_MS


@dataclass(frozen=True)
class Wind:
    """Wind of `speed_ms` blowing *toward* `bearing_deg` (degrees from north)."""

    speed_ms: float = 0.0
    bearing_deg: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.speed_ms <= W_MAX_MS:
            raise ValueError(
                f"wind speed {self.speed_ms} outside modelled range [0, {W_MAX_MS}]"
            )

    def along_track_ms(self, edge_bearing_deg: float) -> float:
        """Component of the wind along the direction of flight, equation (1).

        Positive is a tailwind, negative a headwind.
        """
        delta = math.radians(edge_bearing_deg - self.bearing_deg)
        return self.speed_ms * math.cos(delta)

    def ground_speed_ms(self, edge_bearing_deg: float) -> float:
        """Equation (2). Floored at V_MIN_MS so time stays finite and positive."""
        return max(V_AIR_MS + self.along_track_ms(edge_bearing_deg), V_MIN_MS)

    def energy_multiplier(self, edge_bearing_deg: float) -> float:
        """Equation (4), clamped to keep the constant-power model defensible."""
        mu = V_AIR_MS / self.ground_speed_ms(edge_bearing_deg)
        return min(max(mu, MU_MIN), MU_MAX)

    def as_dict(self) -> dict:
        return {"speed_ms": self.speed_ms, "bearing_deg": self.bearing_deg}


CALM = Wind(0.0, 0.0)
