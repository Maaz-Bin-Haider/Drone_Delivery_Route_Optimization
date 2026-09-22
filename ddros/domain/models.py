"""Core domain entities (TDD section 5.1)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum

from ..analysis.instrumentation import SearchStats


class NodeType(str, Enum):
    WAREHOUSE = "warehouse"
    CUSTOMER = "customer"
    CHARGING_STATION = "charging_station"
    WAYPOINT = "waypoint"


class Priority(IntEnum):
    """Lower value means higher precedence, so the raw value is the heap key."""

    URGENT = 0
    HIGH = 1
    NORMAL = 2


class DeliveryStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    DELIVERED = "delivered"
    UNSERVICEABLE = "unserviceable"


@dataclass(frozen=True)
class Node:
    id: str
    name: str
    lat: float
    lon: float
    type: NodeType


@dataclass(frozen=True)
class Edge:
    """A directed traversal of an air corridor.

    The graph is undirected, but cost is direction-dependent because of wind, so
    each corridor is stored twice with opposing bearings. `bearing_uv_deg` is
    always the bearing actually flown when this edge is traversed.
    """

    u: str                    # departure node
    v: str                    # arrival node
    distance_km: float
    base_energy_pct: float
    bearing_uv_deg: float


@dataclass
class Drone:
    id: str
    current_node: str
    battery_pct: float
    ready_at_min: float = 0.0
    assigned: list[str] = field(default_factory=list)
    distance_flown_km: float = 0.0
    energy_used_pct: float = 0.0

    def copy(self) -> "Drone":
        return Drone(self.id, self.current_node, self.battery_pct,
                     self.ready_at_min, list(self.assigned),
                     self.distance_flown_km, self.energy_used_pct)


@dataclass
class DeliveryRequest:
    id: str
    destination: str
    priority: Priority
    sequence: int
    status: DeliveryStatus = DeliveryStatus.PENDING


@dataclass(frozen=True)
class NoFlyZone:
    id: str
    name: str
    shape: str                                   # "circle" | "polygon"
    active: bool = True
    centre: tuple[float, float] | None = None
    radius_km: float | None = None
    vertices: tuple[tuple[float, float], ...] | None = None


@dataclass(frozen=True)
class Route:
    """A computed route, carrying the evidence behind it (design principle P2)."""

    path: tuple[str, ...]
    distance_km: float
    energy_pct: float
    time_min: float
    cost: float
    algorithm: str
    stats: SearchStats

    def as_dict(self) -> dict:
        return {
            "path": list(self.path),
            "distance_km": round(self.distance_km, 3),
            "energy_pct": round(self.energy_pct, 2),
            "time_min": round(self.time_min, 2),
            "cost": round(self.cost, 6),
            "algorithm": self.algorithm,
            "stats": self.stats.as_dict(),
        }


@dataclass(frozen=True)
class ChargingReroute:
    """A two-leg route through a charging station (TDD section 8.3)."""

    station_id: str
    leg_one: Route
    leg_two: Route
    arrival_battery_pct: float
    recharge_min: float

    @property
    def total_time_min(self) -> float:
        return self.leg_one.time_min + self.recharge_min + self.leg_two.time_min

    @property
    def total_distance_km(self) -> float:
        return self.leg_one.distance_km + self.leg_two.distance_km

    @property
    def path(self) -> tuple[str, ...]:
        return self.leg_one.path + self.leg_two.path[1:]

    def as_dict(self) -> dict:
        return {
            "station_id": self.station_id,
            "leg_one": self.leg_one.as_dict(),
            "leg_two": self.leg_two.as_dict(),
            "arrival_battery_pct": round(self.arrival_battery_pct, 2),
            "recharge_min": round(self.recharge_min, 2),
        }


@dataclass(frozen=True)
class Assignment:
    delivery_id: str
    drone_id: str
    destination: str
    priority: Priority
    route: Route
    reroute: ChargingReroute | None
    depart_min: float
    arrive_min: float
    battery_before_pct: float
    battery_after_pct: float
    reason: str

    def as_dict(self) -> dict:
        return {
            "delivery_id": self.delivery_id,
            "drone_id": self.drone_id,
            "destination": self.destination,
            "priority": self.priority.name,
            "route": self.route.as_dict(),
            "charging_stop": self.reroute.as_dict() if self.reroute else None,
            "depart_min": round(self.depart_min, 2),
            "arrive_min": round(self.arrive_min, 2),
            "battery_before_pct": round(self.battery_before_pct, 2),
            "battery_after_pct": round(self.battery_after_pct, 2),
            "reason": self.reason,
        }


@dataclass(frozen=True)
class Unserviceable:
    delivery_id: str
    destination: str
    priority: Priority
    reason: str

    def as_dict(self) -> dict:
        return {
            "delivery_id": self.delivery_id,
            "destination": self.destination,
            "priority": self.priority.name,
            "reason": self.reason,
        }
