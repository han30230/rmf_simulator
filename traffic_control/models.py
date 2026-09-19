"""Shared domain types for corridor admission control."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class Direction(str, Enum):
    A_TO_B = "A_TO_B"
    B_TO_A = "B_TO_A"

    @property
    def opposite(self) -> "Direction":
        return Direction.B_TO_A if self is Direction.A_TO_B else Direction.A_TO_B


class Decision(str, Enum):
    ADMIT = "ADMIT"
    WAIT = "WAIT"
    BLOCKED = "BLOCKED"


class BlockState(str, Enum):
    FREE = "FREE"
    A_TO_B = "A_TO_B"
    B_TO_A = "B_TO_A"
    BLOCKED = "BLOCKED"


class RobotCorridorState(str, Enum):
    OUTSIDE = "OUTSIDE"
    WAITING = "WAITING"
    RESERVED = "RESERVED"
    INSIDE = "INSIDE"
    CLEARED = "CLEARED"
    EXITED = "EXITED"
    FAULT = "FAULT"


@dataclass
class HoldingBay:
    hb_id: str
    node_id: str
    capacity: int = 1
    geometry: dict[str, Any] | None = None
    occupants: set[str] = field(default_factory=set)
    reservations: set[str] = field(default_factory=set)


@dataclass
class CorridorBlock:
    block_id: str
    group_id: str
    direction_domain: str
    entry_a: str
    entry_b: str
    capacity: int = 1
    require_source_hb_unreserved: bool = False
    geometry: dict[str, Any] | None = None
    edges_a_to_b: set[str] = field(default_factory=set)
    edges_b_to_a: set[str] = field(default_factory=set)
    occupants: dict[str, Direction] = field(default_factory=dict)
    reservations: dict[str, "Reservation"] = field(default_factory=dict)
    fault_reason: str | None = None

    @property
    def state(self) -> BlockState:
        if self.fault_reason:
            return BlockState.BLOCKED
        directions = set(self.occupants.values()) | {
            reservation.direction for reservation in self.reservations.values()
        }
        if not directions:
            return BlockState.FREE
        if len(directions) != 1:
            raise RuntimeError(f"opposite directions present in {self.block_id}")
        direction = next(iter(directions))
        return BlockState(direction.value)


@dataclass(frozen=True)
class Reservation:
    robot_id: str
    block_id: str
    direction: Direction
    destination_hb: str
    source_hb: str | None
    request_time: float
    release_node: str | None = None


@dataclass(frozen=True)
class RouteStep:
    block_id: str
    direction: Direction
    destination_hb: str
    goal_node: str
    source_hb: str | None = None
    release_node: str | None = None


@dataclass(frozen=True)
class RouteIntent:
    route_id: str
    start_nodes: frozenset[str]
    goal_nodes: frozenset[str]
    steps: tuple[RouteStep, ...]
