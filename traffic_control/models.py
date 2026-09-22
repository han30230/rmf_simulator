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


@dataclass(frozen=True)
class SafeStopGroup:
    group_id: str
    members: tuple[str, ...]


@dataclass(frozen=True)
class CorridorChain:
    chain_id: str
    block_ids: tuple[str, ...]
    safe_stop_ids: tuple[str, ...]
    max_active_robots: int


@dataclass(frozen=True)
class ChainPath:
    chain_id: str
    direction: Direction
    block_ids: tuple[str, ...]
    safe_stop_ids: tuple[str, ...]
    source_group: str
    destination_group: str
    source_slot: str
    destination_slot: str


@dataclass
class MovementAuthority:
    authority_id: str
    robot_id: str
    chain_id: str
    direction: Direction
    source_group: str
    source_slot: str
    destination_group: str
    destination_slot: str
    goal_node: str
    block_ids: tuple[str, ...]
    request_time: float
    released_blocks: set[str] = field(default_factory=set)

    @property
    def unreleased_blocks(self) -> tuple[str, ...]:
        return tuple(
            block_id
            for block_id in self.block_ids
            if block_id not in self.released_blocks
        )


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
    release_node_a_to_b: str | None = None
    release_node_b_to_a: str | None = None
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

    def release_node(self, direction: Direction) -> str | None:
        return (
            self.release_node_a_to_b
            if direction is Direction.A_TO_B
            else self.release_node_b_to_a
        )


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
    requires_opposite_routes_cleared: bool = False


@dataclass(frozen=True)
class RouteIntent:
    route_id: str
    start_nodes: frozenset[str]
    goal_nodes: frozenset[str]
    steps: tuple[RouteStep, ...]
    requires_no_opposite_jobs: bool = False
