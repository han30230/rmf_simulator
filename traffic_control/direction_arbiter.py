"""Thread-safe directional admission state machine."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import threading
import time
from uuid import uuid4

from .corridor_chain import PlannedAuthority
from .corridor_registry import CorridorRegistry
from .models import (
    Decision,
    Direction,
    MovementAuthority,
    Reservation,
    RobotCorridorState,
)

logger = logging.getLogger(__name__)


@dataclass
class _DomainState:
    active_direction: Direction | None = None
    batch_closed: bool = False


@dataclass
class _WaitingQueues:
    by_direction: dict[Direction, list[Reservation]] = field(
        default_factory=lambda: {
            Direction.A_TO_B: [],
            Direction.B_TO_A: [],
        }
    )


class DirectionArbiter:
    def __init__(self, registry: CorridorRegistry) -> None:
        self.registry = registry
        self._lock = threading.RLock()
        self._domains: dict[str, _DomainState] = {
            block.direction_domain: _DomainState()
            for block in registry.blocks.values()
        }
        self._waiting: dict[str, _WaitingQueues] = {
            block_id: _WaitingQueues() for block_id in registry.blocks
        }
        self._grants: dict[tuple[str, str], Reservation] = {}
        self._robot_states: dict[tuple[str, str], RobotCorridorState] = {}
        self._authorities: dict[str, MovementAuthority] = {}
        self._pending_authorities: dict[str, MovementAuthority] = {}

    def can_reserve_path(
        self,
        block_ids: tuple[str, ...],
        direction: Direction | str,
        destination_slot: str,
        *,
        robot_id: str = "",
        chain_id: str | None = None,
    ) -> bool:
        direction = Direction(direction)
        with self._lock:
            if destination_slot not in self.registry.holding_bays:
                return False
            if chain_id is not None:
                chain = self.registry.corridor_chains[chain_id]
                active = sum(
                    item.chain_id == chain_id
                    for item in self._authorities.values()
                )
                if robot_id not in self._authorities and active >= chain.max_active_robots:
                    return False
            for block_id in block_ids:
                block = self.registry.blocks[block_id]
                if block.fault_reason:
                    return False
                used = (set(block.occupants) | set(block.reservations)) - {robot_id}
                if len(used) >= block.capacity:
                    return False
                directions = {
                    value
                    for candidate, value in block.occupants.items()
                    if candidate != robot_id
                } | {
                    value.direction
                    for candidate, value in block.reservations.items()
                    if candidate != robot_id
                }
                if directions and directions != {direction}:
                    return False
                domain = self._domains[block.direction_domain]
                busy = self._domain_busy(block.direction_domain)
                if busy and (
                    domain.active_direction is not direction or domain.batch_closed
                ):
                    return False
                if (
                    not busy
                    and domain.batch_closed
                    and domain.active_direction is direction
                ):
                    return False
            bay = self.registry.holding_bays[destination_slot]
            used_bay = (bay.occupants | bay.reservations) - {robot_id}
            return len(used_bay) < bay.capacity

    def request_authority(
        self,
        planned: PlannedAuthority,
        *,
        robot_id: str,
        request_time: float | None = None,
    ) -> Decision:
        with self._lock:
            if robot_id in self._authorities:
                return Decision.ADMIT
            if robot_id in self._pending_authorities:
                self._reconcile_authorities()
                return self.decision_for_authority(robot_id)
            authority = MovementAuthority(
                authority_id=uuid4().hex,
                robot_id=robot_id,
                chain_id=planned.chain_id,
                direction=planned.direction,
                source_group=planned.source_group,
                source_slot=planned.source_slot,
                destination_group=planned.destination_group,
                destination_slot=planned.destination_slot,
                goal_node=planned.goal_node,
                block_ids=planned.block_ids,
                request_time=(
                    time.monotonic() if request_time is None else request_time
                ),
            )
            if any(
                self.registry.blocks[block_id].fault_reason
                for block_id in authority.block_ids
            ):
                return Decision.BLOCKED
            if self._authority_resources_available(authority):
                self._grant_authority(authority)
                return Decision.ADMIT
            self._pending_authorities[robot_id] = authority
            for block_id in authority.block_ids:
                block = self.registry.blocks[block_id]
                domain = self._domains[block.direction_domain]
                if (
                    domain.active_direction is not None
                    and domain.active_direction is not authority.direction
                ):
                    domain.batch_closed = True
            return Decision.WAIT

    def authority_for_robot(self, robot_id: str) -> MovementAuthority | None:
        with self._lock:
            return self._authorities.get(robot_id)

    def decision_for_authority(self, robot_id: str) -> Decision:
        with self._lock:
            authority = self._authorities.get(robot_id)
            if authority is not None:
                if any(
                    self.registry.blocks[block_id].fault_reason
                    for block_id in authority.unreleased_blocks
                ):
                    return Decision.BLOCKED
                return Decision.ADMIT
            pending = self._pending_authorities.get(robot_id)
            if pending is not None and any(
                self.registry.blocks[block_id].fault_reason
                for block_id in pending.block_ids
            ):
                return Decision.BLOCKED
            return Decision.WAIT

    def cancel_authority(self, robot_id: str) -> bool:
        with self._lock:
            pending = self._pending_authorities.pop(robot_id, None)
            authority = self._authorities.get(robot_id)
            if authority is not None and any(
                robot_id in self.registry.blocks[block_id].occupants
                for block_id in authority.unreleased_blocks
            ):
                if pending is not None:
                    self._pending_authorities[robot_id] = pending
                return False
            authority = self._authorities.pop(robot_id, None)
            changed = pending is not None or authority is not None
            if authority is not None:
                for block_id in authority.block_ids:
                    block = self.registry.blocks[block_id]
                    block.reservations.pop(robot_id, None)
                    self._grants.pop((robot_id, block_id), None)
                    self._robot_states.pop((robot_id, block_id), None)
                self.registry.holding_bays[
                    authority.destination_slot
                ].reservations.discard(robot_id)
                self._reconcile_authority_domains(authority)
            if changed:
                self._reconcile_authorities()
            return changed

    def mark_authority_arrived(self, robot_id: str) -> bool:
        with self._lock:
            authority = self._authorities.pop(robot_id, None)
            if authority is None:
                return False
            for block_id in authority.block_ids:
                block = self.registry.blocks[block_id]
                block.occupants.pop(robot_id, None)
                block.reservations.pop(robot_id, None)
                self._grants.pop((robot_id, block_id), None)
                self._robot_states[(robot_id, block_id)] = RobotCorridorState.EXITED
            source = self.registry.holding_bays[authority.source_slot]
            source.occupants.discard(robot_id)
            source.reservations.discard(robot_id)
            destination = self.registry.holding_bays[authority.destination_slot]
            destination.reservations.discard(robot_id)
            destination.occupants.add(robot_id)
            self._reconcile_authority_domains(authority)
            self._reconcile_authorities()
            self._assert_invariants()
            return True

    def request(
        self,
        robot_id: str,
        block_id: str,
        direction: Direction | str,
        destination_hb: str,
        *,
        source_hb: str | None = None,
        request_time: float | None = None,
        release_node: str | None = None,
    ) -> Decision:
        direction = Direction(direction)
        with self._lock:
            logger.info(
                "[TRAFFIC] ARBITER_REQUEST robot=%s block=%s dir=%s hb=%s",
                robot_id,
                block_id,
                direction.value,
                destination_hb,
            )
            block = self.registry.blocks[block_id]
            if destination_hb not in self.registry.holding_bays:
                raise KeyError(destination_hb)
            key = (robot_id, block_id)
            if key in self._grants or robot_id in block.occupants:
                return Decision.ADMIT
            if block.fault_reason:
                return Decision.BLOCKED
            if self._find_waiting(robot_id, block_id) is not None:
                return Decision.WAIT

            reservation = Reservation(
                robot_id=robot_id,
                block_id=block_id,
                direction=direction,
                destination_hb=destination_hb,
                source_hb=source_hb,
                request_time=time.monotonic() if request_time is None else request_time,
                release_node=release_node,
            )
            domain = self._domains[block.direction_domain]
            if domain.active_direction is None:
                self._enqueue(reservation)
                self._robot_states[key] = RobotCorridorState.WAITING
                self._reconcile_domain(block.direction_domain)
                return self.decision_for(robot_id, block_id)
            elif domain.active_direction is not direction:
                domain.batch_closed = True
                self._enqueue(reservation)
                self._robot_states[key] = RobotCorridorState.WAITING
                logger.info(
                    "[TRAFFIC] ARBITER_WAIT robot=%s block=%s dir=%s reason=opposite_direction",
                    robot_id, block_id, direction.value,
                )
                self._reconcile_domain(block.direction_domain)
                return self.decision_for(robot_id, block_id)

            if domain.batch_closed or not self._resources_available(reservation):
                self._enqueue(reservation)
                self._robot_states[key] = RobotCorridorState.WAITING
                reason = "batch_closed" if domain.batch_closed else "resource_unavailable"
                logger.info(
                    "[TRAFFIC] ARBITER_WAIT robot=%s block=%s dir=%s reason=%s",
                    robot_id, block_id, direction.value, reason,
                )
                return Decision.WAIT

            self._grant(reservation)
            return Decision.ADMIT

    def mark_entered(self, robot_id: str, block_id: str) -> None:
        with self._lock:
            key = (robot_id, block_id)
            if self._robot_states.get(key) is RobotCorridorState.CLEARED:
                return
            block = self.registry.blocks[block_id]
            reservation = self._grants.get(key)
            if reservation is None:
                if robot_id in block.occupants:
                    return
                raise ValueError(f"robot {robot_id} has no grant for {block_id}")
            block.reservations.pop(robot_id, None)
            block.occupants[robot_id] = reservation.direction
            if reservation.source_hb:
                source = self.registry.holding_bays[reservation.source_hb]
                source.occupants.discard(robot_id)
                source.reservations.discard(robot_id)
            self._robot_states[key] = RobotCorridorState.INSIDE
            for domain_id in self._domains:
                self._reconcile_domain(domain_id)
            self._assert_invariants()
            logger.info(
                "[TRAFFIC] ROBOT_ENTERED_BLOCK robot=%s block=%s",
                robot_id, block_id,
            )

    def mark_cleared(self, robot_id: str, block_id: str) -> bool:
        """Release conflict resources while retaining the destination bay."""
        with self._lock:
            key = (robot_id, block_id)
            reservation = self._grants.get(key)
            if reservation is None or reservation.release_node is None:
                return False
            if self._robot_states.get(key) is RobotCorridorState.CLEARED:
                return False
            block = self.registry.blocks[block_id]
            block.occupants.pop(robot_id, None)
            block.reservations.pop(robot_id, None)
            if reservation.source_hb:
                source = self.registry.holding_bays[reservation.source_hb]
                source.occupants.discard(robot_id)
                source.reservations.discard(robot_id)
            self._robot_states[key] = RobotCorridorState.CLEARED
            authority = self._authorities.get(robot_id)
            if authority is not None and block_id in authority.block_ids:
                authority.released_blocks.add(block_id)
            logger.info(
                "[TRAFFIC] ROBOT_CLEARED_BLOCK robot=%s block=%s release_node=%s",
                robot_id,
                block_id,
                reservation.release_node,
            )
            self._reconcile_domain(block.direction_domain)
            self._reconcile_authorities()
            self._assert_invariants()
            return True

    def mark_arrived(self, robot_id: str, block_id: str) -> bool:
        """Finalize a retained grant after its destination is reached."""
        with self._lock:
            key = (robot_id, block_id)
            reservation = self._grants.pop(key, None)
            if reservation is None:
                return False
            block = self.registry.blocks[block_id]
            block.occupants.pop(robot_id, None)
            block.reservations.pop(robot_id, None)
            destination = self.registry.holding_bays[reservation.destination_hb]
            destination.reservations.discard(robot_id)
            destination.occupants.add(robot_id)
            self._robot_states[key] = RobotCorridorState.EXITED
            self._reconcile_domain(block.direction_domain)
            self._assert_invariants()
            return True

    def mark_exited(self, robot_id: str, block_id: str) -> None:
        with self._lock:
            block = self.registry.blocks[block_id]
            block.occupants.pop(robot_id, None)
            block.reservations.pop(robot_id, None)
            reservation = self._grants.pop((robot_id, block_id), None)
            if reservation is not None:
                destination = self.registry.holding_bays[reservation.destination_hb]
                destination.reservations.discard(robot_id)
                destination.occupants.add(robot_id)
            self._robot_states[(robot_id, block_id)] = RobotCorridorState.EXITED
            logger.info(
                "[TRAFFIC] ROBOT_EXITED_BLOCK robot=%s block=%s",
                robot_id, block_id,
            )
            self._reconcile_domain(block.direction_domain)
            if not block.occupants and not block.reservations:
                logger.info("[TRAFFIC] BLOCK_CLEAR block=%s", block_id)
            self._assert_invariants()

    def occupy_holding_bay(self, hb_id: str, robot_id: str) -> None:
        with self._lock:
            bay = self.registry.holding_bays[hb_id]
            if robot_id not in bay.occupants and len(bay.occupants) >= bay.capacity:
                raise ValueError(f"holding bay {hb_id} is full")
            for other in self.registry.holding_bays.values():
                if other is not bay:
                    other.occupants.discard(robot_id)
            bay.occupants.add(robot_id)

    def release_holding_bay(self, hb_id: str, robot_id: str) -> None:
        with self._lock:
            bay = self.registry.holding_bays[hb_id]
            bay.occupants.discard(robot_id)
            bay.reservations.discard(robot_id)
            for domain_id in self._domains:
                self._reconcile_domain(domain_id)

    def decision_for(self, robot_id: str, block_id: str) -> Decision:
        with self._lock:
            block = self.registry.blocks[block_id]
            if block.fault_reason:
                return Decision.BLOCKED
            if (robot_id, block_id) in self._grants or robot_id in block.occupants:
                return Decision.ADMIT
            return Decision.WAIT

    def active_block_for_robot(self, robot_id: str) -> str | None:
        """Return the robot's single granted or occupied block, if unambiguous."""
        with self._lock:
            candidates = {
                block_id
                for candidate_robot, block_id in self._grants
                if candidate_robot == robot_id
                and self._robot_states.get((candidate_robot, block_id))
                is not RobotCorridorState.CLEARED
            }
            candidates.update(
                block_id
                for block_id, block in self.registry.blocks.items()
                if robot_id in block.occupants
            )
            return next(iter(candidates)) if len(candidates) == 1 else None

    def granted_blocks_for_robot(self, robot_id: str) -> tuple[str, ...]:
        """Return active grants in travel order for occupancy classification."""
        with self._lock:
            authority = self._authorities.get(robot_id)
            if authority is not None:
                return authority.unreleased_blocks
            return tuple(
                block_id
                for candidate_robot, block_id in self._grants
                if candidate_robot == robot_id
                and self._robot_states.get((candidate_robot, block_id))
                is not RobotCorridorState.CLEARED
            )

    def has_cleared(self, robot_id: str, block_id: str) -> bool:
        """Return whether a retained grant has cleared its conflict section."""
        with self._lock:
            return (
                self._robot_states.get((robot_id, block_id))
                is RobotCorridorState.CLEARED
            )

    def release_node_for_robot(self, robot_id: str) -> tuple[str, str] | None:
        """Return the configured release point for one retained grant."""
        with self._lock:
            authority = self._authorities.get(robot_id)
            if authority is not None:
                for block_id in authority.unreleased_blocks:
                    reservation = self._grants.get((robot_id, block_id))
                    if reservation is not None and reservation.release_node is not None:
                        return block_id, reservation.release_node
                return None
            matches = [
                (block_id, reservation.release_node)
                for (candidate_robot, block_id), reservation in self._grants.items()
                if candidate_robot == robot_id and reservation.release_node is not None
            ]
            return matches[0] if len(matches) == 1 else None

    def source_holding_bay_for_robot(self, robot_id: str) -> str | None:
        """Return the source bay for the robot's single retained grant."""
        with self._lock:
            authority = self._authorities.get(robot_id)
            if authority is not None:
                return authority.source_slot
            reservations = [
                reservation
                for (candidate_robot, _), reservation in self._grants.items()
                if candidate_robot == robot_id
            ]
            if len(reservations) != 1:
                return None
            return reservations[0].source_hb

    def destination_holding_bay_for_robot(self, robot_id: str) -> str | None:
        """Return the destination bay for the robot's single active grant."""
        with self._lock:
            authority = self._authorities.get(robot_id)
            if authority is not None:
                return authority.destination_slot
            reservations = [
                reservation
                for (candidate_robot, _), reservation in self._grants.items()
                if candidate_robot == robot_id
            ]
            if len(reservations) != 1:
                return None
            return reservations[0].destination_hb

    def cancel(self, robot_id: str, block_id: str | None = None) -> bool:
        with self._lock:
            targets = [block_id] if block_id else list(self.registry.blocks)
            for candidate in targets:
                block = self.registry.blocks[candidate]
                if robot_id in block.occupants:
                    return False
            changed = False
            affected_domains: set[str] = set()
            for candidate in targets:
                block = self.registry.blocks[candidate]
                affected_domains.add(block.direction_domain)
                waiting = self._waiting[candidate].by_direction
                for queue in waiting.values():
                    before = len(queue)
                    queue[:] = [item for item in queue if item.robot_id != robot_id]
                    changed = changed or before != len(queue)
                reservation = self._grants.pop((robot_id, candidate), None)
                block.reservations.pop(robot_id, None)
                if reservation:
                    self.registry.holding_bays[
                        reservation.destination_hb
                    ].reservations.discard(robot_id)
                    changed = True
                self._robot_states.pop((robot_id, candidate), None)
            for domain_id in affected_domains:
                self._reconcile_domain(domain_id)
            return changed

    def fault(self, robot_id: str, *, reason: str) -> None:
        with self._lock:
            authority = self._authorities.get(robot_id)
            if authority is not None:
                logger.error(
                    "[TRAFFIC] ROBOT_FAULT robot=%s reason=%s",
                    robot_id,
                    reason,
                )
                for block_id in authority.unreleased_blocks:
                    block = self.registry.blocks[block_id]
                    block.fault_reason = reason
                    self._robot_states[(robot_id, block_id)] = RobotCorridorState.FAULT
                    logger.error(
                        "[TRAFFIC] BLOCK_FAULT robot=%s block=%s reason=%s",
                        robot_id,
                        block_id,
                        reason,
                    )
                return
            for block_id, block in self.registry.blocks.items():
                if robot_id in block.occupants:
                    block.fault_reason = reason
                    self._robot_states[(robot_id, block_id)] = RobotCorridorState.FAULT
                    logger.error(
                        "[TRAFFIC] ROBOT_FAULT robot=%s reason=%s",
                        robot_id,
                        reason,
                    )
                    logger.error(
                        "[TRAFFIC] BLOCK_FAULT robot=%s block=%s reason=%s",
                        robot_id, block_id, reason,
                    )
                    return
            self.cancel(robot_id)

    def report_unexpected_occupancy(
        self, robot_id: str, block_id: str, *, reason: str
    ) -> None:
        """Fail closed when telemetry places an unreserved robot in a block."""
        with self._lock:
            block = self.registry.blocks[block_id]
            domain = self._domains[block.direction_domain]
            direction = domain.active_direction or Direction.A_TO_B
            block.occupants[robot_id] = direction
            block.fault_reason = reason
            self._robot_states[(robot_id, block_id)] = RobotCorridorState.FAULT
            logger.error(
                "[TRAFFIC] BLOCK_FAULT robot=%s block=%s reason=%s",
                robot_id, block_id, reason,
            )

    def reset(self, *, force: bool = False) -> bool:
        with self._lock:
            if not force and any(block.occupants for block in self.registry.blocks.values()):
                return False
            for block in self.registry.blocks.values():
                block.occupants.clear()
                block.reservations.clear()
                block.fault_reason = None
            for bay in self.registry.holding_bays.values():
                bay.occupants.clear()
                bay.reservations.clear()
            for queues in self._waiting.values():
                for queue in queues.by_direction.values():
                    queue.clear()
            for domain in self._domains.values():
                domain.active_direction = None
                domain.batch_closed = False
            self._grants.clear()
            self._robot_states.clear()
            self._authorities.clear()
            self._pending_authorities.clear()
            return True

    def snapshot(self):
        with self._lock:
            return {
                "enabled": self.registry.enabled,
                "domains": {
                    domain_id: {
                        "active_direction": (
                            state.active_direction.value if state.active_direction else None
                        ),
                        "batch_closed": state.batch_closed,
                    }
                    for domain_id, state in sorted(self._domains.items())
                },
                "blocks": {
                    block_id: {
                        "group_id": block.group_id,
                        "direction_domain": block.direction_domain,
                        "state": block.state.value,
                        "occupants": sorted(block.occupants),
                        "reservations": sorted(block.reservations),
                        "waiting": {
                            direction.value: [
                                item.robot_id
                                for item in self._waiting[block_id].by_direction[direction]
                            ]
                            for direction in Direction
                        },
                        "fault_reason": block.fault_reason,
                    }
                    for block_id, block in sorted(self.registry.blocks.items())
                },
                "holding_bays": {
                    hb_id: {
                        "node_id": bay.node_id,
                        "capacity": bay.capacity,
                        "occupants": sorted(bay.occupants),
                        "reservations": sorted(bay.reservations),
                    }
                    for hb_id, bay in sorted(self.registry.holding_bays.items())
                },
                "authorities": {
                    robot_id: {
                        "authority_id": authority.authority_id,
                        "chain_id": authority.chain_id,
                        "direction": authority.direction.value,
                        "source_slot": authority.source_slot,
                        "destination_slot": authority.destination_slot,
                        "blocks": list(authority.block_ids),
                        "unreleased_blocks": list(authority.unreleased_blocks),
                        "state": "ACTIVE",
                    }
                    for robot_id, authority in sorted(self._authorities.items())
                },
                "pending_authorities": {
                    robot_id: {
                        "authority_id": authority.authority_id,
                        "chain_id": authority.chain_id,
                        "direction": authority.direction.value,
                        "destination_slot": authority.destination_slot,
                        "blocks": list(authority.block_ids),
                        "state": "WAITING",
                    }
                    for robot_id, authority in sorted(
                        self._pending_authorities.items()
                    )
                },
            }

    def _authority_resources_available(
        self,
        authority: MovementAuthority,
    ) -> bool:
        return self.can_reserve_path(
            authority.block_ids,
            authority.direction,
            authority.destination_slot,
            robot_id=authority.robot_id,
            chain_id=authority.chain_id,
        )

    def _grant_authority(self, authority: MovementAuthority) -> None:
        destination = self.registry.holding_bays[authority.destination_slot]
        for index, block_id in enumerate(authority.block_ids):
            block = self.registry.blocks[block_id]
            domain = self._domains[block.direction_domain]
            if not self._domain_busy(block.direction_domain):
                domain.active_direction = authority.direction
                domain.batch_closed = False
            reservation = Reservation(
                robot_id=authority.robot_id,
                block_id=block_id,
                direction=authority.direction,
                destination_hb=authority.destination_slot,
                source_hb=(authority.source_slot if index == 0 else None),
                request_time=authority.request_time,
                release_node=block.release_node(authority.direction),
            )
            block.reservations[authority.robot_id] = reservation
            self._grants[(authority.robot_id, block_id)] = reservation
            self._robot_states[
                (authority.robot_id, block_id)
            ] = RobotCorridorState.RESERVED
            logger.info(
                "[TRAFFIC] BLOCK_RESERVED robot=%s block=%s authority=%s",
                authority.robot_id,
                block_id,
                authority.authority_id,
            )
        destination.reservations.add(authority.robot_id)
        self._authorities[authority.robot_id] = authority
        self._pending_authorities.pop(authority.robot_id, None)
        logger.info(
            "[TRAFFIC] AUTHORITY_ADMIT robot=%s authority=%s chain=%s "
            "blocks=%s destination=%s",
            authority.robot_id,
            authority.authority_id,
            authority.chain_id,
            ",".join(authority.block_ids),
            authority.destination_slot,
        )
        self._assert_invariants()

    def _reconcile_authority_domains(self, authority: MovementAuthority) -> None:
        for domain_id in {
            self.registry.blocks[block_id].direction_domain
            for block_id in authority.block_ids
        }:
            self._reconcile_domain(domain_id)

    def _reconcile_authorities(self) -> None:
        for authority in sorted(
            list(self._pending_authorities.values()),
            key=lambda item: (item.request_time, item.authority_id),
        ):
            if not self._authority_resources_available(authority):
                continue
            self._grant_authority(authority)
        for pending in self._pending_authorities.values():
            for block_id in pending.block_ids:
                block = self.registry.blocks[block_id]
                domain = self._domains[block.direction_domain]
                if (
                    domain.active_direction is not None
                    and domain.active_direction is not pending.direction
                ):
                    domain.batch_closed = True

    def _find_waiting(self, robot_id: str, block_id: str) -> Reservation | None:
        for queue in self._waiting[block_id].by_direction.values():
            for item in queue:
                if item.robot_id == robot_id:
                    return item
        return None

    def _enqueue(self, reservation: Reservation) -> None:
        queue = self._waiting[reservation.block_id].by_direction[reservation.direction]
        if not any(item.robot_id == reservation.robot_id for item in queue):
            queue.append(reservation)

    def _resources_available(self, reservation: Reservation) -> bool:
        block = self.registry.blocks[reservation.block_id]
        if block.fault_reason:
            return False
        if len(block.occupants) + len(block.reservations) >= block.capacity:
            return False
        existing_directions = set(block.occupants.values()) | {
            item.direction for item in block.reservations.values()
        }
        if existing_directions and existing_directions != {reservation.direction}:
            return False
        if block.require_source_hb_unreserved and reservation.source_hb is not None:
            source = self.registry.holding_bays[reservation.source_hb]
            if source.reservations - {reservation.robot_id}:
                return False
        bay = self.registry.holding_bays[reservation.destination_hb]
        used = (bay.occupants | bay.reservations) - {reservation.robot_id}
        return len(used) < bay.capacity

    def _grant(self, reservation: Reservation) -> None:
        block = self.registry.blocks[reservation.block_id]
        bay = self.registry.holding_bays[reservation.destination_hb]
        block.reservations[reservation.robot_id] = reservation
        bay.reservations.add(reservation.robot_id)
        self._grants[(reservation.robot_id, reservation.block_id)] = reservation
        self._robot_states[
            (reservation.robot_id, reservation.block_id)
        ] = RobotCorridorState.RESERVED
        logger.info(
            "[TRAFFIC] ARBITER_ADMIT robot=%s block=%s dir=%s hb=%s",
            reservation.robot_id,
            reservation.block_id,
            reservation.direction.value,
            reservation.destination_hb,
        )
        logger.info(
            "[TRAFFIC] BLOCK_RESERVED robot=%s block=%s",
            reservation.robot_id,
            reservation.block_id,
        )
        logger.info(
            "[TRAFFIC] HB_RESERVED robot=%s hb=%s",
            reservation.robot_id,
            reservation.destination_hb,
        )
        self._assert_invariants()

    def _domain_busy(self, domain_id: str) -> bool:
        return any(
            block.direction_domain == domain_id
            and (block.occupants or block.reservations)
            for block in self.registry.blocks.values()
        )

    def _domain_faulted(self, domain_id: str) -> bool:
        return any(
            block.direction_domain == domain_id and block.fault_reason
            for block in self.registry.blocks.values()
        )

    def _domain_waiting(self, domain_id: str, direction: Direction) -> list[Reservation]:
        waiting: list[Reservation] = []
        for block_id, block in self.registry.blocks.items():
            if block.direction_domain == domain_id:
                waiting.extend(self._waiting[block_id].by_direction[direction])
        return sorted(waiting, key=lambda item: item.request_time)

    def _reconcile_domain(self, domain_id: str) -> None:
        if self._domain_faulted(domain_id):
            return
        domain = self._domains[domain_id]
        waiting = {
            direction: self._domain_waiting(domain_id, direction)
            for direction in Direction
        }
        if not self._domain_busy(domain_id):
            available = [
                direction
                for direction in Direction
                if any(self._resources_available(item) for item in waiting[direction])
            ]
            if not available:
                # Waiting requests may remain resource-blocked by a shared HB.
                # Do not let one of them claim a direction until it is grantable.
                domain.active_direction = None
                return
            preferred = (
                domain.active_direction.opposite
                if domain.batch_closed
                and domain.active_direction is not None
                and domain.active_direction.opposite in available
                else min(available, key=lambda d: waiting[d][0].request_time)
            )
            if preferred is not domain.active_direction:
                logger.info(
                    "[TRAFFIC] BLOCK_DIRECTION_CHANGE domain=%s direction=%s",
                    domain_id, preferred.value,
                )
            domain.active_direction = preferred
            domain.batch_closed = False

        active = domain.active_direction
        if active is None:
            return
        for reservation in list(waiting[active]):
            if domain.batch_closed:
                break
            if self._resources_available(reservation):
                queue = self._waiting[reservation.block_id].by_direction[active]
                queue.remove(reservation)
                self._grant(reservation)
        if self._domain_waiting(domain_id, active.opposite):
            domain.batch_closed = True

    def _assert_invariants(self) -> None:
        for block in self.registry.blocks.values():
            directions = set(block.occupants.values()) | {
                reservation.direction for reservation in block.reservations.values()
            }
            if len(directions) > 1:
                raise AssertionError(
                    f"opposite directions occupy/reserve block {block.block_id}"
                )
        for bay in self.registry.holding_bays.values():
            used = bay.occupants | bay.reservations
            if len(used) > bay.capacity:
                raise AssertionError(f"holding bay {bay.hb_id} exceeds capacity")
