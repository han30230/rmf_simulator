"""Adapter-side path admission backed by the shared DirectionArbiter."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Any, Sequence

from .corridor_registry import CorridorRegistry
from .direction_arbiter import DirectionArbiter
from .models import Decision, Direction

logger = logging.getLogger(__name__)


class PathIntentError(ValueError):
    """Raised when one navigation path is ambiguous for managed corridors."""


@dataclass(frozen=True)
class PathIntent:
    path: tuple[str, ...]
    block_ids: tuple[str, ...]
    direction: Direction
    managed_edges: tuple[str, ...]
    source_hb: str | None
    destination_hb: str | None
    goal_node: str


class PathIntentResolver:
    """Resolve a concrete Adapter path into one managed movement intent."""

    def __init__(self, registry: CorridorRegistry) -> None:
        self.registry = registry
        edge_index: dict[str, list[tuple[str, Direction]]] = {}
        for block_id, block in registry.blocks.items():
            for edge in block.edges_a_to_b:
                edge_index.setdefault(edge, []).append(
                    (block_id, Direction.A_TO_B)
                )
            for edge in block.edges_b_to_a:
                edge_index.setdefault(edge, []).append(
                    (block_id, Direction.B_TO_A)
                )
        self._edge_index = edge_index

    def resolve(self, path: Sequence[str]) -> PathIntent | None:
        nodes = tuple(str(item) for item in path)
        if len(nodes) < 2:
            return None

        hits: list[tuple[str, Direction, str]] = []
        for start, end in zip(nodes, nodes[1:]):
            edge = f"{start}>{end}"
            matches = self._edge_index.get(edge, [])
            if not matches:
                continue
            unique = list(dict.fromkeys(matches))
            if len(unique) != 1:
                raise PathIntentError(
                    f"managed edge {edge} belongs to multiple blocks"
                )
            block_id, direction = unique[0]
            hits.append((block_id, direction, edge))

        if not hits:
            return None

        directions = {direction for _, direction, _ in hits}
        if len(directions) != 1:
            raise PathIntentError(
                "one Adapter order crosses managed corridors in mixed directions"
            )
        direction = next(iter(directions))

        block_ids: list[str] = []
        managed_edges: list[str] = []
        seen_blocks: set[str] = set()
        for block_id, _direction, edge in hits:
            managed_edges.append(edge)
            if block_id in seen_blocks:
                if block_ids[-1] != block_id:
                    raise PathIntentError(
                        f"path leaves and re-enters managed block {block_id}"
                    )
                continue
            block_ids.append(block_id)
            seen_blocks.add(block_id)

        return PathIntent(
            path=nodes,
            block_ids=tuple(block_ids),
            direction=direction,
            managed_edges=tuple(managed_edges),
            source_hb=self.registry.holding_bay_for_node(nodes[0]),
            destination_hb=self.registry.holding_bay_for_node(nodes[-1]),
            goal_node=nodes[-1],
        )


class AdapterAdmissionController:
    """Own the HTTP-facing Adapter admission contract.

    The DirectionArbiter remains the single source of truth. Repeated HSM send
    attempts use the same movement_key, so retries poll one existing authority
    instead of creating duplicate reservations.
    """

    def __init__(
        self,
        registry: CorridorRegistry,
        arbiter: DirectionArbiter,
        *,
        readiness: Any | None = None,
    ) -> None:
        self.registry = registry
        self.arbiter = arbiter
        self.readiness = readiness
        self.resolver = PathIntentResolver(registry)

    def admit(
        self,
        *,
        robot_id: str,
        movement_key: str,
        path: Sequence[str],
    ) -> dict[str, Any]:
        robot_id = str(robot_id).strip()
        movement_key = str(movement_key).strip()
        if not robot_id:
            raise ValueError("robot_id must not be empty")
        if not movement_key:
            raise ValueError("movement_key must not be empty")
        if isinstance(path, (str, bytes)) or not isinstance(path, Sequence):
            raise TypeError("path must be a sequence of node ids")

        if self.readiness is not None:
            can_accept = getattr(self.readiness, "can_accept_tasks", None)
            if callable(can_accept) and not can_accept():
                return {
                    "decision": Decision.BLOCKED.value,
                    "managed": True,
                    "reason": "runtime_not_ready",
                }

        try:
            intent = self.resolver.resolve(path)
        except PathIntentError as error:
            logger.error(
                "[TRAFFIC] ADAPTER_PATH_BLOCKED robot=%s request=%s error=%s",
                robot_id,
                movement_key,
                error,
            )
            return {
                "decision": Decision.BLOCKED.value,
                "managed": True,
                "reason": "path_intent_invalid",
                "error": str(error),
            }

        if intent is None:
            return {
                "decision": "BYPASS",
                "managed": False,
                "movement_key": movement_key,
            }

        decision = self.arbiter.request_path_authority(
            robot_id=robot_id,
            request_key=movement_key,
            block_ids=intent.block_ids,
            direction=intent.direction,
            source_hb=intent.source_hb,
            destination_hb=intent.destination_hb,
            goal_node=intent.goal_node,
        )
        authority = self.arbiter.authority_for_robot(robot_id)
        return {
            "decision": decision.value,
            "managed": True,
            "movement_key": movement_key,
            "direction": intent.direction.value,
            "blocks": list(intent.block_ids),
            "managed_edges": list(intent.managed_edges),
            "source_hb": intent.source_hb,
            "destination_hb": intent.destination_hb,
            "authority_id": (
                authority.authority_id if authority is not None else None
            ),
        }

    def cancel(self, *, robot_id: str, movement_key: str) -> dict[str, Any]:
        robot_id = str(robot_id).strip()
        movement_key = str(movement_key).strip()
        if not robot_id or not movement_key:
            raise ValueError("robot_id and movement_key must not be empty")
        cancelled = self.arbiter.cancel_path_authority(
            robot_id, movement_key
        )
        return {
            "cancelled": cancelled,
            "robot_id": robot_id,
            "movement_key": movement_key,
        }
