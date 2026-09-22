"""Planning helpers for connected single-lane corridor chains."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .corridor_registry import CorridorRegistry
from .models import ChainPath, Direction


Availability = Callable[[tuple[str, ...], Direction, str], bool]


@dataclass(frozen=True)
class PlannedAuthority:
    chain_id: str
    direction: Direction
    source_group: str
    source_slot: str
    destination_group: str
    destination_slot: str
    goal_node: str
    block_ids: tuple[str, ...]


class CorridorChainPlanner:
    """Choose the farthest safe stop reachable with one atomic authority."""

    def __init__(self, registry: CorridorRegistry) -> None:
        self.registry = registry

    def plan(
        self,
        path: ChainPath,
        available: Availability,
    ) -> PlannedAuthority | None:
        for destination_index in range(len(path.safe_stop_ids) - 1, 0, -1):
            destination_group = path.safe_stop_ids[destination_index]
            block_ids = path.block_ids[:destination_index]
            for destination_slot in self._candidate_slots(
                path,
                destination_group,
            ):
                if not available(block_ids, path.direction, destination_slot):
                    continue
                return PlannedAuthority(
                    chain_id=path.chain_id,
                    direction=path.direction,
                    source_group=path.source_group,
                    source_slot=path.source_slot,
                    destination_group=destination_group,
                    destination_slot=destination_slot,
                    goal_node=self.registry.holding_bays[destination_slot].node_id,
                    block_ids=block_ids,
                )
        return None

    def _candidate_slots(
        self,
        path: ChainPath,
        destination_group: str,
    ) -> tuple[str, ...]:
        if destination_group == path.destination_group:
            return (path.destination_slot,)
        return self.registry.safe_stop_groups[destination_group].members
