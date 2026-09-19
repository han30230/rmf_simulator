"""Configuration loader for corridor blocks, holding bays, and route intents."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .models import (
    ChainPath,
    CorridorBlock,
    CorridorChain,
    Direction,
    HoldingBay,
    RouteIntent,
    RouteStep,
    SafeStopGroup,
)


class CorridorRegistry:
    def __init__(self) -> None:
        self.enabled = True
        self.holding_bays: dict[str, HoldingBay] = {}
        self.safe_stop_groups: dict[str, SafeStopGroup] = {}
        self.corridor_chains: dict[str, CorridorChain] = {}
        self.blocks: dict[str, CorridorBlock] = {}
        self.routes: list[RouteIntent] = []
        self.settings: dict[str, Any] = {}

    @classmethod
    def from_yaml(cls, path: str | Path) -> "CorridorRegistry":
        with Path(path).open("r", encoding="utf-8") as stream:
            return cls.from_dict(yaml.safe_load(stream) or {})

    @classmethod
    def from_dict(cls, config: dict[str, Any]) -> "CorridorRegistry":
        registry = cls()
        settings = dict(config.get("traffic_control") or {})
        registry.settings = settings
        registry.enabled = bool(settings.get("enabled", True))

        holding_bays = config.get("holding_bays") or settings.get("holding_bays") or {}
        safe_stop_groups = (
            config.get("safe_stop_groups")
            or settings.get("safe_stop_groups")
            or {}
        )
        raw_blocks = list(config.get("blocks") or settings.get("blocks") or [])
        if not raw_blocks:
            for group_id, group in (settings.get("groups") or {}).items():
                for item in (group or {}).get("blocks") or []:
                    raw = dict(item)
                    raw.setdefault("group_id", str(group_id))
                    raw_blocks.append(raw)
        raw_routes = config.get("routes") or settings.get("routes") or []
        raw_chains = list(
            config.get("corridor_chains")
            or settings.get("corridor_chains")
            or []
        )

        for hb_id, raw in holding_bays.items():
            capacity = int(raw.get("capacity", 1))
            if capacity < 1:
                raise ValueError(f"holding bay {hb_id} capacity must be positive")
            registry.holding_bays[hb_id] = HoldingBay(
                hb_id=hb_id,
                node_id=str(raw["node_id"]),
                capacity=capacity,
                geometry=raw.get("geometry"),
            )

        member_groups: dict[str, str] = {}
        for group_id, raw in safe_stop_groups.items():
            members = tuple(str(item) for item in raw.get("members") or [])
            if not members:
                raise ValueError(f"safe-stop group {group_id} must have members")
            if len(set(members)) != len(members):
                raise ValueError(f"safe-stop group {group_id} has duplicate members")
            for hb_id in members:
                if hb_id not in registry.holding_bays:
                    raise ValueError(
                        f"safe-stop group {group_id} references unknown holding bay {hb_id}"
                    )
                previous = member_groups.get(hb_id)
                if previous is not None:
                    raise ValueError(
                        f"holding bay {hb_id} belongs to more than one safe-stop group: "
                        f"{previous}, {group_id}"
                    )
                member_groups[hb_id] = str(group_id)
            registry.safe_stop_groups[str(group_id)] = SafeStopGroup(
                group_id=str(group_id),
                members=members,
            )

        for raw in raw_blocks:
            block_id = str(raw["id"])
            if block_id in registry.blocks:
                raise ValueError(f"duplicate block id: {block_id}")
            entry_a = str(raw["entry_a"])
            entry_b = str(raw["entry_b"])
            for endpoint in (entry_a, entry_b):
                if (
                    endpoint not in registry.holding_bays
                    and endpoint not in registry.safe_stop_groups
                ):
                    raise ValueError(
                        f"block {block_id} references unknown holding bay or "
                        f"safe-stop group {endpoint}"
                    )
            capacity = int(raw.get("capacity", 1))
            if capacity < 1:
                raise ValueError(f"block {block_id} capacity must be positive")
            group_id = str(raw.get("group_id", block_id))
            registry.blocks[block_id] = CorridorBlock(
                block_id=block_id,
                group_id=group_id,
                direction_domain=str(raw.get("direction_domain", block_id)),
                entry_a=entry_a,
                entry_b=entry_b,
                capacity=capacity,
                require_source_hb_unreserved=bool(
                    raw.get("require_source_hb_unreserved", False)
                ),
                geometry=raw.get("geometry"),
                edges_a_to_b={str(item) for item in raw.get("edges_a_to_b", [])},
                edges_b_to_a={str(item) for item in raw.get("edges_b_to_a", [])},
                release_node_a_to_b=(
                    str(raw["release_node_a_to_b"])
                    if raw.get("release_node_a_to_b") is not None
                    else None
                ),
                release_node_b_to_a=(
                    str(raw["release_node_b_to_a"])
                    if raw.get("release_node_b_to_a") is not None
                    else None
                ),
            )

        used_chain_blocks: set[str] = set()
        used_chain_stops: set[str] = set()
        for index, raw in enumerate(raw_chains):
            chain_id = str(raw.get("id", f"chain_{index}"))
            if chain_id in registry.corridor_chains:
                raise ValueError(f"duplicate corridor chain id: {chain_id}")
            block_ids = tuple(str(item) for item in raw.get("blocks") or [])
            safe_stop_ids = tuple(
                str(item) for item in raw.get("safe_stops") or []
            )
            if not block_ids:
                raise ValueError(f"corridor chain {chain_id} must have blocks")
            if len(safe_stop_ids) != len(block_ids) + 1:
                raise ValueError(
                    f"corridor chain {chain_id} must have one more safe stop than blocks"
                )
            max_active_robots = int(raw.get("max_active_robots", 1))
            if max_active_robots < 1:
                raise ValueError(
                    f"corridor chain {chain_id} max_active_robots must be positive"
                )
            for stop_id in safe_stop_ids:
                if stop_id not in registry.safe_stop_groups:
                    raise ValueError(
                        f"corridor chain {chain_id} references unknown safe-stop "
                        f"group {stop_id}"
                    )
                if stop_id in used_chain_stops:
                    raise ValueError(
                        f"safe-stop group {stop_id} belongs to more than one corridor chain"
                    )
                used_chain_stops.add(stop_id)
            for position, block_id in enumerate(block_ids):
                if block_id not in registry.blocks:
                    raise ValueError(
                        f"corridor chain {chain_id} references unknown block {block_id}"
                    )
                if block_id in used_chain_blocks:
                    raise ValueError(
                        f"block {block_id} belongs to more than one corridor chain"
                    )
                used_chain_blocks.add(block_id)
                block = registry.blocks[block_id]
                expected = (safe_stop_ids[position], safe_stop_ids[position + 1])
                if (block.entry_a, block.entry_b) != expected:
                    raise ValueError(
                        f"corridor chain {chain_id} block {block_id} endpoints "
                        f"{block.entry_a}->{block.entry_b} do not match "
                        f"{expected[0]}->{expected[1]}"
                    )
            registry.corridor_chains[chain_id] = CorridorChain(
                chain_id=chain_id,
                block_ids=block_ids,
                safe_stop_ids=safe_stop_ids,
                max_active_robots=max_active_robots,
            )

        for index, raw in enumerate(raw_routes):
            steps: list[RouteStep] = []
            for step in raw.get("steps") or []:
                block_id = str(step["block_id"])
                destination_hb = str(step["destination_hb"])
                source_hb = step.get("source_hb")
                if block_id not in registry.blocks:
                    raise ValueError(f"route references unknown block {block_id}")
                if destination_hb not in registry.holding_bays:
                    raise ValueError(
                        f"route references unknown holding bay {destination_hb}"
                    )
                if source_hb is not None and str(source_hb) not in registry.holding_bays:
                    raise ValueError(f"route references unknown holding bay {source_hb}")
                direction = Direction(str(step["direction"]))
                block = registry.blocks[block_id]
                expected_source, expected_destination = (
                    (block.entry_a, block.entry_b)
                    if direction is Direction.A_TO_B
                    else (block.entry_b, block.entry_a)
                )
                normalized_source = (
                    expected_source if source_hb is None else str(source_hb)
                )
                if (
                    normalized_source != expected_source
                    or destination_hb != expected_destination
                ):
                    raise ValueError(
                        f"route step {block_id} {direction.value} does not match "
                        f"block endpoints {expected_source}->{expected_destination}"
                    )
                goal_node = str(step["goal_node"])
                if registry.holding_bays[destination_hb].node_id != goal_node:
                    raise ValueError(
                        f"route step {block_id} goal {goal_node} does not match "
                        f"destination holding bay {destination_hb}"
                    )
                release_node = step.get("release_node")
                normalized_release = (
                    None if release_node is None else str(release_node).strip()
                )
                if normalized_release == "":
                    raise ValueError(
                        f"route step {block_id} release node must not be empty"
                    )
                if normalized_release is not None:
                    direction_edges = (
                        block.edges_a_to_b
                        if direction is Direction.A_TO_B
                        else block.edges_b_to_a
                    )
                    direction_nodes = {
                        node
                        for edge in direction_edges
                        for node in edge.split(">", maxsplit=1)
                    }
                    if (
                        normalized_release == goal_node
                        or normalized_release not in direction_nodes
                    ):
                        raise ValueError(
                            f"route step {block_id} release node "
                            f"{normalized_release} must precede goal {goal_node} "
                            f"on {direction.value} edges"
                        )
                steps.append(
                    RouteStep(
                        block_id=block_id,
                        direction=direction,
                        destination_hb=destination_hb,
                        goal_node=goal_node,
                        source_hb=normalized_source,
                        release_node=normalized_release,
                        requires_opposite_routes_cleared=bool(
                            step.get("requires_opposite_routes_cleared", False)
                        ),
                    )
                )
            if not steps:
                raise ValueError("each managed route must contain at least one step")
            registry.routes.append(
                RouteIntent(
                    route_id=str(raw.get("id", f"route_{index}")),
                    start_nodes=frozenset(str(item) for item in raw["start_nodes"]),
                    goal_nodes=frozenset(str(item) for item in raw["goal_nodes"]),
                    steps=tuple(steps),
                    requires_no_opposite_jobs=bool(
                        raw.get("requires_no_opposite_jobs", False)
                    ),
                )
            )
        return registry

    def resolve_routes(self, start_node: str, goal_node: str) -> list[RouteIntent]:
        return [
            route
            for route in self.routes
            if start_node in route.start_nodes and goal_node in route.goal_nodes
        ]

    def resolve_route(self, start_node: str, goal_node: str) -> RouteIntent | None:
        routes = self.resolve_routes(start_node, goal_node)
        return routes[0] if routes else None

    def resolve_chain_path(self, start_node: str, goal_node: str) -> ChainPath | None:
        source_slot = self.holding_bay_for_node(str(start_node))
        destination_slot = self.holding_bay_for_node(str(goal_node))
        if source_slot is None or destination_slot is None:
            return None

        source_group = self.safe_stop_group_for_holding_bay(source_slot)
        destination_group = self.safe_stop_group_for_holding_bay(destination_slot)
        if (
            source_group is None
            or destination_group is None
            or source_group == destination_group
        ):
            return None

        for chain in self.corridor_chains.values():
            if (
                source_group not in chain.safe_stop_ids
                or destination_group not in chain.safe_stop_ids
            ):
                continue
            source_index = chain.safe_stop_ids.index(source_group)
            destination_index = chain.safe_stop_ids.index(destination_group)
            low, high = sorted((source_index, destination_index))
            block_ids = chain.block_ids[low:high]
            safe_stop_ids = chain.safe_stop_ids[low : high + 1]
            direction = (
                Direction.A_TO_B
                if source_index < destination_index
                else Direction.B_TO_A
            )
            if direction is Direction.B_TO_A:
                block_ids = tuple(reversed(block_ids))
                safe_stop_ids = tuple(reversed(safe_stop_ids))
            return ChainPath(
                chain_id=chain.chain_id,
                direction=direction,
                block_ids=tuple(block_ids),
                safe_stop_ids=tuple(safe_stop_ids),
                source_group=source_group,
                destination_group=destination_group,
                source_slot=source_slot,
                destination_slot=destination_slot,
            )
        return None

    def safe_stop_group_for_holding_bay(self, hb_id: str) -> str | None:
        matches = [
            group_id
            for group_id, group in self.safe_stop_groups.items()
            if hb_id in group.members
        ]
        return matches[0] if len(matches) == 1 else None

    def block_for_position(self, x: float, y: float) -> str | None:
        matches = self.blocks_for_position(x, y)
        return matches[0] if matches else None

    def blocks_for_position(self, x: float, y: float) -> list[str]:
        """Return every managed block whose geometry contains a position."""
        return [
            block_id
            for block_id, block in self.blocks.items()
            if _contains(block.geometry, x, y)
        ]

    def holding_bay_for_position(self, x: float, y: float) -> str | None:
        for hb_id, bay in self.holding_bays.items():
            if _contains(bay.geometry, x, y):
                return hb_id
        return None

    def holding_bay_for_node(self, node_id: str) -> str | None:
        for hb_id, bay in self.holding_bays.items():
            if bay.node_id == node_id:
                return hb_id
        return None

    def block_for_edges(self, edge_states: list[dict[str, Any]]) -> str | None:
        edge_ids = {
            str(item.get("edgeId"))
            for item in edge_states
            if isinstance(item, dict) and item.get("edgeId") is not None
        }
        matches = [
            block_id
            for block_id, block in self.blocks.items()
            if edge_ids & (block.edges_a_to_b | block.edges_b_to_a)
        ]
        return matches[0] if len(matches) == 1 else None


def _contains(geometry: dict[str, Any] | None, x: float, y: float) -> bool:
    if not geometry:
        return False
    if "bounds" in geometry:
        bounds = geometry["bounds"]
        return (
            float(bounds["min_x"]) <= x <= float(bounds["max_x"])
            and float(bounds["min_y"]) <= y <= float(bounds["max_y"])
        )
    if "circle" in geometry:
        circle = geometry["circle"]
        dx = x - float(circle["x"])
        dy = y - float(circle["y"])
        return dx * dx + dy * dy <= float(circle["radius"]) ** 2
    raise ValueError(f"unsupported geometry: {geometry}")
