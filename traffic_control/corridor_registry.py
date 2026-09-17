"""Configuration loader for corridor blocks, holding bays, and route intents."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .models import CorridorBlock, Direction, HoldingBay, RouteIntent, RouteStep


class CorridorRegistry:
    def __init__(self) -> None:
        self.enabled = True
        self.holding_bays: dict[str, HoldingBay] = {}
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
        raw_blocks = list(config.get("blocks") or settings.get("blocks") or [])
        if not raw_blocks:
            for group_id, group in (settings.get("groups") or {}).items():
                for item in (group or {}).get("blocks") or []:
                    raw = dict(item)
                    raw.setdefault("group_id", str(group_id))
                    raw_blocks.append(raw)
        raw_routes = config.get("routes") or settings.get("routes") or []

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

        for raw in raw_blocks:
            block_id = str(raw["id"])
            if block_id in registry.blocks:
                raise ValueError(f"duplicate block id: {block_id}")
            entry_a = str(raw["entry_a"])
            entry_b = str(raw["entry_b"])
            for hb_id in (entry_a, entry_b):
                if hb_id not in registry.holding_bays:
                    raise ValueError(f"block {block_id} references unknown holding bay {hb_id}")
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
                geometry=raw.get("geometry"),
                edges_a_to_b={str(item) for item in raw.get("edges_a_to_b", [])},
                edges_b_to_a={str(item) for item in raw.get("edges_b_to_a", [])},
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
                )
            )
        return registry

    def resolve_route(self, start_node: str, goal_node: str) -> RouteIntent | None:
        for route in self.routes:
            if start_node in route.start_nodes and goal_node in route.goal_nodes:
                return route
        return None

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
