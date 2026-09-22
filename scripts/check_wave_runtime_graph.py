#!/usr/bin/env python3
"""Fail closed if the Arbiter nav-graph snapshot differs from the running WAVE adapter."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic_control.corridor_registry import CorridorRegistry


def _run(*args: str) -> str:
    result = subprocess.run(
        list(args),
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(message or f"command failed: {' '.join(args)}")
    return result.stdout


def _runtime_graph_yaml(container: str) -> str:
    command = (
        'f=$(ls -1t /tmp/WAVE_nav_graph_*.yaml 2>/dev/null | head -1); '
        '[ -n "$f" ] || exit 3; cat "$f"'
    )
    return _run("docker", "exec", container, "bash", "-lc", command)


def _runtime_revision(container: str) -> str | None:
    logs = _run("docker", "logs", container)
    matches = re.findall(
        r"Nav graph read from DB table \[[^\]]+\]:.*?revision=(\d+)",
        logs,
    )
    return matches[-1] if matches else None


def _normalized_graph(raw: dict[str, Any]) -> dict[str, Any]:
    levels = raw.get("levels")
    if not isinstance(levels, dict) or len(levels) != 1:
        raise ValueError("navigation graph must contain exactly one level")
    level_name, level = next(iter(levels.items()))
    if not isinstance(level, dict):
        raise ValueError("navigation graph level is invalid")

    vertices = level.get("vertices") or []
    names: list[str] = []
    nodes: dict[str, tuple[float, float]] = {}
    for vertex in vertices:
        if (
            not isinstance(vertex, list)
            or len(vertex) < 3
            or not isinstance(vertex[2], dict)
            or not vertex[2].get("name")
        ):
            raise ValueError("every navigation vertex must have a name")
        name = str(vertex[2]["name"])
        if name in nodes:
            raise ValueError(f"duplicate navigation node: {name}")
        names.append(name)
        nodes[name] = (float(vertex[0]), float(vertex[1]))

    edges: set[tuple[str, str]] = set()
    for lane in level.get("lanes") or []:
        if not isinstance(lane, list) or len(lane) < 2:
            raise ValueError("invalid navigation lane")
        edges.add((names[int(lane[0])], names[int(lane[1])]))

    return {
        "level": str(level_name),
        "nodes": sorted(
            (name, round(point[0], 9), round(point[1], 9))
            for name, point in nodes.items()
        ),
        "edges": sorted(edges),
    }


def _fingerprint(raw: dict[str, Any]) -> str:
    normalized = _normalized_graph(raw)
    payload = json.dumps(normalized, separators=(",", ":"), sort_keys=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _validate_corridor(raw_graph: dict[str, Any], corridor_path: Path) -> list[str]:
    graph = _normalized_graph(raw_graph)
    node_names = {item[0] for item in graph["nodes"]}
    edges = {tuple(item) for item in graph["edges"]}
    registry = CorridorRegistry.from_yaml(corridor_path)

    errors: list[str] = []
    for hb_id, bay in registry.holding_bays.items():
        if bay.node_id not in node_names:
            errors.append(f"holding_bay_node_missing:{hb_id}:{bay.node_id}")

    for block_id, block in registry.blocks.items():
        for edge in sorted(block.edges_a_to_b | block.edges_b_to_a):
            parts = tuple(edge.split(">", 1))
            if len(parts) != 2 or parts not in edges:
                errors.append(f"corridor_edge_missing:{block_id}:{edge}")

    for route in registry.routes:
        for node in sorted(route.start_nodes | route.goal_nodes):
            if node not in node_names:
                errors.append(f"route_node_missing:{route.route_id}:{node}")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", default="Wave_adapter")
    parser.add_argument("--snapshot", required=True, type=Path)
    parser.add_argument("--corridor", type=Path)
    parser.add_argument("--expected-revision")
    args = parser.parse_args(argv)

    try:
        runtime_text = _runtime_graph_yaml(args.container)
        runtime_raw = yaml.safe_load(runtime_text) or {}
        snapshot_raw = yaml.safe_load(args.snapshot.read_text(encoding="utf-8")) or {}
        runtime_revision = _runtime_revision(args.container)
        runtime_hash = _fingerprint(runtime_raw)
        snapshot_hash = _fingerprint(snapshot_raw)
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print(f"ERROR graph.preflight:{error}", file=sys.stderr)
        return 2

    print(f"container={args.container}")
    print(f"runtime_revision={runtime_revision or 'unknown'}")
    print(f"runtime_sha256={runtime_hash}")
    print(f"snapshot_sha256={snapshot_hash}")

    failed = False
    if runtime_hash != snapshot_hash:
        print("ERROR graph.snapshot_mismatch", file=sys.stderr)
        failed = True

    if (
        args.expected_revision
        and runtime_revision is not None
        and runtime_revision != str(args.expected_revision)
    ):
        print(
            "ERROR graph.revision_mismatch:"
            f"expected={args.expected_revision}:actual={runtime_revision}",
            file=sys.stderr,
        )
        failed = True

    if args.corridor is not None:
        try:
            corridor_errors = _validate_corridor(runtime_raw, args.corridor)
        except (OSError, ValueError, yaml.YAMLError) as error:
            print(f"ERROR corridor.preflight:{error}", file=sys.stderr)
            return 2
        for error in corridor_errors:
            print(f"ERROR {error}", file=sys.stderr)
        failed = failed or bool(corridor_errors)

    if failed:
        return 2

    print("WAVE runtime graph preflight: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
