#!/usr/bin/env python3
"""Capture the navigation graph currently loaded by a running WAVE adapter."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import yaml


def _run(*args: str) -> str:
    result = subprocess.run(
        list(args),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(detail or f"command failed: {' '.join(args)}")
    return result.stdout


def _revision(container: str) -> str | None:
    logs = _run("docker", "logs", container)
    matches = re.findall(
        r"Nav graph read from DB table \[[^\]]+\]:.*?revision=(\d+)",
        logs,
    )
    return matches[-1] if matches else None


def _graph(container: str) -> str:
    command = (
        'f=$(ls -1t /tmp/WAVE_nav_graph_*.yaml 2>/dev/null | head -1); '
        '[ -n "$f" ] || exit 3; cat "$f"'
    )
    return _run("docker", "exec", container, "bash", "-lc", command)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", default="Wave_adapter")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--expected-revision")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)

    try:
        revision = _revision(args.container)
        raw_text = _graph(args.container)
        raw = yaml.safe_load(raw_text)
        if not isinstance(raw, dict) or not isinstance(raw.get("levels"), dict):
            raise ValueError("runtime graph is not a valid navigation graph")
    except (RuntimeError, ValueError, yaml.YAMLError) as error:
        print(f"ERROR capture.failed:{error}", file=sys.stderr)
        return 2

    if args.expected_revision and revision != str(args.expected_revision):
        print(
            "ERROR graph.revision_mismatch:"
            f"expected={args.expected_revision}:actual={revision or 'unknown'}",
            file=sys.stderr,
        )
        return 2

    output = args.output.expanduser().resolve()
    if output.exists() and not args.force:
        print(f"ERROR output.exists:{output}", file=sys.stderr)
        return 2
    output.parent.mkdir(parents=True, exist_ok=True)

    # Preserve the adapter's exact runtime YAML bytes; write atomically.
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=str(output.parent),
        prefix=f".{output.name}.",
        delete=False,
    ) as stream:
        stream.write(raw_text)
        temp_path = Path(stream.name)
    temp_path.replace(output)

    print(f"container={args.container}")
    print(f"runtime_revision={revision or 'unknown'}")
    print(f"captured={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
