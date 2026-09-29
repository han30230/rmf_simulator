#!/usr/bin/env python3
"""Build runtime-only Fleet Adapter and Compose files from a field profile."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic_control.deployment import DeploymentConfigError, DeploymentProfile
from traffic_control.production_runtime import prepare_production_runtime


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    try:
        profile = DeploymentProfile.load(args.profile)
        prepared = prepare_production_runtime(profile, args.output_dir)
    except DeploymentConfigError as exc:
        print(f"production runtime preparation failed: {exc}", file=sys.stderr)
        return 2
    print(prepared.fleet_config)
    print(prepared.compose_override)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
