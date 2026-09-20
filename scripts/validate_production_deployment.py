#!/usr/bin/env python3
"""Validate a field deployment profile without starting motion services."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic_control.deployment import DeploymentConfigError, DeploymentProfile


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile")
    args = parser.parse_args(argv)
    try:
        profile = DeploymentProfile.load(Path(args.profile))
        errors = profile.validate()
    except DeploymentConfigError as error:
        print(f"ERROR {error}", file=sys.stderr)
        return 2
    if errors:
        for code in errors:
            print(f"ERROR {code}", file=sys.stderr)
        return 2
    print("production preflight: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
