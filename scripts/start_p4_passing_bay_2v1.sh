#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PASSING_BAY_DISPATCH_SCRIPT=t4_dispatch_passing_bay_2v1.sh \
  exec "${script_dir}/start_p4_passing_bay.sh" AGV_A1 AGV_B1 AGV_B2
