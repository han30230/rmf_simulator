#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "P4 A: 2101 -> 2108"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_P4_A 2108
echo "P4 B: 2108 -> 2101"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_P4_B 2101
echo "P5 B: 3108 -> 3101"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_P5_B 3101
echo "P5 A: 3101 -> 3108"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_P5_A 3108
