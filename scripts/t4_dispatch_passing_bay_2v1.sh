#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
delay_seconds="${PASSING_BAY_DISPATCH_DELAY_SECONDS:-1}"

echo "A1: 2101 -> 2108 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 2108

sleep "${delay_seconds}"

echo "B1: 2108 -> 2101 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 2101

sleep "${delay_seconds}"

echo "B2: 2108 -> 2101 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 2101
