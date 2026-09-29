#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
delay_seconds="${PASSING_BAY_DISPATCH_DELAY_SECONDS:-1}"

echo "A1: P4_LS1 -> P4_RS1 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 P4_RS1
sleep "${delay_seconds}"

echo "A2: P4_LS2 -> P4_RS3 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A2 P4_RS3
sleep "${delay_seconds}"

echo "B1: P4_RS1 -> P4_LS1 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 P4_LS1
sleep "${delay_seconds}"

echo "B2: P4_RS2 -> P4_LS2 요청"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 P4_LS2
