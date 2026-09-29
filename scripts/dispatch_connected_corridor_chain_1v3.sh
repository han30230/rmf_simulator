#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 CHAIN_R4
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 CHAIN_L4
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 CHAIN_L3
"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B3 CHAIN_L2
