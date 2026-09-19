#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"

exec "${workspace_dir}/.venv/bin/python" \
  "${workspace_dir}/scripts/run_dynamic_corridor_scenario.py" \
  "${workspace_dir}/config/dynamic_connected_corridor_chain_2v2.yaml" \
  "$@"
