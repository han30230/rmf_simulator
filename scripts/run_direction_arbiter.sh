#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_path="${1:-${workspace_dir}/config/corridor_blocks.yaml}"
python_bin="${PYTHON_BIN:-python3}"

cd "${workspace_dir}"
exec "${python_bin}" -m traffic_control.task_gate \
  --config "${config_path}" \
  --listen-host 127.0.0.1 \
  --port 8200 \
  --upstream http://127.0.0.1:8100/tasks/robot_task \
  --mqtt-host 127.0.0.1 \
  --mqtt-port 1883 \
  --mqtt-topic 'uagv/v2.0.0/inatech/+/state'
