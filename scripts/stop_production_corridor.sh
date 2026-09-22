#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${workspace_dir}/.venv/bin/python"
profile="${1:?usage: $0 PRODUCTION_PROFILE}"
pid_file="${workspace_dir}/.runtime/production_arbiter.pid"

profile="$("${python_bin}" - "${profile}" <<'PY'
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve())
PY
)"
cd "${workspace_dir}"

if [[ -f "${pid_file}" ]]; then
  pid="$(cat "${pid_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
    if [[ "${cmdline}" == *"traffic_control.task_gate"* && "${cmdline}" == *"${workspace_dir}"* && "${cmdline}" == *"${profile}"* ]]; then
      kill "${pid}"
    else
      echo "Refusing to stop unrelated pid ${pid}; removing stale pid file." >&2
    fi
  fi
  rm -f "${pid_file}"
fi

mapfile -t compose_files < <("${python_bin}" - "${profile}" <<'PY'
from pathlib import Path
import sys
from traffic_control.deployment import DeploymentProfile
p = DeploymentProfile.load(Path(sys.argv[1]))
for item in p.paths.compose_files:
    print(item)
PY
)
compose_args=()
for compose_file in "${compose_files[@]}"; do
  compose_args+=(-f "${compose_file}")
done
docker compose "${compose_args[@]}" stop \
  vda5050_fleet_adapter rmf_api_server rmf_traffic_blockade \
  rmf_task_dispatcher rmf_traffic_schedule
