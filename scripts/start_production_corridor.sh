#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${workspace_dir}/.venv/bin/python"
profile="${1:?usage: $0 PRODUCTION_PROFILE}"
runtime_dir="${workspace_dir}/.runtime"

"${python_bin}" "${workspace_dir}/scripts/validate_production_deployment.py" "${profile}"
: "${RMF_API_BEARER_TOKEN:?RMF_API_BEARER_TOKEN must be supplied externally}"

mapfile -t profile_values < <("${python_bin}" - "${profile}" <<'PY'
from pathlib import Path
import sys
from traffic_control.deployment import DeploymentProfile
p = DeploymentProfile.load(Path(sys.argv[1]))
p.require_valid()
print(p.paths.corridor_config)
for item in p.paths.compose_files:
    print(item)
PY
)
corridor_config="${profile_values[0]}"
compose_args=()
for compose_file in "${profile_values[@]:1}"; do
  compose_args+=(-f "${compose_file}")
done

mkdir -p "${runtime_dir}"
cd "${workspace_dir}"
docker compose "${compose_args[@]}" up -d \
  rmf_traffic_schedule rmf_task_dispatcher rmf_traffic_blockade rmf_api_server \
  vda5050_fleet_adapter

nohup env PYTHON_BIN="${python_bin}" RMF_API_BEARER_TOKEN="${RMF_API_BEARER_TOKEN}" \
  "${workspace_dir}/scripts/run_direction_arbiter.sh" \
  "${corridor_config}" --deployment-profile "${profile}" \
  >"${runtime_dir}/production_arbiter.log" 2>&1 &
echo "$!" >"${runtime_dir}/production_arbiter.pid"

for _ in $(seq 1 120); do
  if curl -fsS --noproxy '*' http://127.0.0.1:8200/health >/dev/null; then
    break
  fi
  sleep 1
done
curl -fsS --noproxy '*' http://127.0.0.1:8200/health >/dev/null

for _ in $(seq 1 120); do
  if curl -fsS --noproxy '*' http://127.0.0.1:8200/ready >/dev/null; then
    echo "Production corridor runtime is ready."
    exit 0
  fi
  sleep 1
done
echo "Runtime is healthy but not ready; inspect /ready and /traffic/status." >&2
exit 3
