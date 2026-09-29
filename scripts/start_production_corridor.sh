#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${workspace_dir}/.venv/bin/python"
profile="${1:?usage: $0 PRODUCTION_PROFILE}"
runtime_dir="${workspace_dir}/.runtime"
pid_file="${runtime_dir}/production_arbiter.pid"
runtime_fleet_dir="${runtime_dir}/production-field"
runtime_override="${runtime_fleet_dir}/production.compose.override.yml"
health_timeout="${PRODUCTION_HEALTH_TIMEOUT_SECONDS:-300}"
ready_timeout="${PRODUCTION_READY_TIMEOUT_SECONDS:-900}"

if [[ ! "${health_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  echo "PRODUCTION_HEALTH_TIMEOUT_SECONDS must be a positive integer." >&2
  exit 2
fi
if [[ ! "${ready_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  echo "PRODUCTION_READY_TIMEOUT_SECONDS must be a positive integer." >&2
  exit 2
fi

profile="$("${python_bin}" - "${profile}" <<'PY'
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve())
PY
)"
cd "${workspace_dir}"

"${python_bin}" "${workspace_dir}/scripts/validate_production_deployment.py" "${profile}"

mkdir -p "${runtime_dir}"
if [[ -f "${pid_file}" ]]; then
  existing_pid="$(cat "${pid_file}")"
  if [[ "${existing_pid}" =~ ^[0-9]+$ ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    existing_cmdline="$(tr '\0' ' ' <"/proc/${existing_pid}/cmdline" 2>/dev/null || true)"
    if [[ "${existing_cmdline}" == *"traffic_control.task_gate"* && "${existing_cmdline}" == *"${workspace_dir}"* ]]; then
      echo "A production Arbiter from this workspace is already running with pid ${existing_pid}." >&2
      exit 4
    fi
  fi
  rm -f "${pid_file}"
fi

"${python_bin}" "${workspace_dir}/scripts/prepare_production_runtime.py" \
  "${profile}" --output-dir "${runtime_fleet_dir}" >/dev/null

mapfile -t profile_values < <("${python_bin}" - "${profile}" <<'PY'
from pathlib import Path
import sys
from traffic_control.deployment import DeploymentProfile
p = DeploymentProfile.load(Path(sys.argv[1]))
p.require_valid()
print(p.paths.corridor_config)
required = [name for name, robot in p.robots.items() if robot.required]
print(len(required))
for name in required:
    print(name)
for item in p.paths.compose_files:
    print(item)
PY
)
corridor_config="${profile_values[0]}"
expected_robots="${profile_values[1]}"
required_robots=("${profile_values[@]:2:${expected_robots}}")
compose_args=()
compose_start=$((2 + expected_robots))
for compose_file in "${profile_values[@]:${compose_start}}"; do
  compose_args+=(-f "${compose_file}")
done
compose_args+=(-f "${runtime_override}")

resolve_profile_secret() {
  "${python_bin}" - "${profile}" "$1" <<'PY'
from pathlib import Path
import os
import sys
from traffic_control.deployment import DeploymentProfile
p = DeploymentProfile.load(Path(sys.argv[1]))
secret = p.mqtt.username if sys.argv[2] == "username" else p.mqtt.password
if secret is None:
    raise SystemExit("missing MQTT secret")
print(secret.resolve(os.environ), end="")
PY
}
export RMF_FIELD_MQTT_USERNAME="$(resolve_profile_secret username)"
export RMF_FIELD_MQTT_PASSWORD="$(resolve_profile_secret password)"

runtime_started=0
arbiter_started=0
cleanup_failed_start() {
  status=$?
  trap - EXIT
  if [[ "${arbiter_started}" == 1 && -f "${pid_file}" ]]; then
    pid="$(cat "${pid_file}")"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
      if [[ "${cmdline}" == *"traffic_control.task_gate"* && "${cmdline}" == *"${workspace_dir}"* && "${cmdline}" == *"${profile}"* ]]; then
        kill "${pid}" || true
      fi
    fi
    rm -f "${pid_file}"
  fi
  if [[ "${runtime_started}" == 1 ]]; then
    docker compose "${compose_args[@]}" stop \
      vda5050_fleet_adapter rmf_api_server rmf_traffic_blockade \
      rmf_task_dispatcher rmf_traffic_schedule >/dev/null 2>&1 || true
  fi
  exit "${status}"
}
trap cleanup_failed_start EXIT

runtime_started=1
launch_since="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
docker compose "${compose_args[@]}" up -d \
  rmf_traffic_schedule rmf_task_dispatcher rmf_traffic_blockade rmf_api_server \
  vda5050_fleet_adapter

registered=0
for _ in $(seq 1 "${health_timeout}"); do
  adapter_id="$(docker compose "${compose_args[@]}" ps -q vda5050_fleet_adapter)"
  if [[ -n "${adapter_id}" ]] && [[ "$(docker inspect -f '{{.State.Running}}' "${adapter_id}" 2>/dev/null || true)" == "true" ]]; then
    adapter_logs="$(docker logs --since "${launch_since}" "${adapter_id}" 2>&1 || true)"
    registered=0
    for robot_id in "${required_robots[@]}"; do
      if grep -Fq "Successfully added robot [${robot_id}]" <<<"${adapter_logs}"; then
        registered=$((registered + 1))
      fi
    done
    if [[ "${registered}" -ge "${expected_robots}" ]]; then
      break
    fi
  fi
  sleep 1
done
if [[ "${registered}" -lt "${expected_robots}" ]]; then
  echo "Fleet Adapter did not register all required robots (${registered}/${expected_robots})." >&2
  exit 3
fi

nohup env PYTHON_BIN="${python_bin}" \
  "${workspace_dir}/scripts/run_direction_arbiter.sh" \
  "${corridor_config}" --deployment-profile "${profile}" \
  >"${runtime_dir}/production_arbiter.log" 2>&1 &
echo "$!" >"${pid_file}"
arbiter_started=1

for _ in $(seq 1 "${health_timeout}"); do
  if curl -fsS --noproxy '*' http://127.0.0.1:8200/health >/dev/null; then
    break
  fi
  sleep 1
done
curl -fsS --noproxy '*' http://127.0.0.1:8200/health >/dev/null

for _ in $(seq 1 "${ready_timeout}"); do
  if curl -fsS --noproxy '*' http://127.0.0.1:8200/ready >/dev/null; then
    echo "Production corridor runtime is ready."
    trap - EXIT
    exit 0
  fi
  sleep 1
done
echo "Runtime is healthy but not ready; inspect /ready and /traffic/status." >&2
exit 3
