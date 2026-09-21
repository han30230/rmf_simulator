#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
platform_dir="${workspace_dir}/rmf_platform-main"
simulator_dir="${workspace_dir}/rmf_dev_tool-main/vda5050_robot_simulator"
python_bin="${workspace_dir}/.venv/bin/python"
runtime_dir="${workspace_dir}/.runtime"
simulator_log="${runtime_dir}/simulator.log"
arbiter_log="${runtime_dir}/arbiter.log"
base_compose="${platform_dir}/docker-compose.yml"
portable_compose="${platform_dir}/docker-compose.portable.yml"

resolve_workspace_path() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s/%s\n' "${workspace_dir}" "$1" ;;
  esac
}

runtime_name="${PASSING_BAY_RUNTIME_NAME:-}"
if [[ -n "${runtime_name}" ]]; then
  simulator_config="${runtime_dir}/${runtime_name}_runtime.yaml"
else
  simulator_config="${runtime_dir}/p4_passing_bay_runtime.yaml"
fi
simulator_source="$(resolve_workspace_path "${PASSING_BAY_SIMULATOR_SCENARIO:-rmf_dev_tool-main/vda5050_robot_simulator/p4_scenario.yaml}")"
passing_compose="$(resolve_workspace_path "${PASSING_BAY_COMPOSE_FILE:-rmf_platform-main/docker-compose.p4-passing-bay.yml}")"
arbiter_config="$(resolve_workspace_path "${PASSING_BAY_ARBITER_CONFIG:-config/corridor_blocks_p4_passing_bay.yaml}")"
deployment_profile="${PASSING_BAY_DEPLOYMENT_PROFILE:-}"
fault_scenarios="${PASSING_BAY_FAULT_SCENARIOS:-}"
if [[ -n "${deployment_profile}" ]]; then
  deployment_profile="$(resolve_workspace_path "${deployment_profile}")"
fi
if [[ -n "${fault_scenarios}" ]]; then
  fault_scenarios="$(resolve_workspace_path "${fault_scenarios}")"
fi
robot_ids=("$@")
if [[ "${#robot_ids[@]}" -eq 0 ]]; then
  robot_ids=(AGV_A1 AGV_B1)
fi
expected_robots="${#robot_ids[@]}"
dispatch_script="${PASSING_BAY_DISPATCH_SCRIPT:-t4_dispatch_passing_bay.sh}"

[[ -x "${python_bin}" ]] || {
  echo "가상환경이 없습니다. 먼저 ./scripts/setup_workspace.sh 를 실행하세요." >&2
  exit 1
}
docker image inspect rmf-core:latest >/dev/null 2>&1 || {
  echo "rmf-core:latest 이미지가 없습니다. setup_workspace.sh를 다시 실행하세요." >&2
  exit 1
}
docker image inspect rmf-api-server:latest >/dev/null 2>&1 || {
  echo "rmf-api-server 이미지가 없습니다. setup_workspace.sh를 다시 실행하세요." >&2
  exit 1
}

mkdir -p "${runtime_dir}"

stop_pid_file() {
  local pid_file="$1"
  if [[ -f "${pid_file}" ]]; then
    local pid
    pid="$(cat "${pid_file}")"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}"
      for _ in $(seq 1 20); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.1
      done
    fi
    rm -f "${pid_file}"
  fi
}

port_open() {
  "${python_bin}" - "$1" <<'PY'
import socket
import sys

try:
    with socket.create_connection(("127.0.0.1", int(sys.argv[1])), timeout=0.3):
        pass
except OSError:
    raise SystemExit(1)
PY
}

wait_for_port() {
  local port="$1"
  local label="$2"
  for _ in $(seq 1 90); do
    if port_open "${port}"; then
      echo "[OK] ${label}: ${port}"
      return
    fi
    sleep 1
  done
  echo "${label} 포트 ${port}가 열리지 않았습니다." >&2
  exit 1
}

stop_pid_file "${runtime_dir}/simulator.pid"
stop_pid_file "${runtime_dir}/arbiter.pid"

cd "${platform_dir}"
if ! port_open 1883; then
  docker compose -f "${base_compose}" -f "${portable_compose}" up -d mqtt_broker
fi
wait_for_port 1883 MQTT

docker compose -f "${base_compose}" up -d \
  rmf_traffic_schedule rmf_task_dispatcher rmf_traffic_blockade rmf_api_server

api_code=""
for _ in $(seq 1 90); do
  api_code="$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' \
    http://127.0.0.1:8100/ || true)"
  if [[ "${api_code}" == "404" ]]; then
    echo "[OK] RMF API: HTTP 404"
    break
  fi
  sleep 1
done
[[ "${api_code}" == "404" ]] || {
  echo "RMF API가 준비되지 않았습니다." >&2
  exit 1
}

cd "${simulator_dir}"
"${python_bin}" - "${simulator_source}" "${simulator_config}" "${robot_ids[@]}" <<'PY'
from pathlib import Path
import sys
import yaml

source = Path(sys.argv[1])
target = Path(sys.argv[2])
requested = sys.argv[3:]
data = yaml.safe_load(source.read_text(encoding="utf-8"))
data["robots"] = [
    robot for robot in data["robots"]
    if robot["serial_number"] in set(requested)
]
selected = [robot["serial_number"] for robot in data["robots"]]
if len(requested) != len(set(requested)) or set(selected) != set(requested):
    raise SystemExit(f"unknown or duplicate robot selection: {requested}")
target.write_text(
    yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
    encoding="utf-8",
)
PY

simulator_args=("${python_bin}" run.py --config "${simulator_config}")
if [[ -n "${fault_scenarios}" ]]; then
  simulator_args+=(--fault-scenarios "${fault_scenarios}")
fi
nohup "${simulator_args[@]}" \
  >"${simulator_log}" 2>&1 &
echo "$!" >"${runtime_dir}/simulator.pid"

cd "${platform_dir}"
docker compose -f "${base_compose}" -f "${passing_compose}" up -d \
  --force-recreate vda5050_fleet_adapter

registered=0
for _ in $(seq 1 120); do
  registered="$(docker logs vda5050_fleet_adapter 2>&1 | \
    grep -c 'Successfully added robot' || true)"
  if [[ "${registered}" -ge "${expected_robots}" ]]; then
    echo "[OK] Fleet Adapter: ${robot_ids[*]}"
    break
  fi
  sleep 1
done
[[ "${registered}" -ge "${expected_robots}" ]] || {
  echo "Fleet Adapter가 로봇 ${expected_robots}대를 등록하지 못했습니다." >&2
  exit 1
}

rmf_api_bearer_token="$(docker exec -i rmf_api_server python3 - <<'PY'
import time
import warnings
import jwt
from api_server.app_config import app_config

warnings.filterwarnings("ignore")
now = int(time.time())
print(jwt.encode(
    {
        "preferred_username": app_config.builtin_admin,
        "aud": app_config.aud,
        "iss": app_config.iss,
        "iat": now,
        "exp": now + 43200,
    },
    app_config.jwt_secret,
    algorithm="HS256",
))
PY
)"

cd "${workspace_dir}"
arbiter_args=("${arbiter_config}")
if [[ -n "${deployment_profile}" ]]; then
  arbiter_args+=(--deployment-profile "${deployment_profile}")
fi
nohup env \
  PYTHON_BIN="${python_bin}" \
  RMF_API_BEARER_TOKEN="${rmf_api_bearer_token}" \
  ./scripts/run_direction_arbiter.sh \
  "${arbiter_args[@]}" \
  >"${arbiter_log}" 2>&1 &
echo "$!" >"${runtime_dir}/arbiter.pid"
wait_for_port 8200 Arbiter

echo
echo "Passing-bay runtime is ready."
echo "Dispatch: ./scripts/${dispatch_script}"
echo "Status:   curl -s --noproxy '*' http://127.0.0.1:8200/traffic/status | python3 -m json.tool"
echo "Logs:     tail -F .runtime/arbiter.log"
