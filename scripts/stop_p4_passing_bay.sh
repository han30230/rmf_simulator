#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="${workspace_dir}/.runtime"
platform_dir="${workspace_dir}/rmf_platform-main"

stop_pid_file() {
  local pid_file="$1"
  local label="$2"
  if [[ ! -f "${pid_file}" ]]; then
    echo "[SKIP] ${label}: PID file 없음"
    return
  fi

  local pid
  pid="$(cat "${pid_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}"
    echo "[OK] ${label} 종료: ${pid}"
  fi
  rm -f "${pid_file}"
}

stop_pid_file "${runtime_dir}/arbiter.pid" Arbiter
stop_pid_file "${runtime_dir}/simulator.pid" Simulator

if [[ "${1:-}" == "--all" ]]; then
  cd "${platform_dir}"
  docker compose \
    -f docker-compose.yml \
    -f docker-compose.portable.yml \
    -f docker-compose.p4-passing-bay.yml \
    stop \
    vda5050_fleet_adapter rmf_api_server rmf_task_dispatcher \
    rmf_traffic_blockade rmf_traffic_schedule mqtt_broker
fi
