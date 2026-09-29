#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scenario="${1:-1v3}"
export PASSING_BAY_SIMULATOR_SCENARIO=rmf_dev_tool-main/vda5050_robot_simulator/connected_corridor_chain_scenario.yaml
export PASSING_BAY_COMPOSE_FILE=rmf_platform-main/docker-compose.connected-corridor-chain.yml
export PASSING_BAY_ARBITER_CONFIG=config/corridor_blocks_connected_chain.yaml
export PASSING_BAY_RUNTIME_NAME="connected_corridor_chain_${scenario}"
case "${scenario}" in
  1v3) exec "${script_dir}/start_p4_passing_bay.sh" AGV_A1 AGV_B1 AGV_B2 AGV_B3 ;;
  2v2) exec "${script_dir}/start_p4_passing_bay.sh" AGV_A1 AGV_A2 AGV_B1 AGV_B2 ;;
  *) echo "usage: $0 [1v3|2v2]" >&2; exit 2 ;;
esac
