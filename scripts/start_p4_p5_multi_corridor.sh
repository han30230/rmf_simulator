#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PASSING_BAY_SIMULATOR_SCENARIO=rmf_dev_tool-main/vda5050_robot_simulator/p4_p5_multi_corridor_scenario.yaml
export PASSING_BAY_COMPOSE_FILE=rmf_platform-main/docker-compose.p4-p5-multi-corridor.yml
export PASSING_BAY_ARBITER_CONFIG=config/corridor_blocks_p4_p5_multi.yaml
export PASSING_BAY_RUNTIME_NAME=p4_p5_multi_corridor
export PASSING_BAY_DISPATCH_SCRIPT=t4_dispatch_p4_p5_multi_corridor.sh

exec "${script_dir}/start_p4_passing_bay.sh" AGV_P4_A AGV_P4_B AGV_P5_A AGV_P5_B
