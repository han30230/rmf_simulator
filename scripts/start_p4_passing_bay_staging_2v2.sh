#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PASSING_BAY_SIMULATOR_SCENARIO=rmf_dev_tool-main/vda5050_robot_simulator/p4_staging_scenario.yaml
export PASSING_BAY_COMPOSE_FILE=rmf_platform-main/docker-compose.p4-passing-bay-staging.yml
export PASSING_BAY_ARBITER_CONFIG=config/corridor_blocks_p4_passing_bay_staging.yaml
export PASSING_BAY_RUNTIME_NAME=p4_passing_bay_staging_2v2
export PASSING_BAY_DISPATCH_SCRIPT=t4_dispatch_passing_bay_staging_2v2.sh

exec "${script_dir}/start_p4_passing_bay.sh" AGV_A1 AGV_A2 AGV_B1 AGV_B2
