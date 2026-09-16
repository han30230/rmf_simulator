#!/usr/bin/env bash
set -euo pipefail

gate_url="${GATE_URL:-http://127.0.0.1:8200/tasks/robot_task}"

dispatch() {
  local robot_id="$1"
  local goal_node="$2"
  curl --silent --show-error --noproxy '*' \
    --request POST "${gate_url}" \
    --header 'Content-Type: application/json' \
    --data-binary @- <<JSON
{"type":"robot_task_request","robot":"${robot_id}","fleet":"TOOL","request":{"unix_millis_earliest_start_time":0,"category":"patrol","priority":{"type":"default","value":0},"description":{"places":["${goal_node}"],"rounds":1}}}
JSON
  echo
}

if [[ "$#" -eq 2 ]]; then
  dispatch "$1" "$2"
  exit 0
fi

if [[ "$#" -ne 0 ]]; then
  echo "usage: $0 [ROBOT_ID GOAL_NODE]" >&2
  exit 2
fi

# Example node IDs only. Replace them with the active map/config values.
dispatch AGV_A1 N3
dispatch AGV_A2 N3
dispatch AGV_B1 N0
dispatch AGV_B2 N0
