#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "usage: bash $0 ROBOT_ID GOAL_NODE [IDEMPOTENCY_KEY]" >&2
  exit 2
fi

robot_id="$1"
goal_node="$2"
gate_url="${DSR_GATE_URL:-http://127.0.0.1:18200/tasks/robot_task}"
key="${3:-dsr-${robot_id}-${goal_node}-$(date +%s%N)}"

curl --fail-with-body --silent --show-error --noproxy '*' \
  --request POST "$gate_url" \
  --header 'Content-Type: application/json' \
  --header "Idempotency-Key: $key" \
  --data-binary @- <<JSON
{"type":"robot_task_request","robot":"${robot_id}","fleet":"WAVE","request":{"unix_millis_earliest_start_time":0,"category":"patrol","priority":{"type":"default","value":0},"description":{"places":["${goal_node}"],"rounds":1}}}
JSON
echo
echo "idempotency_key=$key" >&2
