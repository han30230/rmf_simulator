#!/usr/bin/env bash
set -euo pipefail

base="${DSR_GATE_BASE_URL:-http://127.0.0.1:18200}"

echo "=== HEALTH ==="
curl --fail-with-body --silent --show-error --noproxy '*' "$base/health" |
  python3 -m json.tool

echo
echo "=== READY ==="
if ! curl --fail-with-body --silent --show-error --noproxy '*' "$base/ready" |
  python3 -m json.tool; then
  echo "TaskGate is not ready" >&2
fi

echo
echo "=== TRAFFIC STATUS ==="
curl --fail-with-body --silent --show-error --noproxy '*' "$base/traffic/status" |
  python3 -m json.tool
