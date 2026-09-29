#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo "usage: $0 /path/to/dsr-lab-profile.yaml" >&2
  exit 2
fi
PROFILE="$1"
PORT="${DSR_ARBITER_PORT:-18200}"
CONTAINER="${WAVE_ADAPTER_CONTAINER:-Wave_adapter}"
EXPECTED_REVISION="${EXPECTED_WAVE_NAV_REVISION:-}"
EVENT_LOG="${DSR_EVENT_LOG:-$ROOT/.runtime/dsr-lab/arbiter-events.jsonl}"

cd "$ROOT"
mkdir -p .runtime
EVENT_LOG="${DSR_EVENT_LOG:-$ROOT/.runtime/dsr_lab_events.jsonl}"

if [[ ! -x ".venv/bin/python" ]]; then
  echo "ERROR missing .venv; create/activate the repository virtualenv first" >&2
  exit 2
fi

readarray -t PATHS < <(
  .venv/bin/python - "$PROFILE" <<'PY'
from pathlib import Path
import sys
from traffic_control.deployment import DeploymentProfile

profile = DeploymentProfile.load(Path(sys.argv[1]))
profile.require_valid()
print(profile.paths.nav_graph)
print(profile.paths.corridor_config)
PY
)

NAV_GRAPH="${PATHS[0]}"
CORRIDOR="${PATHS[1]}"

CHECK=(
  .venv/bin/python scripts/check_wave_runtime_graph.py
  --container "$CONTAINER"
  --snapshot "$NAV_GRAPH"
  --corridor "$CORRIDOR"
)
if [[ -n "$EXPECTED_REVISION" ]]; then
  CHECK+=(--expected-revision "$EXPECTED_REVISION")
fi
"${CHECK[@]}"

if ss -lnt | grep -qE "[:.]$PORT([[:space:]]|$)"; then
  echo "ERROR port $PORT is already in use" >&2
  exit 2
fi

if ss -lnt | grep -qE '(^|[[:space:]])(0\.0\.0\.0|\[::\]):8100([[:space:]]|$)'; then
  echo "WARNING RMF API :8100 is externally bound; managed DSR tasks must still be sent only to TaskGate :$PORT" >&2
fi

echo "Starting DSR TaskGate on 127.0.0.1:$PORT using strict lab profile"
echo "Managed task endpoint: http://127.0.0.1:$PORT/tasks/robot_task"
exec .venv/bin/python -m traffic_control.task_gate \
  --deployment-profile "$PROFILE" \
  --listen-host 127.0.0.1 \
  --port "$PORT" \
  --event-log "$EVENT_LOG" \
  --log-level INFO
