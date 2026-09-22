#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec "${workspace_dir}/.venv/bin/python" \
  "${workspace_dir}/scripts/t4_dispatch_passing_bay_staging_dynamic_1v3.py" \
  "$@"
