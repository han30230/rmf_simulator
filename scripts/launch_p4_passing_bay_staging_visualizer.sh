#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${workspace_dir}/.venv/bin/python"
gui="${workspace_dir}/rmf_dev_tool-main/vda5050_gui/vda5050_gui.py"
map="${workspace_dir}/rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_passing_bay_staging.yaml"

exec "${python_bin}" "${gui}" --map "${map}"
