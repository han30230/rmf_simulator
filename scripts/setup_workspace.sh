#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON_BIN:-python3}"
rmf_core_source="${RMF_CORE_SOURCE_IMAGE:-ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest}"
rmf_api_source="${RMF_API_SOURCE_IMAGE:-ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly}"

cd "${workspace_dir}"

command -v "${python_bin}" >/dev/null || {
  echo "Python 3가 필요합니다: ${python_bin}" >&2
  exit 1
}
command -v docker >/dev/null || {
  echo "Docker가 필요합니다. Docker Desktop의 WSL integration을 켜주세요." >&2
  exit 1
}
docker info >/dev/null
docker compose version >/dev/null

if [[ ! -x .venv/bin/python ]]; then
  "${python_bin}" -m venv .venv
fi

.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

ensure_image() {
  local target="$1"
  local source="$2"

  if docker image inspect "${target}" >/dev/null 2>&1; then
    echo "[OK] Docker image: ${target}"
    return
  fi

  if ! docker image inspect "${source}" >/dev/null 2>&1; then
    docker pull "${source}"
  fi
  docker tag "${source}" "${target}"
  echo "[OK] Docker image alias: ${target} -> ${source}"
}

ensure_image rmf-core:latest "${rmf_core_source}"
ensure_image rmf-api-server:latest "${rmf_api_source}"

.venv/bin/python -m unittest discover -s tests -p 'test_*.py'

echo
echo "Workspace setup complete."
echo "Next: ./scripts/start_p4_passing_bay.sh"
