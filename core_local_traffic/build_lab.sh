#!/usr/bin/env bash
# Standalone C++ evaluator. Requires cmake, g++, Eigen3, libccd, yaml-cpp dev packages.
set -euo pipefail
lab_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# != 1 ]]; then
  echo 'usage: build_lab.sh NEW_LAB_DIRECTORY' >&2; exit 2
fi
lab_workspace="$1"
if [[ -e "$lab_workspace" ]]; then
  echo 'lab directory must not already exist (preserve existing workspaces)' >&2; exit 2
fi
mkdir -p "$lab_workspace"
lab_workspace="$(cd -- "$lab_workspace" && pwd)"
python3 - "$lab_dir" "$lab_workspace" <<'PY'
import pathlib, subprocess, sys, yaml
lab, ws = map(pathlib.Path,sys.argv[1:])
manifest=yaml.safe_load((lab/'manifest.yaml').read_text())
for name, item in manifest.items():
    repo=ws/'src'/name
    subprocess.run(['git','clone',item['url'],str(repo)],check=True)
    subprocess.run(['git','-C',str(repo),'checkout','--detach',item['commit']],check=True)
subprocess.run([sys.executable,str(lab/'apply_overlay.py'),str(ws/'src'),'--apply'],check=True)
PY
cmake -S "$lab_workspace/src/rmf_utils/rmf_utils" -B "$lab_workspace/build/utils" \
  -DCMAKE_INSTALL_PREFIX="$lab_workspace/install" -DBUILD_TESTING=OFF
cmake --build "$lab_workspace/build/utils" -j2
cmake --install "$lab_workspace/build/utils"
cmake -S "$lab_workspace/src/rmf_traffic/rmf_traffic" -B "$lab_workspace/build/traffic" \
  -DCMAKE_PREFIX_PATH="$lab_workspace/install" -DCMAKE_INSTALL_PREFIX="$lab_workspace/install" \
  -DBUILD_TESTING=OFF -DFCL_WITH_OCTOMAP=OFF
cmake --build "$lab_workspace/build/traffic" -j2
cmake --install "$lab_workspace/build/traffic"
cmake -S "$lab_dir" -B "$lab_workspace/build/evaluator" -DCMAKE_PREFIX_PATH="$lab_workspace/install"
cmake --build "$lab_workspace/build/evaluator" -j2
printf 'Evaluator: %s\n' "$lab_workspace/build/evaluator/local_traffic_evaluator"
