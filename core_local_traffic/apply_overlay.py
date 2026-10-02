#!/usr/bin/env python3
"""Apply lab patches only to exact, clean pinned checkouts. No automatic upgrade."""
import argparse
from pathlib import Path
import shutil
import subprocess
import yaml
LAB=Path(__file__).resolve().parent

def git(repo,*args):
    return subprocess.run(['git','-C',str(repo),*args],check=True,
                          capture_output=True,text=True).stdout.strip()

def apply(source, mutate=False):
    manifest=yaml.safe_load((LAB/'manifest.yaml').read_text())
    checked=[]
    for name,data in manifest.items():
        if 'patch' not in data: continue
        repo=source/name
        if git(repo,'rev-parse','HEAD') != data['commit']:
            raise RuntimeError(f'{name}: commit mismatch; refusing to modify')
        if git(repo,'status','--porcelain'):
            raise RuntimeError(f'{name}: checkout has changes; refusing to modify')
        patch=LAB/data['patch']
        git(repo,'apply','--check',str(patch))
        checked.append((repo,patch))
    if not mutate: return
    # Validate every checkout before any write. These changes remain uncommitted.
    for repo,patch in checked: git(repo,'apply',str(patch))
    target=source/'rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/local_traffic/SearchBudget.hpp'
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(LAB/'SearchBudget.hpp',target)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('source',type=Path,help='directory containing clean rmf_ros2 and rmf_traffic')
    parser.add_argument('--apply',action='store_true',help='write patches; otherwise dry check')
    args=parser.parse_args()
    try: apply(args.source,args.apply)
    except (RuntimeError,subprocess.CalledProcessError) as e: parser.exit(2,f'{e}\n')
    print('patches applied' if args.apply else 'exact versions and patch application checked')
