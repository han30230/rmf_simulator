#!/usr/bin/env python3
"""Kill/reap the evaluator process on hard timeout; never leave a solve thread alive."""
import argparse
import hashlib
import re
from pathlib import Path
import subprocess
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
def artifact(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def run(binary, graph, scenario, hard_timeout):
    cfg = yaml.safe_load(Path(scenario).read_text())
    limits = {key: cfg.get(key, default) for key, default in dict(
        solve_seconds=10.0, node_limit=100000, cost_leeway=10.0,
        extra_cost=120.0, tail_seconds=60.0).items()}
    limits.update(maximum_alternatives=100, hard_timeout_seconds=hard_timeout)
    provenance = dict(binary=artifact(binary), graph=artifact(graph), scenario=artifact(scenario))
    # The same loader environment is inherited by ldd and the evaluator.
    # For hard timeouts no evaluator report may exist; label that distinction.
    linkage = subprocess.run(['ldd', str(binary)], capture_output=True, text=True, timeout=5)
    match = re.search(r'librmf_traffic\.so[^ ]* => (\S+)', linkage.stdout)
    if match and Path(match[1]).is_file():
        provenance['rmf_traffic_library'] = artifact(match[1])
        provenance['rmf_traffic_library']['observation'] = 'loader_resolution'
    else:
        provenance['rmf_traffic_library'] = dict(path=None, sha256=None, observation='unavailable')
    t0 = time.monotonic()
    try:
        p = subprocess.run([str(binary), str(graph), str(scenario)], capture_output=True,
                           text=True, timeout=hard_timeout)
    except subprocess.TimeoutExpired:
        result = dict(status='HARD_TIMEOUT', diagnostics=dict(termination_reason='SUBPROCESS_HARD_TIMEOUT'))
    else:
        if p.returncode:
            result = dict(status='EVALUATOR_ERROR', returncode=p.returncode, stderr=p.stderr)
        else:
            try:
                result = yaml.safe_load(p.stdout)
            except yaml.YAMLError:
                result = None
            if not isinstance(result, dict) or 'status' not in result:
                result = dict(status='INVALID_OUTPUT', stderr=p.stderr)
            loaded = result.get('loaded_rmf_traffic_library')
            if loaded:
                provenance['rmf_traffic_library'] = artifact(loaded)
                provenance['rmf_traffic_library']['observation'] = 'evaluator_proc_maps'
    result.update(wall_seconds=time.monotonic()-t0, provenance=provenance, limits=limits)
    return result

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--binary',required=True,type=Path)
    p.add_argument('--graph',type=Path,default=ROOT/'rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_local_traffic.yaml')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repeat',type=int,default=10)
    args=p.parse_args()
    if args.repeat < 1: p.error('--repeat must be positive')
    args.output.mkdir(parents=True,exist_ok=True)
    summary=[]
    for scenario in sorted((ROOT/'core_local_traffic/scenarios').glob('*.yaml')):
        cfg=yaml.safe_load(scenario.read_text())
        for n in range(args.repeat):
            result=run(args.binary,args.graph,scenario,float(cfg.get('solve_seconds',10))+5)
            path=args.output/f'{scenario.stem}-{n:02}.yaml'
            path.write_text(yaml.safe_dump(result,sort_keys=False))
            item=dict(scenario=scenario.name,repeat=n,status=result['status'],
                      wall_seconds=result.get('wall_seconds'),result=path.name)
            summary.append(item)
            print(item,flush=True)
    (args.output/'summary.yaml').write_text(yaml.safe_dump(summary,sort_keys=False))
if __name__=='__main__': main()
