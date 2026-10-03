"""Runner provenance must survive successful and hard-timeout subprocesses."""
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
import yaml
from core_local_traffic.run_matrix import run

ROOT = Path(__file__).resolve().parents[1]
BINARY = os.environ.get('RMF_LOCAL_TRAFFIC_EVALUATOR')
MAP = ROOT / 'rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_local_traffic.yaml'

@unittest.skipUnless(BINARY, 'build evaluator and set RMF_LOCAL_TRAFFIC_EVALUATOR')
class CoreMatrixTests(unittest.TestCase):
    def check_provenance(self, result, scenario, timeout):
        self.assertIn('provenance', result)
        p = result['provenance']
        for key, path in [('binary', Path(BINARY)), ('scenario', scenario), ('graph', MAP)]:
            self.assertEqual(p[key]['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
            self.assertEqual(p[key]['path'], str(path.resolve()))
        library = p['rmf_traffic_library']
        self.assertEqual(library['sha256'], hashlib.sha256(Path(library['path']).read_bytes()).hexdigest())
        self.assertIn('librmf_traffic.so', library['path'])
        self.assertEqual(result['limits'], dict(solve_seconds=10.0, node_limit=12345,
                         cost_leeway=7.0, extra_cost=99.0, tail_seconds=60.0,
                         maximum_alternatives=100, hard_timeout_seconds=timeout))

    def test_each_result_records_loaded_artifacts_and_exact_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            scenario = Path(tmp) / 'scenario.yaml'
            scenario.write_text(yaml.safe_dump(dict(agents=[dict(name='a', start='2101', goal='2108')],
                               node_limit=12345, cost_leeway=7, extra_cost=99)))
            result = run(BINARY, MAP, scenario, 15)
            self.assertEqual(result['status'], 'VALID')
            self.check_provenance(result, scenario, 15)

    def test_hard_timeout_keeps_provenance_and_distinct_reason(self):
        scenario = ROOT / 'core_local_traffic/scenarios/opposite_3.yaml'
        with tempfile.TemporaryDirectory() as tmp:
            fixture = Path(tmp) / 'scenario.yaml'
            cfg = yaml.safe_load(scenario.read_text())
            cfg.update(solve_seconds=10, node_limit=12345, cost_leeway=7, extra_cost=99)
            fixture.write_text(yaml.safe_dump(cfg))
            result = run(BINARY, MAP, fixture, 0.001)
            self.assertEqual(result['status'], 'HARD_TIMEOUT')
            self.check_provenance(result, fixture, 0.001)
            self.assertEqual(result['diagnostics']['termination_reason'], 'SUBPROCESS_HARD_TIMEOUT')
