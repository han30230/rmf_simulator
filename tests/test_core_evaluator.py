"""Integration tests against real rmf_traffic (never a Python planner substitute)."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
BINARY = os.environ.get('RMF_LOCAL_TRAFFIC_EVALUATOR')
MAP = ROOT / 'rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_local_traffic.yaml'

@unittest.skipUnless(BINARY, 'build C++ evaluator and set RMF_LOCAL_TRAFFIC_EVALUATOR')
class CoreEvaluatorTests(unittest.TestCase):
    def solve(self, agents, graph=MAP, **options):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as f:
            yaml.safe_dump(dict(agents=agents, **options), f)
            f.flush()
            proc = subprocess.run([BINARY, str(graph), f.name], capture_output=True,
                                  text=True, timeout=30)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            return yaml.safe_load(proc.stdout)

    def test_single_robot_preserves_goal(self):
        r = self.solve([dict(name='one', start='2101', goal='2108')])
        self.assertEqual(r['status'], 'VALID')
        self.assertEqual(r['plans'][0]['goal'], '2108')
        self.assertGreater(r['plans'][0]['finish_seconds'], 0)

    def test_overlapping_starts_are_rejected(self):
        r = self.solve([dict(name='a', start='2101', goal='2108'),
                        dict(name='b', start='2101', goal='2102')])
        self.assertEqual(r['status'], 'INVALID_INITIAL_OCCUPANCY')

    def test_idle_goal_does_not_disappear(self):
        r = self.solve([dict(name='move', start='2101', goal='2108'),
                        dict(name='idle', start='2108', goal='2108', fixed=True)])
        self.assertNotEqual(r['status'], 'VALID')

    def test_duplicate_names_rejected(self):
        with self.assertRaises(AssertionError):
            self.solve([dict(name='a', start='2101', goal='2108'),
                        dict(name='a', start='2108', goal='2101')])

    def test_negotiation_deadline_returns_without_orphan_worker(self):
        r = self.solve([dict(name='a', start='2101', goal='2108'),
                        dict(name='b', start='2108', goal='2101', yaw=3.141592653589793),
                        dict(name='c', start='PARK_E', goal='2102', yaw=3.141592653589793)],
                       solve_seconds=10.0)
        self.assertEqual(r['status'], 'TIMEOUT')
        self.assertLess(r['solve_seconds'], 12.0)

    def test_timeout_has_bounded_search_diagnostics_not_impossibility(self):
        r = self.solve([dict(name='a', start='2101', goal='2108'),
                        dict(name='b', start='2108', goal='2101', yaw=3.141592653589793),
                        dict(name='c', start='PARK_E', goal='2102', yaw=3.141592653589793)],
                       solve_seconds=0.2, node_limit=12345, cost_leeway=7, extra_cost=99)
        self.assertEqual(r['status'], 'TIMEOUT')
        self.assertIn('diagnostics', r)
        d = r['diagnostics']
        self.assertEqual(d['termination_reason'], 'TIMEOUT')
        self.assertEqual(d['limits']['node_limit'], 12345)
        self.assertEqual(d['limits']['cost_leeway'], 7)
        self.assertEqual(d['limits']['extra_cost'], 99)
        self.assertEqual(d['limits']['solve_seconds'], 0.2)
        self.assertGreater(d['tables_selected'], 0)
        self.assertGreater(d['respond_calls'], 0)
        self.assertGreater(d['plan_calls'], 0)
        self.assertGreater(d['rollout_calls'], 0)
        self.assertGreaterEqual(d['elapsed_seconds'], 0.2)
        self.assertLessEqual(len(d['events']), d['event_limit'])
        self.assertTrue(any(e.get('candidate_endpoints') for e in d['events']))
        self.assertNotIn('physical_impossibility: true', yaml.safe_dump(r))

    def test_disconnected_goal_reports_no_route(self):
        cfg = yaml.safe_load(MAP.read_text())
        cfg['levels']['L1']['lanes'] = []
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as graph:
            yaml.safe_dump(cfg, graph); graph.flush()
            r = self.solve([dict(name='one', start='2101', goal='2108')], graph=graph.name)
        self.assertEqual(r['status'], 'NO_ROUTE')
        self.assertGreater(r['diagnostics']['no_route_plans'], 0)

    def test_opposite_two_uses_bay_and_keeps_original_goals(self):
        r=self.solve([dict(name='a',start='2101',goal='2108'),
                      dict(name='b',start='2108',goal='2101',yaw=3.141592653589793)])
        self.assertEqual(r['status'],'VALID')
        self.assertEqual({p['goal'] for p in r['plans']},{'2101','2108'})
        self.assertTrue(any(abs(w['y']-89.8)<0.001
                            for p in r['plans'] for w in p['trajectory']))

    def test_opposite_two_without_bay_never_reports_success(self):
        cfg=yaml.safe_load(MAP.read_text())
        cfg['levels']['L1']['lanes']=[lane for lane in cfg['levels']['L1']['lanes']
                                    if 8 not in lane[:2]]
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as graph:
            yaml.safe_dump(cfg,graph);graph.flush()
            r=self.solve([dict(name='a',start='2101',goal='2108'),
                          dict(name='b',start='2108',goal='2101',yaw=3.141592653589793)],
                         graph=graph.name)
        self.assertNotEqual(r['status'],'VALID')

    def test_unbounded_or_invalid_solver_options_rejected(self):
        for options in [dict(node_limit=0),dict(node_limit=1000001),
                        dict(cost_leeway=float('nan')),dict(extra_cost=-1)]:
            with self.subTest(options=options), self.assertRaises(AssertionError):
                self.solve([dict(name='one',start='2101',goal='2108')],**options)
