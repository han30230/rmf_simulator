from pathlib import Path
import math
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / 'rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_local_traffic.yaml'
SIM = ROOT / 'rmf_dev_tool-main/vda5050_robot_simulator/p4_local_traffic.yaml'

class LocalTrafficConfigTests(unittest.TestCase):
    def test_bay_is_accessible_and_off_main_lane(self):
        self.assertTrue(MAP.is_file(), 'local traffic map missing')
        level = yaml.safe_load(MAP.read_text())['levels']['L1']
        vertices = level['vertices']
        by_name = {str(v[2]['name']): i for i, v in enumerate(vertices)}
        bay, junction = by_name['6137'], by_name['2105']
        lanes = {(lane[0], lane[1]) for lane in level['lanes']}
        self.assertIn((bay, junction), lanes)
        self.assertIn((junction, bay), lanes)
        self.assertTrue(vertices[bay][2]['is_holding_point'])
        self.assertGreater(abs(vertices[bay][1] - vertices[junction][1]), 1.4)

    def test_idle_robots_do_not_start_overlapping(self):
        self.assertTrue(SIM.is_file(), 'local traffic simulator config missing')
        cfg = yaml.safe_load(SIM.read_text())
        poses = [r['initial_position'] for r in cfg['robots']]
        self.assertEqual(len(poses), 4)
        for i, a in enumerate(poses):
            for b in poses[i + 1:]:
                self.assertGreater(math.hypot(a['x'] - b['x'], a['y'] - b['y']), 1.4)
        self.assertTrue(cfg['robot_defaults']['obstacle_detection']['enabled'])
