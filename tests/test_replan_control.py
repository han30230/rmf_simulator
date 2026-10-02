from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rmf_platform-main/src/rmf_vda5050_fleet_adapter'))
from vda5050_fleet_adapter.presentation.replan_control import request_replan
class CoreHandle:
    def __init__(self): self.requests=0
    def replan(self): self.requests += 1
class EasyHandle:
    def __init__(self): self.core=CoreHandle()
    def more(self): return self.core
class Robot:
    def __init__(self, registered=True):
        self.update_handle=EasyHandle() if registered else None
class ReplanControlTests(unittest.TestCase):
    def test_requests_real_core_replan_handle(self):
        r=Robot()
        self.assertTrue(request_replan({'r':r},'r'))
        self.assertEqual(r.update_handle.core.requests,1)
    def test_unknown_or_unregistered_robot_not_accepted(self):
        self.assertFalse(request_replan({},'missing'))
        self.assertFalse(request_replan({'r':Robot(False)},'r'))
