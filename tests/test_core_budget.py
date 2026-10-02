"""Compile and exercise the exact standalone header installed by the core patch."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
@unittest.skipUnless(shutil.which('g++'), 'requires C++17 compiler')
class CoreBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        p = Path(cls.temp.name)
        (p/'test.cpp').write_text('''
#include "SearchBudget.hpp"
#include <iostream>
int main() {
 try {
 auto b=rmf_fleet_adapter::local_traffic::SearchBudget::load();
 std::atomic_bool cancel{false};
 using T=std::chrono::steady_clock::time_point;
 const T now{};
 std::optional<T> missing;
 if(rmf_fleet_adapter::local_traffic::interrupted(cancel,missing,now)) return 4;
 std::optional<T> expired{now};
 if(!rmf_fleet_adapter::local_traffic::interrupted(cancel,expired,now)) return 5;
 cancel=true;
 if(!rmf_fleet_adapter::local_traffic::interrupted(cancel,missing,now)) return 6;
 if (b.planning_node_limit(250000) != (b.enabled ? 250000 : 10000)) return 7;
 if (b.planning_node_limit(std::nullopt) != (b.enabled ? b.nodes : 10000)) return 8;
 std::cout << b.enabled << " " << b.nodes << " " << b.seconds << " " << b.compliant_cost(2);
 } catch(const std::exception&) { return 2; }
}
''')
        cls.exe = str(p/'budget')
        subprocess.run(['g++','-std=c++17','-I',str(ROOT/'core_local_traffic'),
                        str(p/'test.cpp'),'-o',cls.exe], check=True, capture_output=True)
    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()
    def invoke(self, **settings):
        env={k:v for k,v in os.environ.items() if not k.startswith('RMF_LOCAL_TRAFFIC_')}
        env.update(settings)
        return subprocess.run([self.exe],env=env,text=True,capture_output=True)
    def test_default_and_opt_in(self):
        self.assertEqual(self.invoke().stdout, '0 10000 5 6')
        self.assertEqual(self.invoke(RMF_LOCAL_TRAFFIC_BUDGETS='1').stdout, '1 100000 20 122')
    def test_invalid_limits_rejected(self):
        for key,value in [('NODE_LIMIT','10.5'),('SOLVE_SECONDS','nan'),('COST_LEEWAY','-1'),('EXTRA_COST','120x')]:
            with self.subTest(key=key):
                self.assertEqual(self.invoke(RMF_LOCAL_TRAFFIC_BUDGETS='1',
                    **{'RMF_LOCAL_TRAFFIC_'+key:value}).returncode,2)
