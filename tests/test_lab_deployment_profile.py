from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import yaml

from traffic_control.deployment import DeploymentProfile


class LabDeploymentProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root / "nav.yaml").write_text(
            yaml.safe_dump(
                {
                    "levels": {
                        "L1": {
                            "vertices": [
                                [0.0, 0.0, {"name": "A"}],
                                [1.0, 0.0, {"name": "B"}],
                            ],
                            "lanes": [[0, 1, {}], [1, 0, {}]],
                        }
                    }
                }
            ),
            encoding="utf-8",
        )
        (self.root / "corridor.yaml").write_text(
            "traffic_control:\n  enabled: true\n",
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def _write_profile(self, *, operational_checks_required: bool = True) -> Path:
        path = self.root / "lab.yaml"
        path.write_text(
            yaml.safe_dump(
                {
                    "deployment": {"mode": "lab"},
                    "mqtt": {
                        "host": "12.81.224.41",
                        "port": 1883,
                        "state_topic": "uagv/v2/YujinRobot/+/state",
                        "keepalive_sec": 30,
                        "reconnect_max_delay_sec": 30,
                        "tls": {"required": False},
                    },
                    "robots": {
                        "yujin_robot_1": {
                            "manufacturer": "YujinRobot",
                            "serial_number": "yujin_robot_1",
                            "allowed_map_ids": ["L1"],
                            "required": True,
                        }
                    },
                    "calibration": {
                        "rmf": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                        "robot": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                        "max_residual": 0.05,
                        "min_scale": 0.95,
                        "max_scale": 1.05,
                    },
                    "physical": {
                        "footprint_radius": 0.5,
                        "vicinity_radius": 0.7,
                        "max_linear_speed": 0.5,
                        "max_acceleration": 0.25,
                        "max_deceleration": 0.25,
                    },
                    "telemetry": {
                        "state_timeout": 5.0,
                        "connection_timeout": 10.0,
                        "operational_checks_required": operational_checks_required,
                    },
                    "rmf_api": {
                        "url": "http://127.0.0.1:8100/tasks/robot_task"
                    },
                    "paths": {
                        "nav_graph": "nav.yaml",
                        "corridor_config": "corridor.yaml",
                    },
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return path

    def test_lab_mode_is_strict_but_does_not_require_tls_or_secrets(self) -> None:
        profile = DeploymentProfile.load(self._write_profile())

        self.assertEqual(profile.mode, "lab")
        self.assertEqual(profile.validate(), ())

    def test_lab_mode_requires_operational_checks(self) -> None:
        profile = DeploymentProfile.load(
            self._write_profile(operational_checks_required=False)
        )

        self.assertIn(
            "telemetry.operational_checks_required",
            profile.validate(),
        )


if __name__ == "__main__":
    unittest.main()
