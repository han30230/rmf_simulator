from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml

from traffic_control.deployment import DeploymentConfigError, DeploymentProfile


ROOT = Path(__file__).resolve().parents[1]


class DeploymentProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary.name)
        for name in ("ca.pem", "client.pem", "client.key"):
            (self.root / name).write_text(f"test {name}\n", encoding="utf-8")
        for name in ("fleet.yaml", "corridor.yaml", "compose.yaml"):
            (self.root / name).write_text("{}\n", encoding="utf-8")
        self.map_path = self.root / "map.yaml"
        self._write_map(simulation_only=False)

    def tearDown(self) -> None:
        self._temporary.cleanup()

    def _write_map(self, *, simulation_only: bool) -> None:
        self.map_path.write_text(
            yaml.safe_dump({
                "building_name": "fab",
                "metadata": {"simulation_only": simulation_only},
                "levels": {"L1": {"vertices": [], "lanes": []}},
            }),
            encoding="utf-8",
        )

    def _raw(self, *, mode: str = "production") -> dict:
        return {
            "deployment": {"mode": mode},
            "mqtt": {
                "host": "mqtt.fab.example",
                "port": 8883,
                "keepalive_sec": 30,
                "reconnect_max_delay_sec": 30,
                "username": {"env": "FAB_MQTT_USER"},
                "password": {"env": "FAB_MQTT_PASSWORD"},
                "tls": {
                    "required": True,
                    "ca_file": "ca.pem",
                    "client_cert_file": "client.pem",
                    "client_key_file": "client.key",
                },
            },
            "robots": {
                "ROBOT_01": {
                    "manufacturer": "vendor",
                    "serial_number": "ROBOT_01",
                    "allowed_map_ids": ["FAB_L1"],
                    "required": True,
                }
            },
            "calibration": {
                "rmf": [[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]],
                "robot": [[100.0, 100.0], [110.0, 100.0], [100.0, 110.0]],
            },
            "physical": {
                "footprint_radius": 0.5,
                "vicinity_radius": 0.7,
                "max_linear_speed": 1.0,
                "max_acceleration": 0.5,
                "max_deceleration": 0.5,
            },
            "telemetry": {
                "state_timeout": 5.0,
                "connection_timeout": 10.0,
                "operational_checks_required": True,
            },
            "rmf_api": {
                "url": "https://rmf.fab.example/tasks/robot_task",
                "bearer_token": {"env": "RMF_API_TOKEN"},
            },
            "paths": {
                "fleet_config": "fleet.yaml",
                "nav_graph": "map.yaml",
                "corridor_config": "corridor.yaml",
                "compose_files": ["compose.yaml"],
            },
        }

    def _write(self, raw: dict, name: str = "deployment.yaml") -> Path:
        path = self.root / name
        path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
        return path

    @staticmethod
    def _environment() -> dict[str, str]:
        return {
            "FAB_MQTT_USER": "operator",
            "FAB_MQTT_PASSWORD": "super-secret",
            "RMF_API_TOKEN": "signed-token",
        }

    def test_valid_production_profile_has_no_errors(self) -> None:
        profile = DeploymentProfile.load(
            self._write(self._raw()), environ=self._environment()
        )

        self.assertEqual(profile.validate(), ())
        self.assertEqual(profile.mode, "production")
        self.assertEqual(profile.mqtt.host, "mqtt.fab.example")
        self.assertEqual(profile.robots["ROBOT_01"].allowed_map_ids, ("FAB_L1",))

    def test_simulation_profile_does_not_require_field_secrets(self) -> None:
        raw = self._raw(mode="simulation")
        raw["mqtt"] = {"host": "127.0.0.1", "port": 1883}
        raw["rmf_api"] = {"url": "http://127.0.0.1:8100/tasks/robot_task"}
        raw["calibration"] = {"rmf": [], "robot": []}

        profile = DeploymentProfile.load(self._write(raw), environ={})

        self.assertEqual(profile.validate(), ())

    def test_production_rejects_placeholder_and_simulation_map(self) -> None:
        raw = self._raw()
        raw["mqtt"]["host"] = "REPLACE_ME_BROKER"
        self._write_map(simulation_only=True)

        errors = DeploymentProfile.load(
            self._write(raw), environ=self._environment()
        ).validate()

        self.assertIn("mqtt.host.placeholder", errors)
        self.assertIn("map.simulation_only", errors)

    def test_production_requires_three_non_collinear_reference_points(self) -> None:
        raw = self._raw()
        raw["calibration"] = {
            "rmf": [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]],
            "robot": [[0.0, 0.0], [2.0, 2.0], [4.0, 4.0]],
        }

        errors = DeploymentProfile.load(
            self._write(raw), environ=self._environment()
        ).validate()

        self.assertIn("calibration.rmf.collinear", errors)
        self.assertIn("calibration.robot.collinear", errors)

    def test_production_rejects_duplicate_robot_identity_and_empty_map_ids(self) -> None:
        raw = self._raw()
        raw["robots"]["ROBOT_02"] = {
            "manufacturer": "vendor",
            "serial_number": "ROBOT_01",
            "allowed_map_ids": [],
            "required": True,
        }

        errors = DeploymentProfile.load(
            self._write(raw), environ=self._environment()
        ).validate()

        self.assertIn("robots.identity.duplicate", errors)
        self.assertIn("robots.ROBOT_02.allowed_map_ids.empty", errors)

    def test_production_rejects_unresolved_secrets_and_insecure_tls(self) -> None:
        raw = self._raw()
        raw["mqtt"]["tls"]["required"] = False

        errors = DeploymentProfile.load(self._write(raw), environ={}).validate()

        self.assertIn("mqtt.tls.required", errors)
        self.assertIn("mqtt.username.unresolved", errors)
        self.assertIn("mqtt.password.unresolved", errors)
        self.assertIn("rmf_api.bearer_token.unresolved", errors)

    def test_production_rejects_missing_client_key_and_nonpositive_values(self) -> None:
        raw = self._raw()
        raw["mqtt"]["tls"].pop("client_key_file")
        raw["physical"]["max_deceleration"] = 0
        raw["telemetry"]["state_timeout"] = 0

        errors = DeploymentProfile.load(
            self._write(raw), environ=self._environment()
        ).validate()

        self.assertIn("mqtt.tls.client_pair", errors)
        self.assertIn("physical.max_deceleration.nonpositive", errors)
        self.assertIn("telemetry.state_timeout.nonpositive", errors)

    def test_redacted_snapshot_never_contains_resolved_secrets(self) -> None:
        profile = DeploymentProfile.load(
            self._write(self._raw()), environ=self._environment()
        )

        rendered = json.dumps(profile.redacted_snapshot())

        self.assertNotIn("super-secret", rendered)
        self.assertNotIn("signed-token", rendered)
        self.assertNotIn("operator", rendered)
        self.assertEqual(profile.redacted_snapshot()["mqtt"]["password"], "<redacted>")

    def test_secret_ref_with_both_sources_is_rejected(self) -> None:
        raw = self._raw()
        raw["mqtt"]["password"] = {
            "env": "FAB_MQTT_PASSWORD",
            "file": "password.txt",
        }

        with self.assertRaisesRegex(DeploymentConfigError, "secret.source"):
            DeploymentProfile.load(self._write(raw), environ=self._environment())

    def test_cli_returns_two_for_incomplete_committed_example(self) -> None:
        result = subprocess.run(
            [
                str(ROOT / ".venv/bin/python"),
                str(ROOT / "scripts/validate_production_deployment.py"),
                str(ROOT / "config/production.connected-corridor.example.yaml"),
            ],
            cwd=ROOT,
            env={key: value for key, value in os.environ.items() if not key.startswith("FAB_")},
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("ERROR mqtt.host.placeholder", result.stderr)
        self.assertNotIn("Traceback", result.stderr)


if __name__ == "__main__":
    unittest.main()
