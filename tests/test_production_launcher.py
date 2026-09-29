from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
START = ROOT / "scripts/start_production_corridor.sh"
STOP = ROOT / "scripts/stop_production_corridor.sh"
RUN_ARBITER = ROOT / "scripts/run_direction_arbiter.sh"


class ProductionLauncherTests(unittest.TestCase):
    def test_start_validates_and_prepares_runtime_before_launching(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertLess(
            text.index("validate_production_deployment.py"),
            text.index("docker compose"),
        )
        self.assertIn("prepare_production_runtime.py", text)
        self.assertIn("RMF_FIELD_MQTT_USERNAME", text)
        self.assertIn("production.compose.override.yml", text)
        self.assertIn("Path(sys.argv[1]).expanduser().resolve()", text)
        self.assertIn("--deployment-profile", text)

    def test_start_refuses_duplicate_runtime_and_cleans_up_failed_start(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertIn("production_arbiter.pid", text)
        self.assertIn("already running", text)
        self.assertIn("cleanup_failed_start", text)
        self.assertIn("Successfully added robot", text)
        self.assertIn("rmf_api_server rmf_traffic_blockade", text)

    def test_start_changes_to_repository_before_inline_python_imports(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertLess(
            text.index('cd "${workspace_dir}"'),
            text.index("from traffic_control.deployment import DeploymentProfile"),
        )

    def test_start_uses_configurable_health_and_readiness_timeouts(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertIn("PRODUCTION_HEALTH_TIMEOUT_SECONDS", text)
        self.assertIn("PRODUCTION_READY_TIMEOUT_SECONDS", text)
        self.assertNotIn("for _ in $(seq 1 120)", text)

    def test_start_never_launches_simulator_or_generates_a_development_jwt(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertNotIn("vda5050_robot_simulator", text)
        self.assertNotIn("visualizer", text.lower())
        self.assertNotIn("jwt.encode", text)

    def test_stop_is_scoped_to_recorded_pid_and_profile_compose_files(self) -> None:
        text = STOP.read_text(encoding="utf-8")

        self.assertIn("production_arbiter.pid", text)
        self.assertIn("docker compose", text)
        self.assertIn("/proc/${pid}/cmdline", text)
        self.assertIn('"${profile}"', text)
        self.assertIn('"${workspace_dir}"', text)
        self.assertNotIn("pkill", text)
        self.assertNotIn("killall", text)

    def test_arbiter_runner_accepts_optional_deployment_profile(self) -> None:
        text = RUN_ARBITER.read_text(encoding="utf-8")

        self.assertIn("--deployment-profile", text)


if __name__ == "__main__":
    unittest.main()
