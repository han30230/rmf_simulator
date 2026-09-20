from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
START = ROOT / "scripts/start_production_corridor.sh"
STOP = ROOT / "scripts/stop_production_corridor.sh"
RUN_ARBITER = ROOT / "scripts/run_direction_arbiter.sh"


class ProductionLauncherTests(unittest.TestCase):
    def test_start_validates_before_launching_and_requires_external_token(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertLess(
            text.index("validate_production_deployment.py"),
            text.index("docker compose"),
        )
        self.assertIn("RMF_API_BEARER_TOKEN", text)
        self.assertIn("--deployment-profile", text)

    def test_start_never_launches_simulator_or_generates_a_development_jwt(self) -> None:
        text = START.read_text(encoding="utf-8")

        self.assertNotIn("vda5050_robot_simulator", text)
        self.assertNotIn("visualizer", text.lower())
        self.assertNotIn("jwt.encode", text)

    def test_stop_is_scoped_to_recorded_pid_and_profile_compose_files(self) -> None:
        text = STOP.read_text(encoding="utf-8")

        self.assertIn("production_arbiter.pid", text)
        self.assertIn("docker compose", text)
        self.assertNotIn("pkill", text)
        self.assertNotIn("killall", text)

    def test_arbiter_runner_accepts_optional_deployment_profile(self) -> None:
        text = RUN_ARBITER.read_text(encoding="utf-8")

        self.assertIn("--deployment-profile", text)


if __name__ == "__main__":
    unittest.main()
