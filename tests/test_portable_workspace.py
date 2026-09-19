from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class PortableWorkspaceTests(unittest.TestCase):
    def test_root_requirements_cover_all_python_entrypoints(self) -> None:
        requirements = (ROOT / "requirements.txt").read_text(encoding="utf-8").lower()

        for dependency in ("fastapi", "uvicorn", "paho-mqtt", "pyyaml", "pyqt5"):
            self.assertIn(dependency, requirements)

    def test_portable_runtime_scripts_are_repository_relative(self) -> None:
        for name in (
            "setup_workspace.sh",
            "start_p4_passing_bay.sh",
            "stop_p4_passing_bay.sh",
        ):
            path = ROOT / "scripts" / name
            self.assertTrue(path.is_file(), name)
            content = path.read_text(encoding="utf-8")
            self.assertIn("BASH_SOURCE", content)
            self.assertNotIn("/home/han30230", content)
            self.assertNotIn("rmf_simulation_workspace/.venv", content)

    def test_start_script_selects_the_complete_passing_bay_stack(self) -> None:
        script = (ROOT / "scripts/start_p4_passing_bay.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("docker-compose.p4-passing-bay.yml", script)
        self.assertIn("docker-compose.portable.yml", script)
        self.assertIn("corridor_blocks_p4_passing_bay.yaml", script)
        self.assertIn("RMF_API_BEARER_TOKEN", script)
        self.assertIn("p4_passing_bay_runtime.yaml", script)

    def test_start_script_accepts_config_driven_runtime_files(self) -> None:
        script = (ROOT / "scripts/start_p4_passing_bay.sh").read_text(
            encoding="utf-8"
        )

        for name in (
            "PASSING_BAY_SIMULATOR_SCENARIO",
            "PASSING_BAY_COMPOSE_FILE",
            "PASSING_BAY_ARBITER_CONFIG",
            "PASSING_BAY_RUNTIME_NAME",
        ):
            self.assertIn(name, script)
        self.assertIn("p4_scenario.yaml", script)
        self.assertIn("docker-compose.p4-passing-bay.yml", script)
        self.assertIn("corridor_blocks_p4_passing_bay.yaml", script)

    def test_portable_compose_provides_an_mqtt_broker(self) -> None:
        compose = (
            ROOT / "rmf_platform-main" / "docker-compose.portable.yml"
        ).read_text(encoding="utf-8")
        broker_config = (
            ROOT / "rmf_platform-main" / "mosquitto.conf"
        ).read_text(encoding="utf-8")

        self.assertIn("mqtt_broker:", compose)
        self.assertIn("eclipse-mosquitto", compose)
        self.assertIn("listener 1883", broker_config)
        self.assertIn("allow_anonymous true", broker_config)

    def test_quickstart_describes_nonstop_release_at_2106(self) -> None:
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        run_guide = (ROOT / "docs" / "simulation_run_guide.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("setup_workspace.sh", readme)
        self.assertIn("start_p4_passing_bay.sh", readme)
        self.assertIn("2106", readme)
        self.assertIn("정지하지", readme)
        self.assertIn("start_p4_passing_bay.sh", run_guide)
        self.assertNotIn("/mnt/d/Documents", run_guide)

    def test_docs_describe_distinct_staging_slots_and_one_against_three(self) -> None:
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        run_guide = (ROOT / "docs" / "simulation_run_guide.md").read_text(
            encoding="utf-8"
        )
        for content in (readme, run_guide):
            self.assertIn("start_p4_passing_bay_staging_2v2.sh", content)
            self.assertIn("t4_dispatch_passing_bay_staging_2v2.sh", content)
            self.assertIn("start_p4_passing_bay_staging_1v3.sh", content)
            self.assertIn("t4_dispatch_passing_bay_staging_1v3.sh", content)
            self.assertIn("launch_p4_passing_bay_staging_visualizer.sh", content)
            self.assertIn("capacity 1", content)
            self.assertIn("측량", content)

    def test_rmf_core_dockerfile_has_no_site_specific_proxy(self) -> None:
        dockerfile = (
            ROOT / "rmf_platform-main" / "docker" / "Dockerfile"
        ).read_text(encoding="utf-8")

        self.assertNotIn("12.26.204.100", dockerfile)
        self.assertNotIn("McAfee_Certificate.crt", dockerfile)
        self.assertIn("ghcr.io/open-rmf/rmf/rmf_demos", dockerfile)


if __name__ == "__main__":
    unittest.main()
