from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_connected_corridor_fault_scenario.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("fault_runner", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def status(
    *,
    current_block: str | None,
    eligible: bool,
    reasons: list[str],
    robot_faulted: bool = False,
    block_fault: str | None = None,
    authority: bool = False,
    ready: bool = True,
    readiness_reason: str = "ready",
) -> dict:
    return {
        "readiness": {
            "ready": ready,
            "reason": readiness_reason,
            "recovery_required": readiness_reason == "recovery.required",
        },
        "robots": {
            "R1": {
                "current_block": current_block,
                "current_hb": "SIDE" if current_block is None else None,
                "faulted": robot_faulted,
                "eligibility": {"eligible": eligible, "reasons": reasons},
            }
        },
        "jobs": {
            "job": {"robot_id": "R1", "status": "WAITING"}
        },
        "arbiter": {
            "blocks": {
                "C2": {
                    "state": "BLOCKED" if block_fault else "FREE",
                    "fault_reason": block_fault,
                    "occupants": ["R1"] if current_block else [],
                }
            },
            "authorities": {"R1": {"blocks": ["C2"]}} if authority else {},
            "pending_authorities": {},
        },
    }


class ConnectedCorridorFaultRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()

    def test_inside_estop_requires_robot_and_block_fault(self) -> None:
        verifier = self.runner.FaultScenarioVerifier("inside_estop", "R1")
        snapshot = status(
            current_block="C2",
            eligible=False,
            reasons=["safety.estop"],
            robot_faulted=True,
            block_fault="safety.estop",
            authority=True,
        )

        verifier.observe(snapshot, log_segment="ROBOT_FAULT\nBLOCK_FAULT", at=2.0)

        self.assertTrue(verifier.complete)
        self.assertEqual(verifier.violations, ())

    def test_state_loss_inside_corridor_locks_authority(self) -> None:
        verifier = self.runner.FaultScenarioVerifier("inside_state_loss", "R1")
        snapshot = status(
            current_block="C2",
            eligible=False,
            reasons=["state.stale"],
            robot_faulted=True,
            block_fault="telemetry_timeout",
            authority=True,
        )

        verifier.observe(snapshot, log_segment="BLOCK_FAULT", at=6.0)

        self.assertTrue(verifier.complete)
        self.assertFalse(verifier.evidence["opposite_admitted"])

    def test_manual_at_safe_stop_blocks_only_robot_without_block_fault(self) -> None:
        verifier = self.runner.FaultScenarioVerifier("safe_stop_manual", "R1")
        snapshot = status(
            current_block=None,
            eligible=False,
            reasons=["mode.not_automatic"],
        )

        verifier.observe(snapshot, log_segment="", at=3.0)

        self.assertTrue(verifier.complete)
        self.assertEqual(verifier.violations, ())
        self.assertFalse(verifier.evidence["block_fault_seen"])

    def test_ineligible_robot_at_safe_stop_must_not_receive_authority(self) -> None:
        verifier = self.runner.FaultScenarioVerifier("safe_stop_manual", "R1")
        snapshot = status(
            current_block=None,
            eligible=False,
            reasons=["mode.not_automatic"],
            authority=True,
        )

        verifier.observe(snapshot, at=3.0)

        self.assertIn("ineligible_robot_received_authority", verifier.violations)

    def test_restart_after_inside_observation_remains_recovery_required(self) -> None:
        verifier = self.runner.FaultScenarioVerifier("restart_inside", "R1")
        inside = status(
            current_block="C2",
            eligible=True,
            reasons=[],
            ready=False,
            readiness_reason="recovery.required",
        )
        later_safe = status(
            current_block=None,
            eligible=True,
            reasons=[],
            ready=False,
            readiness_reason="recovery.required",
        )

        verifier.observe(inside, at=1.0)
        verifier.observe(later_safe, at=2.0)

        self.assertTrue(verifier.complete)
        self.assertTrue(verifier.evidence["recovery_sticky"])

    def test_runtime_start_supports_opt_in_profile_and_fault_rules(self) -> None:
        start = (ROOT / "scripts/start_p4_passing_bay.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("PASSING_BAY_DEPLOYMENT_PROFILE", start)
        self.assertIn("PASSING_BAY_FAULT_SCENARIOS", start)
        self.assertIn("--fault-scenarios", start)
        self.assertIn("--deployment-profile", start)

        profile = ROOT / "config/simulation.connected-corridor-readiness.yaml"
        self.assertTrue(profile.is_file())
        rendered = profile.read_text(encoding="utf-8")
        self.assertIn("operational_checks_required: true", rendered)


if __name__ == "__main__":
    unittest.main()
