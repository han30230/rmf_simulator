from __future__ import annotations

from dataclasses import replace
import unittest

from traffic_control.deployment import RobotDeploymentConfig
from traffic_control.eligibility import RobotEligibilityPolicy
from traffic_control.robot_tracker import RobotTelemetry


def healthy_telemetry() -> RobotTelemetry:
    return RobotTelemetry(
        robot_id="R1",
        received_at=95.0,
        x=0.0,
        y=0.0,
        current_hb="LEFT",
        state_header_id=10,
        state_timestamp="2026-09-20T00:00:10Z",
        connection_received_at=96.0,
        connection_header_id=3,
        connection_timestamp="2026-09-20T00:00:09Z",
        connection_state="ONLINE",
        manufacturer="vendor",
        serial_number="R1",
        position_initialized=True,
        map_id="FAB_L1",
        operating_mode="AUTOMATIC",
        e_stop="NONE",
        field_violation=False,
        paused=False,
        error_levels=(),
    )


class RobotEligibilityPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = RobotEligibilityPolicy(
            robots={
                "R1": RobotDeploymentConfig(
                    manufacturer="vendor",
                    serial_number="R1",
                    allowed_map_ids=("FAB_L1",),
                    required=True,
                )
            },
            state_timeout=10.0,
            connection_timeout=10.0,
            blocking_error_levels=("FATAL",),
        )

    def test_healthy_automatic_robot_at_managed_location_is_eligible(self) -> None:
        result = self.policy.evaluate(healthy_telemetry(), now=100.0)

        self.assertTrue(result.eligible)
        self.assertEqual(result.reasons, ())

    def test_operational_failures_have_stable_reasons(self) -> None:
        cases = {
            "offline": ({"connection_state": "OFFLINE"}, "connection.offline"),
            "stale_state": ({"received_at": 80.0}, "state.stale"),
            "estop": ({"e_stop": "AUTOACK"}, "safety.estop"),
            "field": ({"field_violation": True}, "safety.field_violation"),
            "manual": ({"operating_mode": "MANUAL"}, "mode.not_automatic"),
            "paused": ({"paused": True}, "state.paused"),
            "fatal": ({"error_levels": ("FATAL",)}, "errors.blocking"),
            "wrong_map": ({"map_id": "OTHER"}, "position.map_mismatch"),
            "uninitialized": (
                {"position_initialized": False}, "position.uninitialized"
            ),
            "unknown_area": (
                {"current_hb": None, "current_block": None},
                "position.outside_managed_area",
            ),
        }
        for name, (changes, reason) in cases.items():
            with self.subTest(name=name):
                result = self.policy.evaluate(
                    replace(healthy_telemetry(), **changes), now=100.0
                )
                self.assertFalse(result.eligible)
                self.assertIn(reason, result.reasons)

    def test_missing_and_unregistered_robots_are_ineligible(self) -> None:
        self.assertEqual(
            self.policy.evaluate(None, now=100.0).reasons,
            ("telemetry.missing",),
        )
        result = self.policy.evaluate(
            replace(healthy_telemetry(), robot_id="R2"), now=100.0
        )
        self.assertIn("robot.unregistered", result.reasons)

    def test_warning_is_not_blocking_when_policy_only_blocks_fatal(self) -> None:
        result = self.policy.evaluate(
            replace(healthy_telemetry(), error_levels=("WARNING",)), now=100.0
        )

        self.assertTrue(result.eligible)

    def test_retained_online_connection_does_not_require_heartbeats(self) -> None:
        telemetry = replace(
            healthy_telemetry(),
            received_at=999.0,
            connection_received_at=1.0,
        )

        result = self.policy.evaluate(telemetry, now=1000.0)

        self.assertTrue(result.eligible)
        self.assertNotIn("connection.stale", result.reasons)


if __name__ == "__main__":
    unittest.main()
