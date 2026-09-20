from __future__ import annotations

from types import SimpleNamespace
import unittest

from traffic_control.models import Direction
from traffic_control.readiness import RuntimeReadiness

from tests.test_movement_authority import state, tracking_components


def profile(*robot_ids: str):
    return SimpleNamespace(
        mode="production",
        robots={
            robot_id: SimpleNamespace(required=True)
            for robot_id in robot_ids
        },
        validate=lambda: (),
        redacted_snapshot=lambda: {"mode": "production"},
    )


class RuntimeReadinessTests(unittest.TestCase):
    def test_clean_start_waits_for_all_required_robots_at_safe_stops(self) -> None:
        registry, arbiter, _, tracker = tracking_components()
        readiness = RuntimeReadiness(
            profile("A1"),
            registry,
            tracker,
            mqtt_connected=lambda: True,
            rmf_probe=lambda: True,
        )

        self.assertEqual(readiness.ready(now=1.0)["reason"], "telemetry.pending")

        tracker.ingest_state(
            "A1", state("L1", -1.0, 1.0, driving=False), received_at=2.0
        )

        result = readiness.ready(now=2.0)
        self.assertTrue(result["ready"])
        self.assertEqual(result["reason"], "ready")
        self.assertFalse(result["recovery_required"])

    def test_robot_inside_during_startup_requires_operator_recovery(self) -> None:
        registry, arbiter, planner, tracker = tracking_components()
        readiness = RuntimeReadiness(
            profile("A1"),
            registry,
            tracker,
            mqtt_connected=lambda: True,
            rmf_probe=lambda: True,
        )
        path = registry.resolve_chain_path("L1", "R1")
        assert path is not None
        plan = planner.plan(path, lambda *_: True)
        assert plan is not None
        arbiter.request_authority(plan, robot_id="A1")
        tracker.ingest_state(
            "A1", state("N1", 5.0, 1.0, driving=True), received_at=1.0
        )

        first = readiness.ready(now=1.0)
        self.assertFalse(first["ready"])
        self.assertEqual(first["reason"], "recovery.required")

        arbiter.mark_authority_arrived("A1")
        tracker.ingest_state(
            "A1", state("L1", -1.0, 1.0, driving=False), received_at=2.0
        )
        second = readiness.ready(now=2.0)
        self.assertFalse(second["ready"])
        self.assertTrue(second["recovery_required"])

    def test_dependency_failures_keep_runtime_not_ready(self) -> None:
        registry, _, _, tracker = tracking_components()
        tracker.ingest_state(
            "A1", state("L1", -1.0, 1.0, driving=False), received_at=1.0
        )
        mqtt_down = RuntimeReadiness(
            profile("A1"), registry, tracker,
            mqtt_connected=lambda: False, rmf_probe=lambda: True,
        )
        rmf_down = RuntimeReadiness(
            profile("A1"), registry, tracker,
            mqtt_connected=lambda: True, rmf_probe=lambda: False,
        )

        self.assertEqual(mqtt_down.ready(now=1.0)["reason"], "mqtt.disconnected")
        self.assertEqual(rmf_down.ready(now=1.0)["reason"], "rmf.unavailable")

    def test_health_is_independent_from_dependency_readiness(self) -> None:
        registry, _, _, tracker = tracking_components()
        readiness = RuntimeReadiness(
            profile("A1"), registry, tracker,
            mqtt_connected=lambda: False, rmf_probe=lambda: False,
        )

        health = readiness.health()

        self.assertTrue(health["healthy"])
        self.assertFalse(health["dependencies"]["mqtt"])
        self.assertFalse(health["dependencies"]["rmf_api"])


if __name__ == "__main__":
    unittest.main()
