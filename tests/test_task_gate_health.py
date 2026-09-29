from __future__ import annotations

from types import SimpleNamespace
import unittest

from fastapi import HTTPException

from traffic_control.task_gate import TaskGate, create_app

from tests.test_movement_authority import tracking_components


def endpoint(app, path: str):
    return next(route.endpoint for route in app.routes if route.path == path)


class TaskGateHealthTests(unittest.TestCase):
    def make_gate(self, *, ready: bool):
        registry, arbiter, _, tracker = tracking_components()
        readiness = SimpleNamespace(
            health=lambda: {"healthy": True},
            ready=lambda: {
                "ready": ready,
                "reason": "ready" if ready else "mqtt.disconnected",
            },
            can_accept_tasks=lambda: ready,
            profile=SimpleNamespace(
                redacted_snapshot=lambda: {"mode": "production"}
            ),
        )
        return TaskGate(
            registry,
            arbiter,
            tracker,
            lambda request: {"success": True},
            readiness=readiness,
        )

    def test_health_endpoint_reports_process_liveness(self) -> None:
        app = create_app(self.make_gate(ready=False))

        self.assertEqual(endpoint(app, "/health")(), {"healthy": True})

    def test_ready_endpoint_returns_503_until_motion_is_safe(self) -> None:
        app = create_app(self.make_gate(ready=False))

        with self.assertRaises(HTTPException) as context:
            endpoint(app, "/ready")()

        self.assertEqual(context.exception.status_code, 503)
        self.assertEqual(context.exception.detail["reason"], "mqtt.disconnected")

    def test_ready_and_status_are_redacted_when_runtime_is_ready(self) -> None:
        gate = self.make_gate(ready=True)
        app = create_app(gate)

        self.assertTrue(endpoint(app, "/ready")()["ready"])
        status = gate.status()
        self.assertEqual(status["deployment"], {"mode": "production"})
        self.assertTrue(status["readiness"]["ready"])


if __name__ == "__main__":
    unittest.main()
