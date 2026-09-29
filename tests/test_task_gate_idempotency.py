from __future__ import annotations

import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.task_gate import TaskGate


class DummyArbiter:
    pass


class DummyTracker:
    pass


class TaskGateIdempotencyTests(unittest.TestCase):
    def test_same_key_replays_without_forwarding_twice(self) -> None:
        registry = CorridorRegistry.from_dict(
            {"traffic_control": {"enabled": False}}
        )
        calls: list[dict] = []

        def forwarder(payload: dict) -> dict:
            calls.append(payload)
            return {"success": True, "count": len(calls)}

        gate = TaskGate(
            registry,
            DummyArbiter(),
            DummyTracker(),
            forwarder,
        )
        payload = {
            "type": "robot_task_request",
            "robot": "SIM_A",
            "request": {"description": {"places": ["B"]}},
        }

        first = gate.submit(payload, idempotency_key="request-123")
        second = gate.submit(payload, idempotency_key="request-123")

        self.assertEqual(len(calls), 1)
        self.assertEqual(first["decision"], "BYPASS")
        self.assertTrue(second["idempotent_replay"])
        self.assertEqual(second["upstream"], first["upstream"])

    def test_same_key_with_different_payload_is_rejected(self) -> None:
        registry = CorridorRegistry.from_dict(
            {"traffic_control": {"enabled": False}}
        )

        gate = TaskGate(
            registry,
            DummyArbiter(),
            DummyTracker(),
            lambda payload: {"success": True},
        )

        gate.submit(
            {"robot": "SIM_A", "request": {"description": {"places": ["B"]}}},
            idempotency_key="request-123",
        )

        with self.assertRaisesRegex(ValueError, "different payload"):
            gate.submit(
                {
                    "robot": "SIM_A",
                    "request": {"description": {"places": ["C"]}},
                },
                idempotency_key="request-123",
            )


if __name__ == "__main__":
    unittest.main()
