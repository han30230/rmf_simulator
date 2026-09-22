from __future__ import annotations

from types import SimpleNamespace
import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.readiness import RuntimeReadiness
from traffic_control.robot_tracker import RobotTracker
from traffic_control.task_gate import TaskGate


def make_registry() -> CorridorRegistry:
    return CorridorRegistry.from_dict(
        {
            "traffic_control": {"enabled": True},
            "holding_bays": {
                "HB_A": {
                    "node_id": "A",
                    "geometry": {
                        "circle": {"x": 0.0, "y": 0.0, "radius": 0.5}
                    },
                },
                "HB_B": {
                    "node_id": "B",
                    "geometry": {
                        "circle": {"x": 10.0, "y": 0.0, "radius": 0.5}
                    },
                },
            },
            "blocks": [
                {
                    "id": "C1",
                    "entry_a": "HB_A",
                    "entry_b": "HB_B",
                    "geometry": {
                        "bounds": {
                            "min_x": 1.0,
                            "max_x": 9.0,
                            "min_y": -1.0,
                            "max_y": 1.0,
                        }
                    },
                    "edges_a_to_b": ["A>B"],
                    "edges_b_to_a": ["B>A"],
                }
            ],
        }
    )


def payload(*, driving: bool) -> dict:
    return {
        "headerId": 1,
        "timestamp": "2026-09-22T00:00:00Z",
        "manufacturer": "YujinRobot",
        "serialNumber": "R1",
        "lastNodeId": "A",
        "driving": driving,
        "operatingMode": "AUTOMATIC",
        "paused": False,
        "agvPosition": {
            "x": 0.0,
            "y": 0.0,
            "mapId": "L1",
            "positionInitialized": True,
        },
        "safetyState": {"eStop": "NONE", "fieldViolation": False},
        "nodeStates": [],
        "edgeStates": [],
        "errors": [],
    }


class FakeProfile:
    mode = "lab"
    robots = {"R1": SimpleNamespace(required=True)}

    @staticmethod
    def validate():
        return ()


class RecoveryAcknowledgementTests(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = make_registry()
        self.arbiter = DirectionArbiter(self.registry)
        self.tracker = RobotTracker(self.registry, self.arbiter)
        self.readiness = RuntimeReadiness(
            FakeProfile(),
            self.registry,
            self.tracker,
            mqtt_connected=lambda: True,
            rmf_probe=lambda: True,
        )
        self.gate = TaskGate(
            self.registry,
            self.arbiter,
            self.tracker,
            lambda task: {"success": True},
            readiness=self.readiness,
        )

    def test_recovery_ack_clears_latches_only_at_safe_stop(self) -> None:
        self.tracker.ingest_state("R1", payload(driving=False))
        self.tracker._robots["R1"].faulted = True
        self.registry.blocks["C1"].fault_reason = "telemetry_timeout"
        self.readiness._recovery_required = True

        result = self.gate.acknowledge_recovery()

        self.assertTrue(result["recovered"])
        self.assertFalse(self.tracker.telemetry("R1").faulted)
        self.assertIsNone(self.registry.blocks["C1"].fault_reason)
        self.assertTrue(self.readiness.ready()["ready"])

    def test_recovery_ack_rejects_moving_robot(self) -> None:
        self.tracker.ingest_state("R1", payload(driving=True))
        self.readiness._recovery_required = True

        result = self.gate.acknowledge_recovery()

        self.assertFalse(result["recovered"])
        self.assertEqual(result["reason"], "recovery_not_safe")
        self.assertEqual(
            result["readiness"]["reason"],
            "recovery.robot_not_safe",
        )


if __name__ == "__main__":
    unittest.main()
