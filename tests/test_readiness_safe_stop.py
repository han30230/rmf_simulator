from __future__ import annotations

from types import SimpleNamespace
import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.eligibility import EligibilityResult
from traffic_control.readiness import RuntimeReadiness
from traffic_control.robot_tracker import RobotTracker


def registry() -> CorridorRegistry:
    return CorridorRegistry.from_dict(
        {
            "traffic_control": {"enabled": True},
            "holding_bays": {
                "HB_A": {
                    "node_id": "A",
                    "geometry": {"circle": {"x": 0.0, "y": 0.0, "radius": 0.5}},
                },
                "HB_B": {
                    "node_id": "B",
                    "geometry": {"circle": {"x": 5.0, "y": 0.0, "radius": 0.5}},
                },
            },
            "blocks": [
                {
                    "id": "C1",
                    "entry_a": "HB_A",
                    "entry_b": "HB_B",
                    "geometry": {
                        "bounds": {
                            "min_x": 0.6,
                            "max_x": 4.4,
                            "min_y": -0.5,
                            "max_y": 0.5,
                        }
                    },
                    "edges_a_to_b": ["A>B"],
                    "edges_b_to_a": ["B>A"],
                }
            ],
        }
    )


def payload(last_node: str) -> dict:
    return {
        "headerId": 1,
        "timestamp": "2026-09-22T00:00:01Z",
        "manufacturer": "TEST",
        "serialNumber": "SIM_A",
        "lastNodeId": last_node,
        "driving": False,
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


class ToggleEligibilityTracker:
    def __init__(self, inner: RobotTracker) -> None:
        self.inner = inner
        self.allowed = True

    def telemetry(self, robot_id: str):
        return self.inner.telemetry(robot_id)

    def current_safe_node(self, robot_id: str):
        return self.inner.current_safe_node(robot_id)

    def eligibility(self, robot_id: str, *, now: float | None = None):
        if self.allowed:
            return EligibilityResult(True, ())
        return EligibilityResult(False, ("state.stale",))


class ReadinessSafeStopTests(unittest.TestCase):
    def _readiness(self, tracker: RobotTracker, reg: CorridorRegistry):
        profile = SimpleNamespace(
            validate=lambda: (),
            robots={"SIM_A": SimpleNamespace(required=True)},
        )
        return RuntimeReadiness(
            profile,
            reg,
            tracker,
            mqtt_connected=lambda: True,
            rmf_probe=lambda: True,
        )

    def test_geometry_only_holding_bay_is_not_clean_start(self) -> None:
        reg = registry()
        arbiter = DirectionArbiter(reg)
        tracker = RobotTracker(reg, arbiter)
        tracker.ingest_state("SIM_A", payload("UNKNOWN"))

        readiness = self._readiness(tracker, reg)
        result = readiness.ready()

        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "safe_stop.unconfirmed")
        self.assertFalse(result["recovery_required"])

        tracker.ingest_state("SIM_A", {
            **payload("A"),
            "headerId": 2,
            "timestamp": "2026-09-22T00:00:02Z",
        })
        recovered = readiness.ready()
        self.assertTrue(recovered["ready"])

    def test_readiness_rechecks_operational_eligibility_after_clean_start(self) -> None:
        reg = registry()
        arbiter = DirectionArbiter(reg)
        inner = RobotTracker(reg, arbiter)
        inner.ingest_state("SIM_A", payload("A"))
        tracker = ToggleEligibilityTracker(inner)

        readiness = self._readiness(tracker, reg)
        self.assertTrue(readiness.ready()["ready"])

        tracker.allowed = False
        result = readiness.ready()

        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "robots.ineligible")
        self.assertIn("SIM_A:state.stale", result["details"])

    def test_matching_last_node_is_clean_start(self) -> None:
        reg = registry()
        arbiter = DirectionArbiter(reg)
        tracker = RobotTracker(reg, arbiter)
        tracker.ingest_state("SIM_A", payload("A"))

        result = self._readiness(tracker, reg).ready()

        self.assertTrue(result["ready"])
        self.assertEqual(result["reason"], "ready")


if __name__ == "__main__":
    unittest.main()
