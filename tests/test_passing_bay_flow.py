from __future__ import annotations

from pathlib import Path
import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import Direction
from traffic_control.robot_tracker import RobotTracker
from traffic_control.task_gate import TaskGate


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/corridor_blocks_p4_passing_bay.yaml"


POSITIONS = {
    "2101": (6.833, 92.871),
    "2104": (30.424, 92.871),
    "2105": (42.508, 92.871),
    "2106": (54.818, 92.871),
    "2107": (66.612, 92.871),
    "6137": (42.508, 91.100),
    "2108": (73.7274, 92.8248),
}


def payload(robot_id: str, goal_node: str) -> dict:
    return {
        "type": "robot_task_request",
        "robot": robot_id,
        "fleet": "TOOL",
        "request": {
            "unix_millis_earliest_start_time": 0,
            "category": "patrol",
            "priority": {"type": "default", "value": 0},
            "description": {"places": [goal_node], "rounds": 1},
        },
    }


def state(
    x: float,
    y: float,
    *,
    node: str = "",
    driving: bool = True,
) -> dict:
    return {
        "lastNodeId": node,
        "driving": driving,
        "agvPosition": {"x": x, "y": y, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [],
    }


class RecordingForwarder:
    def __init__(self) -> None:
        self.goals: list[tuple[str, str]] = []

    def __call__(self, submitted: dict) -> dict:
        robot_id = str(submitted["robot"])
        goal = str(submitted["request"]["description"]["places"][0])
        self.goals.append((robot_id, goal))
        return {"success": True, "task_id": f"upstream-{len(self.goals)}"}


class PassingBayFlowTests(unittest.TestCase):
    def test_west_approach_remains_in_its_granted_block_until_gate_bay(self) -> None:
        registry = CorridorRegistry.from_yaml(CONFIG)
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)
        gate = TaskGate(registry, arbiter, tracker, RecordingForwarder())

        tracker.ingest_state(
            "AGV_A1",
            state(*POSITIONS["2101"], node="2101", driving=False),
            received_at=1.0,
        )
        result = gate.submit(payload("AGV_A1", "2108"))
        self.assertEqual(result["decision"], "ADMIT")

        tracker.ingest_state("AGV_A1", state(18.46, 92.871), received_at=2.0)
        # This point is immediately before the 2104 holding-bay circle. It must
        # remain part of the west approach instead of matching an overlapping
        # passing-event block and triggering fail-closed.
        tracker.ingest_state("AGV_A1", state(29.75, 92.871), received_at=3.0)

        status = gate.status()
        self.assertEqual(
            status["robots"]["AGV_A1"]["current_block"],
            "P4_WEST_ADVANCE",
        )
        self.assertIsNone(
            status["arbiter"]["blocks"]["P4_EAST_TO_SIDE"]["fault_reason"]
        )

    def test_gate_departure_gap_prefers_the_active_gate_to_right_grant(self) -> None:
        registry = CorridorRegistry.from_yaml(CONFIG)
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)

        tracker.ingest_state(
            "AGV_A1",
            state(*POSITIONS["2104"], node="2104", driving=False),
            received_at=1.0,
        )
        decision = arbiter.request(
            "AGV_A1",
            "P4_GATE_TO_RIGHT",
            Direction.A_TO_B,
            "HB_RIGHT",
            source_hb="HB_WEST_GATE",
            release_node="2106",
        )
        self.assertEqual(decision.value, "ADMIT")

        # HB_WEST_GATE ends at x=31.024. This point is just beyond that bay,
        # where the active P4_GATE_TO_RIGHT grant must win instead of the
        # broader, opposing P4_SIDE_TO_LEFT geometry.
        tracker.ingest_state(
            "AGV_A1",
            state(31.05, 92.871, node="2104", driving=True),
            received_at=2.0,
        )

        status = arbiter.snapshot()
        self.assertEqual(
            tracker.snapshot()["AGV_A1"]["current_block"],
            "P4_GATE_TO_RIGHT",
        )
        self.assertEqual(
            status["blocks"]["P4_GATE_TO_RIGHT"]["occupants"],
            ["AGV_A1"],
        )
        self.assertIsNone(
            status["blocks"]["P4_SIDE_TO_LEFT"]["fault_reason"]
        )

    def test_b1_yields_in_side_bay_then_rejoins_after_a1_crosses(self) -> None:
        registry = CorridorRegistry.from_yaml(CONFIG)
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)
        forwarder = RecordingForwarder()
        gate = TaskGate(registry, arbiter, tracker, forwarder)

        tracker.ingest_state(
            "AGV_A1", state(*POSITIONS["2101"], node="2101", driving=False), received_at=1.0
        )
        tracker.ingest_state(
            "AGV_B1", state(*POSITIONS["2108"], node="2108", driving=False), received_at=1.0
        )

        a1 = gate.submit(payload("AGV_A1", "2108"))
        b1 = gate.submit(payload("AGV_B1", "2101"))
        self.assertEqual(a1["decision"], "ADMIT")
        self.assertEqual(b1["decision"], "ADMIT")
        self.assertEqual(
            forwarder.goals,
            [("AGV_A1", "2104"), ("AGV_B1", "6137")],
        )

        tracker.ingest_state("AGV_A1", state(18.46, 92.871), received_at=2.0)
        tracker.ingest_state(
            "AGV_A1", state(*POSITIONS["2104"], node="2104", driving=False), received_at=3.0
        )
        gate.tick(now=3.0)
        self.assertEqual(gate.status()["jobs"][a1["job_id"]]["status"], "WAITING")
        self.assertEqual(len(forwarder.goals), 2)

        tracker.ingest_state(
            "AGV_B1",
            state(*POSITIONS["2106"], node="2106", driving=True),
            received_at=3.1,
        )
        tracker.ingest_state(
            "AGV_B1", state(*POSITIONS["6137"], node="6137", driving=False), received_at=4.0
        )
        gate.tick(now=4.0)

        status = gate.status()
        self.assertEqual(status["robots"]["AGV_B1"]["current_hb"], "HB_MIDDLE_SIDE")
        self.assertEqual(status["jobs"][b1["job_id"]]["status"], "WAITING")
        self.assertEqual(
            forwarder.goals,
            [
                ("AGV_A1", "2104"),
                ("AGV_B1", "6137"),
                ("AGV_A1", "2108"),
            ],
        )

        tracker.ingest_state(
            "AGV_A1",
            state(*POSITIONS["2105"], node="2105", driving=True),
            received_at=5.0,
        )
        gate.tick(now=5.0)
        self.assertEqual(forwarder.goals[-1], ("AGV_A1", "2108"))
        self.assertEqual(gate.status()["jobs"][b1["job_id"]]["status"], "WAITING")

        tracker.ingest_state(
            "AGV_A1",
            state(*POSITIONS["2106"], node="2106", driving=True),
            received_at=5.5,
        )
        gate.tick(now=5.5)
        self.assertEqual(
            forwarder.goals,
            [
                ("AGV_A1", "2104"),
                ("AGV_B1", "6137"),
                ("AGV_A1", "2108"),
                ("AGV_B1", "2101"),
            ],
        )

        handoff = gate.status()
        self.assertEqual(handoff["jobs"][a1["job_id"]]["status"], "ACTIVE")
        self.assertEqual(handoff["jobs"][b1["job_id"]]["status"], "ACTIVE")
        self.assertTrue(handoff["robots"]["AGV_A1"]["driving"])
        self.assertIsNone(handoff["robots"]["AGV_A1"]["current_block"])
        self.assertIn(
            "AGV_A1",
            handoff["arbiter"]["holding_bays"]["HB_RIGHT"]["reservations"],
        )

        tracker.ingest_state(
            "AGV_A1",
            state(*POSITIONS["2107"], node="2107", driving=True),
            received_at=6.0,
        )
        tracker.ingest_state(
            "AGV_B1",
            state(*POSITIONS["2105"], node="2105", driving=True),
            received_at=6.1,
        )
        self.assertEqual(
            tracker.snapshot()["AGV_B1"]["current_block"],
            "P4_SIDE_TO_LEFT",
        )

        tracker.ingest_state(
            "AGV_A1",
            state(*POSITIONS["2108"], node="2108", driving=False),
            received_at=7.0,
        )
        gate.tick(now=7.0)
        self.assertEqual(gate.status()["jobs"][a1["job_id"]]["status"], "COMPLETE")

        tracker.ingest_state(
            "AGV_B1",
            state(*POSITIONS["2104"], node="2104", driving=True),
            received_at=7.5,
        )
        self.assertEqual(
            tracker.snapshot()["AGV_B1"]["current_block"],
            "P4_SIDE_TO_LEFT",
        )
        tracker.ingest_state("AGV_B1", state(25.0, 92.871), received_at=8.0)
        tracker.ingest_state(
            "AGV_B1",
            state(*POSITIONS["2101"], node="2101", driving=False),
            received_at=8.5,
        )
        gate.tick(now=8.5)

        final = gate.status()
        self.assertEqual(final["jobs"][a1["job_id"]]["status"], "COMPLETE")
        self.assertEqual(final["jobs"][b1["job_id"]]["status"], "COMPLETE")
        for block in final["arbiter"]["blocks"].values():
            self.assertEqual(block["state"], "FREE")
            self.assertEqual(block["occupants"], [])
            self.assertEqual(block["reservations"], [])
            self.assertEqual(block["waiting"]["A_TO_B"], [])
            self.assertEqual(block["waiting"]["B_TO_A"], [])
            self.assertIsNone(block["fault_reason"])


if __name__ == "__main__":
    unittest.main()
