from __future__ import annotations

from pathlib import Path
import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.robot_tracker import RobotTracker
from traffic_control.task_gate import TaskGate


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/corridor_blocks_p4_passing_bay_staging.yaml"

POSITIONS = {
    "2104": (30.424, 92.871),
    "2105": (42.508, 92.871),
    "2106": (54.818, 92.871),
    "6137": (42.508, 91.100),
    "SIDE_APPROACH": (42.508, 91.8),
    "P4_LS1": (6.833, 96.0),
    "P4_LS2": (3.2, 95.0),
    "P4_LS3": (3.2, 90.7),
    "P4_RS1": (73.7274, 96.0),
    "P4_RS2": (77.3, 95.0),
    "P4_RS3": (77.3, 90.7),
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


def state(node: str, *, driving: bool = False) -> dict:
    x, y = POSITIONS[node]
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


def make_gate():
    registry = CorridorRegistry.from_yaml(CONFIG)
    arbiter = DirectionArbiter(registry)
    tracker = RobotTracker(registry, arbiter, telemetry_timeout=20.0)
    forwarder = RecordingForwarder()
    return registry, tracker, TaskGate(registry, arbiter, tracker, forwarder), forwarder


def ingest(tracker: RobotTracker, robot: str, node: str, now: float, *, driving=False):
    tracker.ingest_state(robot, state(node, driving=driving), received_at=now)


def assert_clean(test: unittest.TestCase, status: dict) -> None:
    for block in status["arbiter"]["blocks"].values():
        test.assertEqual(block["state"], "FREE")
        test.assertEqual(block["occupants"], [])
        test.assertEqual(block["reservations"], [])
        test.assertTrue(all(not queue for queue in block["waiting"].values()))
        test.assertIsNone(block["fault_reason"])
    for bay in status["arbiter"]["holding_bays"].values():
        test.assertEqual(bay["reservations"], [])


class PassingBayStagingFlowTests(unittest.TestCase):
    def test_trailing_robot_stays_in_slot_until_opposite_job_completes(self) -> None:
        _, tracker, gate, forwarder = make_gate()
        for robot, node in (
            ("eastbound", "P4_LS1"),
            ("westbound-leader", "P4_RS1"),
            ("westbound-trailer", "P4_RS2"),
        ):
            ingest(tracker, robot, node, 1.0)

        eastbound = gate.submit(payload("eastbound", "P4_RS1"))
        gate.submit(payload("westbound-leader", "P4_LS1"))
        trailer = gate.submit(payload("westbound-trailer", "P4_LS2"))

        ingest(tracker, "eastbound", "2104", 2.0)
        gate.tick(now=2.0)
        ingest(tracker, "westbound-leader", "6137", 3.0)
        gate.tick(now=3.0)
        gate.tick(now=3.1)
        ingest(tracker, "eastbound", "2106", 4.0, driving=True)
        gate.tick(now=4.0)
        ingest(tracker, "westbound-leader", "P4_LS1", 5.0)
        gate.tick(now=5.0)

        self.assertEqual(
            gate.status()["jobs"][eastbound["job_id"]]["status"],
            "ACTIVE",
        )
        self.assertEqual(
            gate.status()["jobs"][trailer["job_id"]]["status"],
            "WAITING",
        )
        self.assertEqual(
            gate.status()["robots"]["westbound-trailer"]["current_hb"],
            "HB_RIGHT_SLOT_2",
        )
        self.assertNotIn(("westbound-trailer", "6137"), forwarder.goals)

        ingest(tracker, "eastbound", "P4_RS1", 6.0)
        gate.tick(now=6.0)
        self.assertEqual(forwarder.goals[-1], ("westbound-trailer", "P4_LS2"))
        self.assertEqual(
            gate.status()["jobs"][trailer["job_id"]]["route_id"],
            "P4_RS2_TO_LS2_DIRECT_WHEN_CLEAR",
        )

    def test_side_bay_grant_covers_the_vertical_branch_from_2105(self) -> None:
        _, tracker, gate, _ = make_gate()
        ingest(tracker, "westbound", "P4_RS1", 1.0)
        result = gate.submit(payload("westbound", "P4_LS1"))
        self.assertEqual(result["decision"], "ADMIT")

        ingest(tracker, "westbound", "SIDE_APPROACH", 2.0, driving=True)
        status = gate.status()
        self.assertEqual(
            status["robots"]["westbound"]["current_block"],
            "P4_RS1_TO_SIDE",
        )
        for block in status["arbiter"]["blocks"].values():
            self.assertIsNone(block["fault_reason"])

    def test_capacity_one_gate_keeps_second_robot_in_its_physical_slot(self) -> None:
        _, tracker, gate, forwarder = make_gate()
        ingest(tracker, "left-one", "P4_LS1", 1.0)
        ingest(tracker, "left-two", "P4_LS2", 1.0)

        first = gate.submit(payload("left-one", "P4_RS1"))
        second = gate.submit(payload("left-two", "P4_RS3"))
        self.assertEqual(first["decision"], "ADMIT")
        self.assertEqual(second["decision"], "WAIT")
        self.assertEqual(forwarder.goals, [("left-one", "2104")])
        self.assertEqual(
            gate.status()["robots"]["left-two"]["current_hb"],
            "HB_LEFT_SLOT_2",
        )

        ingest(tracker, "left-one", "2104", 2.0)
        gate.tick(now=2.0)
        self.assertEqual(forwarder.goals[-1], ("left-one", "P4_RS1"))
        ingest(tracker, "left-one", "2105", 2.5, driving=True)
        gate.tick(now=2.5)
        self.assertEqual(forwarder.goals[-1], ("left-two", "2104"))

    def test_staging_two_against_two_uses_distinct_slots_and_waits_for_a2(self) -> None:
        _, tracker, gate, forwarder = make_gate()
        for robot, node in (
            ("left-one", "P4_LS1"),
            ("left-two", "P4_LS2"),
            ("right-one", "P4_RS1"),
            ("right-two", "P4_RS2"),
        ):
            ingest(tracker, robot, node, 1.0)

        jobs = {
            robot: gate.submit(payload(robot, goal))
            for robot, goal in (
                ("left-one", "P4_RS1"),
                ("left-two", "P4_RS3"),
                ("right-one", "P4_LS1"),
                ("right-two", "P4_LS2"),
            )
        }
        self.assertEqual(
            forwarder.goals,
            [("left-one", "2104"), ("right-one", "6137")],
        )

        ingest(tracker, "left-one", "2104", 2.0)
        gate.tick(now=2.0)
        ingest(tracker, "right-one", "6137", 3.0)
        gate.tick(now=3.0)
        gate.tick(now=3.1)
        self.assertEqual(forwarder.goals[-1], ("left-one", "P4_RS1"))

        ingest(tracker, "left-one", "2105", 3.5, driving=True)
        gate.tick(now=3.5)
        self.assertEqual(forwarder.goals[-1], ("left-two", "2104"))
        ingest(tracker, "left-one", "2106", 4.0, driving=True)
        gate.tick(now=4.0)
        self.assertNotIn(("right-one", "P4_LS1"), forwarder.goals)

        ingest(tracker, "left-two", "2104", 5.0)
        gate.tick(now=5.0)
        self.assertEqual(forwarder.goals[-1], ("left-two", "P4_RS3"))
        ingest(tracker, "left-two", "2106", 6.0, driving=True)
        gate.tick(now=6.0)
        self.assertEqual(forwarder.goals[-1], ("right-one", "P4_LS1"))

        ingest(tracker, "left-one", "P4_RS1", 6.5)
        gate.tick(now=6.5)
        ingest(tracker, "left-two", "P4_RS3", 7.0)
        gate.tick(now=7.0)
        self.assertEqual(forwarder.goals[-1], ("right-two", "P4_LS2"))
        self.assertEqual(
            gate.status()["jobs"][jobs["right-two"]["job_id"]]["route_id"],
            "P4_RS2_TO_LS2_DIRECT_WHEN_CLEAR",
        )

        ingest(tracker, "right-one", "P4_LS1", 8.0)
        gate.tick(now=8.0)
        ingest(tracker, "right-two", "P4_LS2", 9.0)
        gate.tick(now=9.0)

        final = gate.status()
        self.assertEqual(
            {job["robot_id"]: job["status"] for job in final["jobs"].values()},
            {robot: "COMPLETE" for robot in jobs},
        )
        self.assertEqual(
            {robot: final["robots"][robot]["last_node_id"] for robot in jobs},
            {
                "left-one": "P4_RS1",
                "left-two": "P4_RS3",
                "right-one": "P4_LS1",
                "right-two": "P4_LS2",
            },
        )
        assert_clean(self, final)

    def test_staging_one_against_three_pipelines_trailing_direct_routes(self) -> None:
        _, tracker, gate, forwarder = make_gate()
        for robot, node in (
            ("eastbound", "P4_LS1"),
            ("westbound-one", "P4_RS1"),
            ("westbound-two", "P4_RS2"),
            ("westbound-three", "P4_RS3"),
        ):
            ingest(tracker, robot, node, 1.0)

        jobs = {
            robot: gate.submit(payload(robot, goal))
            for robot, goal in (
                ("eastbound", "P4_RS1"),
                ("westbound-one", "P4_LS1"),
                ("westbound-two", "P4_LS2"),
                ("westbound-three", "P4_LS3"),
            )
        }
        self.assertEqual(
            forwarder.goals,
            [("eastbound", "2104"), ("westbound-one", "6137")],
        )

        ingest(tracker, "eastbound", "2104", 2.0)
        gate.tick(now=2.0)
        ingest(tracker, "westbound-one", "6137", 3.0)
        gate.tick(now=3.0)
        gate.tick(now=3.1)
        self.assertEqual(forwarder.goals[-1], ("eastbound", "P4_RS1"))

        ingest(tracker, "eastbound", "2106", 4.0, driving=True)
        gate.tick(now=4.0)
        self.assertEqual(forwarder.goals[-1], ("westbound-one", "P4_LS1"))
        self.assertNotIn(("westbound-two", "P4_LS2"), forwarder.goals)
        self.assertNotIn(("westbound-three", "P4_LS3"), forwarder.goals)

        ingest(tracker, "eastbound", "P4_RS1", 5.0)
        gate.tick(now=5.0)
        direct_releases = [
            goal for goal in forwarder.goals
            if goal[0] in {"westbound-two", "westbound-three"}
            and goal[1] != "6137"
        ]
        self.assertEqual(
            direct_releases,
            [
                ("westbound-two", "P4_LS2"),
                ("westbound-three", "P4_LS3"),
            ],
        )

        ingest(tracker, "westbound-one", "P4_LS1", 6.0)
        gate.tick(now=6.0)
        ingest(tracker, "westbound-two", "P4_LS2", 7.0)
        gate.tick(now=7.0)
        ingest(tracker, "westbound-three", "P4_LS3", 8.0)
        gate.tick(now=8.0)

        final = gate.status()
        self.assertEqual(
            {job["robot_id"]: job["status"] for job in final["jobs"].values()},
            {robot: "COMPLETE" for robot in jobs},
        )
        self.assertEqual(
            {robot: final["robots"][robot]["last_node_id"] for robot in jobs},
            {
                "eastbound": "P4_RS1",
                "westbound-one": "P4_LS1",
                "westbound-two": "P4_LS2",
                "westbound-three": "P4_LS3",
            },
        )
        assert_clean(self, final)


if __name__ == "__main__":
    unittest.main()
