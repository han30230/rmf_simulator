from __future__ import annotations

import unittest
from urllib.error import HTTPError

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.robot_tracker import RobotTracker
from traffic_control.task_gate import TaskGate


def make_registry(*, enabled: bool = True, endpoint_capacity: int = 4) -> CorridorRegistry:
    return CorridorRegistry.from_dict(
        {
            "traffic_control": {
                "enabled": enabled,
                "unmatched_route_policy": "BLOCKED",
            },
            "holding_bays": {
                "HB0": {
                    "node_id": "N0",
                    "capacity": endpoint_capacity,
                    "geometry": {"circle": {"x": 0.0, "y": 0.0, "radius": 0.5}},
                },
                "HB1": {
                    "node_id": "N1",
                    "capacity": 1,
                    "geometry": {"circle": {"x": 10.0, "y": 0.0, "radius": 0.5}},
                },
                "HB2": {
                    "node_id": "N2",
                    "capacity": endpoint_capacity,
                    "geometry": {"circle": {"x": 20.0, "y": 0.0, "radius": 0.5}},
                },
            },
            "blocks": [
                {
                    "id": "TOP_1",
                    "group_id": "TOP",
                    "direction_domain": "TOP_1",
                    "entry_a": "HB0",
                    "entry_b": "HB1",
                    "geometry": {"bounds": {"min_x": 0.6, "max_x": 9.4, "min_y": -1, "max_y": 1}},
                },
                {
                    "id": "TOP_2",
                    "group_id": "TOP",
                    "direction_domain": "TOP_2",
                    "entry_a": "HB1",
                    "entry_b": "HB2",
                    "geometry": {"bounds": {"min_x": 10.6, "max_x": 19.4, "min_y": -1, "max_y": 1}},
                },
            ],
            "routes": [
                {
                    "id": "TOP_FORWARD",
                    "start_nodes": ["N0"],
                    "goal_nodes": ["N2"],
                    "steps": [
                        {"block_id": "TOP_1", "direction": "A_TO_B", "source_hb": "HB0", "destination_hb": "HB1", "goal_node": "N1"},
                        {"block_id": "TOP_2", "direction": "A_TO_B", "source_hb": "HB1", "destination_hb": "HB2", "goal_node": "N2"},
                    ],
                },
                {
                    "id": "TOP_REVERSE",
                    "start_nodes": ["N2"],
                    "goal_nodes": ["N0"],
                    "steps": [
                        {"block_id": "TOP_2", "direction": "B_TO_A", "source_hb": "HB2", "destination_hb": "HB1", "goal_node": "N1"},
                        {"block_id": "TOP_1", "direction": "B_TO_A", "source_hb": "HB1", "destination_hb": "HB0", "goal_node": "N0"},
                    ],
                },
            ],
        }
    )


def payload(robot: str, goal: str) -> dict:
    return {
        "type": "robot_task_request",
        "robot": robot,
        "fleet": "TOOL",
        "request": {
            "unix_millis_earliest_start_time": 0,
            "category": "patrol",
            "priority": {"type": "default", "value": 0},
            "description": {"places": [goal], "rounds": 1},
        },
    }


def telemetry(x: float, node: str, *, driving: bool = False) -> dict:
    return {
        "lastNodeId": node,
        "driving": driving,
        "agvPosition": {"x": x, "y": 0.0, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [],
    }


class RecordingForwarder:
    def __init__(self, fail_count: int = 0) -> None:
        self.payloads: list[dict] = []
        self.fail_count = fail_count
        self.attempts = 0

    def __call__(self, submitted: dict) -> dict:
        self.attempts += 1
        if self.fail_count:
            self.fail_count -= 1
            raise RuntimeError("upstream unavailable")
        self.payloads.append(submitted)
        return {"success": True, "task_id": f"upstream-{len(self.payloads)}"}


def make_gate(*, enabled: bool = True, fail_count: int = 0, endpoint_capacity: int = 4):
    registry = make_registry(enabled=enabled, endpoint_capacity=endpoint_capacity)
    arbiter = DirectionArbiter(registry)
    tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)
    forwarder = RecordingForwarder(fail_count=fail_count)
    gate = TaskGate(registry, arbiter, tracker, forwarder)
    return gate, tracker, forwarder


class DynamicInsertTests(unittest.TestCase):
    def test_disabled_manager_passes_original_task_through(self) -> None:
        gate, _, forwarder = make_gate(enabled=False)

        result = gate.submit(payload("A1", "N2"))

        self.assertEqual(result["decision"], "BYPASS")
        self.assertEqual(forwarder.payloads[0]["request"]["description"]["places"], ["N2"])

    def test_route_is_submitted_one_safe_segment_at_a_time(self) -> None:
        gate, tracker, forwarder = make_gate()
        tracker.ingest_state("A1", telemetry(0.0, "N0"), received_at=1.0)

        result = gate.submit(payload("A1", "N2"))

        self.assertEqual(result["decision"], "ADMIT")
        self.assertEqual(forwarder.payloads[0]["request"]["description"]["places"], ["N1"])

        tracker.ingest_state("A1", telemetry(5.0, "", driving=True), received_at=2.0)
        tracker.ingest_state("A1", telemetry(10.0, "N1"), received_at=3.0)
        gate.tick(now=3.0)

        self.assertEqual(forwarder.payloads[1]["request"]["description"]["places"], ["N2"])

    def test_dynamic_opposite_waits_before_rmf_submission(self) -> None:
        gate, tracker, forwarder = make_gate()
        tracker.ingest_state("A1", telemetry(0.0, "N0"), received_at=1.0)
        tracker.ingest_state("B1", telemetry(20.0, "N2"), received_at=1.0)
        tracker.ingest_state("A2", telemetry(0.0, "N0"), received_at=1.0)

        self.assertEqual(gate.submit(payload("A1", "N2"))["decision"], "ADMIT")
        self.assertEqual(gate.submit(payload("B1", "N0"))["decision"], "WAIT")
        self.assertEqual(gate.submit(payload("A2", "N2"))["decision"], "WAIT")

        self.assertEqual(len(forwarder.payloads), 1)
        status = gate.status()
        self.assertIsNone(status["arbiter"]["domains"]["TOP_2"]["active_direction"])
        self.assertEqual(
            status["arbiter"]["blocks"]["TOP_2"]["waiting"]["B_TO_A"],
            ["B1"],
        )

    def test_waiting_job_can_be_cancelled_without_stale_reservation(self) -> None:
        gate, tracker, _ = make_gate()
        tracker.ingest_state("A1", telemetry(0.0, "N0"), received_at=1.0)
        tracker.ingest_state("B1", telemetry(20.0, "N2"), received_at=1.0)
        gate.submit(payload("A1", "N2"))
        waiting = gate.submit(payload("B1", "N0"))

        self.assertTrue(gate.cancel(waiting["job_id"]))

        jobs = gate.status()["jobs"]
        self.assertEqual(jobs[waiting["job_id"]]["status"], "CANCELLED")
        self.assertEqual(
            gate.status()["arbiter"]["blocks"]["TOP_2"]["waiting"]["B_TO_A"],
            [],
        )

    def test_upstream_failure_keeps_grant_without_automatic_retry(self) -> None:
        gate, tracker, forwarder = make_gate(fail_count=1)
        tracker.ingest_state("A1", telemetry(0.0, "N0"), received_at=1.0)

        result = gate.submit(payload("A1", "N2"))
        self.assertEqual(result["decision"], "FORWARD_UNKNOWN")

        gate.tick(now=2.0)
        gate.tick(now=3.0)

        self.assertEqual(forwarder.attempts, 1)
        self.assertEqual(forwarder.payloads, [])
        jobs = gate.status()["jobs"]
        self.assertEqual(len(jobs), 1)
        self.assertEqual(jobs[result["job_id"]]["status"], "ACTIVE")
        self.assertEqual(jobs[result["job_id"]]["last_error"], "upstream unavailable")

    def test_upstream_http_401_is_blocked_and_releases_reservation(self) -> None:
        gate, tracker, _ = make_gate()
        tracker.ingest_state("A1", telemetry(0.0, "N0"), received_at=1.0)

        def reject(_: dict) -> dict:
            raise HTTPError(
                "http://127.0.0.1:8100/tasks/robot_task",
                401,
                "Unauthorized",
                {},
                None,
            )

        gate.forwarder = reject
        result = gate.submit(payload("A1", "N2"))

        self.assertEqual(result["decision"], "BLOCKED")
        self.assertEqual(result["reason"], "upstream_http_rejection")
        self.assertEqual(result["status_code"], 401)

        status = gate.status()
        self.assertEqual(status["jobs"][result["job_id"]]["status"], "BLOCKED")
        self.assertEqual(
            status["arbiter"]["blocks"]["TOP_1"]["reservations"],
            [],
        )
        self.assertEqual(
            status["arbiter"]["holding_bays"]["HB1"]["reservations"],
            [],
        )

    def test_upstream_explicit_rejection_is_retried_not_marked_active(self) -> None:
        registry = make_registry()
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)
        tracker.ingest_state("A1", telemetry(0.0, "N0"), received_at=1.0)
        gate = TaskGate(
            registry,
            arbiter,
            tracker,
            lambda _: {"success": False, "message": "dispatcher rejected"},
        )

        result = gate.submit(payload("A1", "N2"))

        self.assertEqual(result["decision"], "RETRY")
        self.assertEqual(gate.status()["jobs"][result["job_id"]]["status"], "RETRY")


if __name__ == "__main__":
    unittest.main()
