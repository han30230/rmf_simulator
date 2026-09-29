from __future__ import annotations

from pathlib import Path
import unittest

import yaml

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.robot_tracker import RobotTracker
from traffic_control.task_gate import TaskGate

from tests.test_corridor_chain_task_gate import payload


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/corridor_blocks_connected_chain.yaml"
MAP = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/connected_corridor_chain.yaml"


def map_positions() -> dict[str, tuple[float, float]]:
    vertices = yaml.safe_load(MAP.read_text(encoding="utf-8"))["levels"]["L1"]["vertices"]
    return {str(item[2]["name"]): (float(item[0]), float(item[1])) for item in vertices}


POSITIONS = map_positions()


def telemetry(node: str) -> dict:
    x, y = POSITIONS[node]
    return {
        "lastNodeId": node,
        "driving": False,
        "agvPosition": {"x": x, "y": y, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [],
    }


class Harness:
    def __init__(self, starts: dict[str, str]) -> None:
        self.registry = CorridorRegistry.from_yaml(CONFIG)
        self.arbiter = DirectionArbiter(self.registry)
        self.tracker = RobotTracker(self.registry, self.arbiter, telemetry_timeout=5.0)
        self.forwarded: list[tuple[str, str]] = []
        self.gate = TaskGate(
            self.registry,
            self.arbiter,
            self.tracker,
            lambda request: self.forwarded.append(
                (
                    str(request["robot"]),
                    str(request["request"]["description"]["places"][0]),
                )
            )
            or {"success": True},
        )
        self.now = 1.0
        for robot, node in starts.items():
            self.tracker.ingest_state(robot, telemetry(node), received_at=self.now)

    def submit(self, robot: str, goal: str) -> dict:
        return self.gate.submit(payload(robot, goal))

    def arrive(self, robot: str, node: str) -> None:
        authority = self.arbiter.authority_for_robot(robot)
        assert authority is not None, robot
        assert authority.goal_node == node, (robot, authority.goal_node, node)
        self.arbiter.mark_authority_arrived(robot)
        self.now += 0.1
        self.tracker.ingest_state(robot, telemetry(node), received_at=self.now)
        self.gate.tick(now=self.now)

    def assert_clean_complete(self) -> None:
        self.assert_all_jobs_complete()
        snapshot = self.gate.status()["arbiter"]
        for block in snapshot["blocks"].values():
            assert block["state"] == "FREE", block
            assert block["occupants"] == [], block
            assert block["reservations"] == [], block
            assert all(not queue for queue in block["waiting"].values()), block
            assert block["fault_reason"] is None, block
        assert snapshot["authorities"] == {}
        assert snapshot["pending_authorities"] == {}

    def assert_all_jobs_complete(self) -> None:
        jobs = self.gate.status()["jobs"].values()
        assert jobs and all(job["status"] == "COMPLETE" for job in jobs)


class ConnectedCorridorChainFlowTests(unittest.TestCase):
    def test_one_against_three_uses_a_refuge_then_finishes_cleanly(self) -> None:
        h = Harness({"A1": "CHAIN_L1", "B1": "CHAIN_R1", "B2": "CHAIN_R2", "B3": "CHAIN_R3"})
        self.assertEqual(h.submit("A1", "CHAIN_R1")["decision"], "ADMIT")
        self.assertEqual(h.submit("B1", "CHAIN_L1")["decision"], "WAIT")
        self.assertEqual(h.submit("B2", "CHAIN_L2")["decision"], "WAIT")
        self.assertEqual(h.submit("B3", "CHAIN_L3")["decision"], "WAIT")
        self.assertEqual(h.forwarded, [("A1", "CHAIN_SIDE_2")])

        h.arrive("A1", "CHAIN_SIDE_2")
        self.assertEqual(
            h.forwarded,
            [("A1", "CHAIN_SIDE_2"), ("B1", "CHAIN_L1"), ("B2", "CHAIN_L2"), ("B3", "CHAIN_L3")],
        )
        h.arrive("B1", "CHAIN_L1")
        h.arrive("B2", "CHAIN_L2")
        h.arrive("B3", "CHAIN_L3")
        self.assertEqual(h.forwarded[-1], ("A1", "CHAIN_R1"))
        h.arrive("A1", "CHAIN_R1")
        h.assert_clean_complete()

    def test_two_against_two_uses_distinct_refuges_and_finishes(self) -> None:
        h = Harness({"A1": "CHAIN_L1", "A2": "CHAIN_L2", "B1": "CHAIN_R1", "B2": "CHAIN_R2"})
        h.submit("A1", "CHAIN_R1")
        h.submit("A2", "CHAIN_R2")
        h.submit("B1", "CHAIN_L1")
        h.submit("B2", "CHAIN_L2")
        self.assertEqual(h.forwarded[:2], [("A1", "CHAIN_SIDE_2"), ("A2", "CHAIN_SIDE_1")])

        h.arrive("A1", "CHAIN_SIDE_2")
        h.arrive("A2", "CHAIN_SIDE_1")
        self.assertIn(("B1", "CHAIN_L1"), h.forwarded)
        self.assertIn(("B2", "CHAIN_L2"), h.forwarded)
        h.arrive("B1", "CHAIN_L1")
        h.arrive("B2", "CHAIN_L2")
        self.assertIn(("A1", "CHAIN_R1"), h.forwarded)
        self.assertIn(("A2", "CHAIN_R2"), h.forwarded)
        h.arrive("A1", "CHAIN_R1")
        h.arrive("A2", "CHAIN_R2")
        h.assert_clean_complete()

    def test_older_opposite_waiter_closes_the_current_direction_batch(self) -> None:
        h = Harness({"A1": "CHAIN_L1", "A2": "CHAIN_L2", "B1": "CHAIN_R1"})
        self.assertEqual(h.submit("A1", "CHAIN_R3")["decision"], "ADMIT")
        self.assertEqual(h.submit("B1", "CHAIN_L3")["decision"], "WAIT")

        self.assertEqual(h.submit("A2", "CHAIN_R2")["decision"], "WAIT")
        self.assertNotIn(("A2", "CHAIN_R2"), h.forwarded)

        h.arrive("A1", "CHAIN_R3")
        self.assertEqual(h.forwarded[-1], ("B1", "CHAIN_L3"))
        h.arrive("B1", "CHAIN_L3")
        self.assertEqual(h.forwarded[-1], ("A2", "CHAIN_R2"))
        h.arrive("A2", "CHAIN_R2")
        h.assert_clean_complete()

    def test_stale_or_missing_initial_telemetry_is_fail_closed(self) -> None:
        h = Harness({})
        result = h.submit("unknown", "CHAIN_R1")
        self.assertEqual(result["decision"], "BLOCKED")
        self.assertEqual(result["reason"], "robot_not_at_known_safe_point")


if __name__ == "__main__":
    unittest.main()
