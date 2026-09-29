from __future__ import annotations

from pathlib import Path
import unittest

import yaml

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.robot_tracker import RobotTracker
from traffic_control.task_gate import TaskGate


ROOT = Path(__file__).resolve().parents[1]
MAP_PATH = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_p5_multi_passing_bay.yaml"
SIM_PATH = ROOT / "rmf_dev_tool-main/vda5050_robot_simulator/p4_p5_multi_corridor_scenario.yaml"
FLEET_PATH = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter/config/p4_p5_multi_corridor.yaml"
COMPOSE_PATH = ROOT / "rmf_platform-main/docker-compose.p4-p5-multi-corridor.yml"
ARBITER_PATH = ROOT / "config/corridor_blocks_p4_p5_multi.yaml"
START_PATH = ROOT / "scripts/start_p4_p5_multi_corridor.sh"
DISPATCH_PATH = ROOT / "scripts/t4_dispatch_p4_p5_multi_corridor.sh"
VISUALIZER_PATH = ROOT / "scripts/launch_p4_p5_multi_corridor_visualizer.sh"

POSITIONS = {
    "2101": (6.833, 92.871),
    "2104": (30.424, 92.871),
    "2108": (73.7274, 92.8248),
    "6137": (42.508, 91.100),
    "3101": (6.833, 72.871),
    "3108": (73.7274, 72.8248),
}


def state(node: str) -> dict:
    x, y = POSITIONS[node]
    return {
        "lastNodeId": node,
        "driving": False,
        "agvPosition": {"x": x, "y": y, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [],
    }


def payload(robot: str, goal: str) -> dict:
    return {
        "type": "robot_task_request",
        "robot": robot,
        "fleet": "TOOL",
        "request": {
            "category": "patrol",
            "description": {"places": [goal], "rounds": 1},
        },
    }


class MultiCorridorConfigurationTests(unittest.TestCase):
    def test_runtime_files_exist_and_are_repository_relative(self) -> None:
        for path in (
            MAP_PATH,
            SIM_PATH,
            FLEET_PATH,
            COMPOSE_PATH,
            ARBITER_PATH,
            START_PATH,
            DISPATCH_PATH,
            VISUALIZER_PATH,
        ):
            self.assertTrue(path.is_file(), path)
        for path in (START_PATH, DISPATCH_PATH, VISUALIZER_PATH):
            content = path.read_text(encoding="utf-8")
            self.assertIn("BASH_SOURCE", content)
            self.assertNotIn("/home/han30230", content)

    def test_map_contains_two_disconnected_passing_bay_corridors(self) -> None:
        level = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))["levels"]["L1"]
        names = [str(vertex[2]["name"]) for vertex in level["vertices"]]
        p4 = {str(node) for node in range(2101, 2109)} | {"6137"}
        p5 = {str(node) for node in range(3101, 3109)} | {"7137"}
        self.assertLessEqual(p4 | p5, set(names))

        lanes = [(names[int(lane[0])], names[int(lane[1])]) for lane in level["lanes"]]
        for source, destination in lanes:
            self.assertFalse(
                (source in p4 and destination in p5)
                or (source in p5 and destination in p4)
            )

    def test_corridors_use_independent_direction_domains(self) -> None:
        registry = CorridorRegistry.from_yaml(ARBITER_PATH)
        p4_domains = {
            block.direction_domain
            for block_id, block in registry.blocks.items()
            if block_id.startswith("P4_")
        }
        p5_domains = {
            block.direction_domain
            for block_id, block in registry.blocks.items()
            if block_id.startswith("P5_")
        }
        self.assertTrue(p4_domains)
        self.assertTrue(p5_domains)
        self.assertTrue(p4_domains.isdisjoint(p5_domains))

    def test_opposite_waiter_in_one_corridor_does_not_block_the_other(self) -> None:
        registry = CorridorRegistry.from_yaml(ARBITER_PATH)
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter, telemetry_timeout=20.0)
        released: list[tuple[str, str]] = []

        def forwarder(request: dict) -> dict:
            released.append(
                (
                    str(request["robot"]),
                    str(request["request"]["description"]["places"][0]),
                )
            )
            return {"success": True}

        gate = TaskGate(registry, arbiter, tracker, forwarder)
        for robot, node in (
            ("p4-eastbound", "2101"),
            ("p4-westbound", "2108"),
            ("p5-eastbound", "3101"),
            ("p5-westbound", "3108"),
        ):
            tracker.ingest_state(robot, state(node), received_at=1.0)

        self.assertEqual(
            gate.submit(payload("p4-eastbound", "2108"))["decision"],
            "ADMIT",
        )
        self.assertEqual(
            gate.submit(payload("p4-westbound", "2101"))["decision"],
            "ADMIT",
        )
        self.assertEqual(
            gate.submit(payload("p5-westbound", "3101"))["decision"],
            "ADMIT",
        )
        self.assertEqual(
            gate.submit(payload("p5-eastbound", "3108"))["decision"],
            "ADMIT",
        )
        self.assertEqual(
            released,
            [
                ("p4-eastbound", "2104"),
                ("p4-westbound", "6137"),
                ("p5-westbound", "3101"),
                ("p5-eastbound", "3104"),
            ],
        )

        tracker.ingest_state(
            "p4-eastbound", state("2104"), received_at=2.0
        )
        gate.tick(now=2.0)
        tracker.ingest_state(
            "p4-westbound", state("6137"), received_at=3.0
        )
        gate.tick(now=3.0)
        gate.tick(now=3.1)

        self.assertIn(("p4-eastbound", "2108"), released)
        domains = gate.status()["arbiter"]["domains"]
        self.assertEqual(domains["P4_PASSING_EVENT"]["active_direction"], "A_TO_B")
        self.assertEqual(domains["P5_PASSING_EVENT"]["active_direction"], "B_TO_A")


if __name__ == "__main__":
    unittest.main()
