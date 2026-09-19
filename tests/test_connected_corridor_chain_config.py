from __future__ import annotations

from collections import deque
from pathlib import Path
import unittest

import yaml

from traffic_control.corridor_registry import CorridorRegistry


ROOT = Path(__file__).resolve().parents[1]
ARBITER = ROOT / "config/corridor_blocks_connected_chain.yaml"
MAP = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/connected_corridor_chain.yaml"
FLEET = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter/config/connected_corridor_chain.yaml"
SIMULATOR = ROOT / "rmf_dev_tool-main/vda5050_robot_simulator/connected_corridor_chain_scenario.yaml"
COMPOSE = ROOT / "rmf_platform-main/docker-compose.connected-corridor-chain.yml"
SCRIPTS = (
    ROOT / "scripts/start_connected_corridor_chain.sh",
    ROOT / "scripts/launch_connected_corridor_chain_visualizer.sh",
    ROOT / "scripts/dispatch_connected_corridor_chain_1v3.sh",
    ROOT / "scripts/dispatch_connected_corridor_chain_2v2.sh",
)


class ConnectedCorridorChainConfigTests(unittest.TestCase):
    def test_runtime_files_exist(self) -> None:
        for path in (ARBITER, MAP, FLEET, SIMULATOR, COMPOSE, *SCRIPTS):
            self.assertTrue(path.is_file(), path)

    def test_map_is_one_connected_line_with_two_side_spurs(self) -> None:
        level = yaml.safe_load(MAP.read_text(encoding="utf-8"))["levels"]["L1"]
        names = [str(vertex[2]["name"]) for vertex in level["vertices"]]
        adjacency = {name: set() for name in names}
        for source_index, destination_index, _ in level["lanes"]:
            adjacency[names[source_index]].add(names[destination_index])
        reached = {names[0]}
        queue = deque([names[0]])
        while queue:
            queue.extend(adjacency[queue.popleft()] - reached)
            reached.update(queue)
        self.assertEqual(reached, set(names))
        self.assertEqual(adjacency["CHAIN_SIDE_1"], {"CHAIN_J1"})
        self.assertEqual(adjacency["CHAIN_SIDE_2"], {"CHAIN_J2"})

    def test_layout_uses_wide_comb_staging_and_terminal_robot_slots(self) -> None:
        level = yaml.safe_load(MAP.read_text(encoding="utf-8"))["levels"]["L1"]
        names = [str(vertex[2]["name"]) for vertex in level["vertices"]]
        positions = {
            str(vertex[2]["name"]): (float(vertex[0]), float(vertex[1]))
            for vertex in level["vertices"]
        }
        adjacency = {name: set() for name in names}
        for source_index, destination_index, _ in level["lanes"]:
            adjacency[names[source_index]].add(names[destination_index])

        endpoint_slots = tuple(
            f"CHAIN_{side}{index}"
            for side in ("L", "R")
            for index in range(1, 5)
        )
        self.assertNotIn("CHAIN_E1", positions)
        self.assertNotIn("CHAIN_E2", positions)
        self.assertTrue(all(positions[node][1] > 50.0 for node in endpoint_slots))
        self.assertTrue(all(len(adjacency[node]) == 1 for node in endpoint_slots))
        for side in ("L", "R"):
            for index in range(1, 5):
                self.assertEqual(
                    adjacency[f"CHAIN_{side}{index}"],
                    {f"CHAIN_{side}J{index}"},
                )
        self.assertLess(
            max(positions[node][1] for node in ("CHAIN_SIDE_1", "CHAIN_SIDE_2")),
            50.0,
        )
        self.assertNotEqual(
            positions["CHAIN_SIDE_1"][1],
            positions["CHAIN_SIDE_2"][1],
        )
        for node in (
            "CHAIN_C1_A", "CHAIN_C1_B", "CHAIN_C2_A", "CHAIN_C2_B",
            "CHAIN_C3_A", "CHAIN_C3_B",
        ):
            self.assertIn(node, positions)

        main_line = (
            "CHAIN_LJ4", "CHAIN_LJ3", "CHAIN_LJ2", "CHAIN_LJ1", "CHAIN_M0",
            "CHAIN_C1_A", "CHAIN_N1", "CHAIN_C1_B", "CHAIN_J1",
            "CHAIN_C2_A", "CHAIN_N2", "CHAIN_C2_B", "CHAIN_J2",
            "CHAIN_C3_A", "CHAIN_N3", "CHAIN_C3_B", "CHAIN_M3",
            "CHAIN_RJ1", "CHAIN_RJ2", "CHAIN_RJ3", "CHAIN_RJ4",
        )
        self.assertTrue(all(positions[node][1] == 50.0 for node in main_line))
        gaps = [
            round(positions[right][0] - positions[left][0], 1)
            for left, right in zip(main_line, main_line[1:])
        ]
        self.assertGreaterEqual(min(gaps), 7.0)
        self.assertGreaterEqual(len(set(gaps)), 4)

    def test_dispatch_uses_distinct_terminal_destination_slots(self) -> None:
        one_v_three = SCRIPTS[2].read_text(encoding="utf-8")
        two_v_two = SCRIPTS[3].read_text(encoding="utf-8")
        for expected in ("AGV_A1 CHAIN_R4", "AGV_B1 CHAIN_L4", "AGV_B2 CHAIN_L3", "AGV_B3 CHAIN_L2"):
            self.assertIn(expected, one_v_three)
        for expected in ("AGV_A1 CHAIN_R4", "AGV_A2 CHAIN_R3", "AGV_B1 CHAIN_L4", "AGV_B2 CHAIN_L3"):
            self.assertIn(expected, two_v_two)

    def test_chain_has_three_ordered_blocks_and_physical_safe_slots(self) -> None:
        raw = yaml.safe_load(ARBITER.read_text(encoding="utf-8"))["traffic_control"]
        registry = CorridorRegistry.from_yaml(ARBITER)
        chain = registry.corridor_chains["FAB_MAIN_LINE"]
        self.assertEqual(chain.block_ids, ("CHAIN_C1", "CHAIN_C2", "CHAIN_C3"))
        self.assertEqual(chain.max_active_robots, 4)
        member_slots = {
            member
            for group in registry.safe_stop_groups.values()
            for member in group.members
        }
        member_nodes = {registry.holding_bays[item].node_id for item in member_slots}
        self.assertNotIn("CHAIN_J1", member_nodes)
        self.assertNotIn("CHAIN_J2", member_nodes)
        self.assertTrue(all(registry.holding_bays[item].capacity == 1 for item in member_slots))
        self.assertNotIn("AGV_", ARBITER.read_text(encoding="utf-8"))

    def test_every_configured_edge_and_release_node_exists_in_the_map(self) -> None:
        level = yaml.safe_load(MAP.read_text(encoding="utf-8"))["levels"]["L1"]
        names = [str(vertex[2]["name"]) for vertex in level["vertices"]]
        lanes = {(names[item[0]], names[item[1]]) for item in level["lanes"]}
        registry = CorridorRegistry.from_yaml(ARBITER)
        for block in registry.blocks.values():
            for edge in block.edges_a_to_b | block.edges_b_to_a:
                self.assertIn(tuple(edge.split(">", 1)), lanes)
            self.assertTrue(block.release_node_a_to_b)
            self.assertTrue(block.release_node_b_to_a)

    def test_scenario_candidates_start_at_distinct_endpoint_slots(self) -> None:
        simulator = yaml.safe_load(SIMULATOR.read_text(encoding="utf-8"))
        robots = simulator["robots"]
        self.assertEqual(len(robots), 5)
        positions = {
            (item["initial_position"]["x"], item["initial_position"]["y"])
            for item in robots
        }
        self.assertEqual(len(positions), 5)
        level = yaml.safe_load(MAP.read_text(encoding="utf-8"))["levels"]["L1"]
        map_positions = {
            str(vertex[2]["name"]): (float(vertex[0]), float(vertex[1]))
            for vertex in level["vertices"]
        }
        expected_nodes = {
            "AGV_A1": "CHAIN_L1", "AGV_A2": "CHAIN_L2",
            "AGV_B1": "CHAIN_R1", "AGV_B2": "CHAIN_R2", "AGV_B3": "CHAIN_R3",
        }
        for robot in robots:
            initial = robot["initial_position"]
            self.assertEqual(
                (float(initial["x"]), float(initial["y"])),
                map_positions[expected_nodes[robot["serial_number"]]],
            )
        fleet = yaml.safe_load(FLEET.read_text(encoding="utf-8"))
        self.assertEqual(set(fleet["rmf_fleet"]["robots"]), {
            "AGV_A1", "AGV_A2", "AGV_B1", "AGV_B2", "AGV_B3"
        })

    def test_scripts_are_repository_relative(self) -> None:
        for path in SCRIPTS:
            content = path.read_text(encoding="utf-8")
            self.assertIn("BASH_SOURCE", content)
            self.assertNotIn("/home/han30230", content)
        compose = COMPOSE.read_text(encoding="utf-8")
        self.assertIn("connected_corridor_chain.yaml", compose)


if __name__ == "__main__":
    unittest.main()
