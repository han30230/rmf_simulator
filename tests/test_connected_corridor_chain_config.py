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
