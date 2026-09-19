from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"


class HomeP4ConfigurationTests(unittest.TestCase):
    def test_required_files_exist(self) -> None:
        for path in (
            ADAPTER / "map/p4_edit_node_add.yaml",
            ADAPTER / "config/p4_edit_before.yaml",
            ROOT / "rmf_dev_tool-main/vda5050_robot_simulator/p4_scenario.yaml",
            ROOT / "config/corridor_blocks_p4.yaml",
            ROOT / "rmf_platform-main/docker-compose.p4.yml",
        ):
            self.assertTrue(path.is_file(), path)

    def test_map_is_bidirectional_2101_to_2108(self) -> None:
        data = yaml.safe_load(
            (ADAPTER / "map/p4_edit_node_add.yaml").read_text(encoding="utf-8")
        )
        level = data["levels"]["L1"]
        names = [str(vertex[2]["name"]) for vertex in level["vertices"]]
        self.assertEqual(names, [str(node) for node in range(2101, 2109)])

        lanes = {(int(lane[0]), int(lane[1])) for lane in level["lanes"]}
        expected = set()
        for index in range(7):
            expected.add((index, index + 1))
            expected.add((index + 1, index))
        self.assertEqual(lanes, expected)

    def test_four_robots_share_the_same_endpoint_coordinates(self) -> None:
        scenario = yaml.safe_load(
            (
                ROOT
                / "rmf_dev_tool-main/vda5050_robot_simulator/p4_scenario.yaml"
            ).read_text(encoding="utf-8")
        )
        robots = {
            robot["serial_number"]: robot["initial_position"]
            for robot in scenario["robots"]
        }
        self.assertEqual(
            set(robots),
            {"AGV_A1", "AGV_A2", "AGV_B1", "AGV_B2"},
        )
        for robot_id in ("AGV_A1", "AGV_A2"):
            self.assertAlmostEqual(float(robots[robot_id]["x"]), 6.833, places=3)
            self.assertAlmostEqual(float(robots[robot_id]["y"]), 92.871, places=3)
        for robot_id in ("AGV_B1", "AGV_B2"):
            self.assertAlmostEqual(float(robots[robot_id]["x"]), 73.7274, places=3)
            self.assertAlmostEqual(float(robots[robot_id]["y"]), 92.8248, places=3)

        fleet = yaml.safe_load(
            (ADAPTER / "config/p4_edit_before.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            set(fleet["rmf_fleet"]["robots"]),
            {"AGV_A1", "AGV_A2", "AGV_B1", "AGV_B2"},
        )

    def test_arbiter_endpoints_match_navigation_graph(self) -> None:
        arbiter = yaml.safe_load(
            (ROOT / "config/corridor_blocks_p4.yaml").read_text(encoding="utf-8")
        )["traffic_control"]
        left = arbiter["holding_bays"]["HB_LEFT"]
        right = arbiter["holding_bays"]["HB_RIGHT"]
        self.assertEqual(left["node_id"], "2101")
        self.assertEqual(right["node_id"], "2108")
        self.assertAlmostEqual(float(right["geometry"]["circle"]["x"]), 73.7274, places=3)


if __name__ == "__main__":
    unittest.main()
