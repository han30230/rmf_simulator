from __future__ import annotations

import ast
from pathlib import Path
import unittest

import yaml

from traffic_control.corridor_registry import CorridorRegistry


ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"
MAP_PATH = ADAPTER / "map/p4_passing_bay_staging.yaml"
SIMULATOR_PATH = (
    ROOT / "rmf_dev_tool-main/vda5050_robot_simulator/p4_staging_scenario.yaml"
)
FLEET_PATH = ADAPTER / "config/p4_staging.yaml"
COMPOSE_PATH = ROOT / "rmf_platform-main/docker-compose.p4-passing-bay-staging.yml"
ARBITER_PATH = ROOT / "config/corridor_blocks_p4_passing_bay_staging.yaml"
START_2V2_PATH = ROOT / "scripts/start_p4_passing_bay_staging_2v2.sh"
DISPATCH_2V2_PATH = ROOT / "scripts/t4_dispatch_passing_bay_staging_2v2.sh"
START_1V3_PATH = ROOT / "scripts/start_p4_passing_bay_staging_1v3.sh"
DISPATCH_1V3_PATH = ROOT / "scripts/t4_dispatch_passing_bay_staging_1v3.sh"
VISUALIZER_PATH = ROOT / "scripts/launch_p4_passing_bay_staging_visualizer.sh"
VISUALIZER_GUI_PATH = ROOT / "rmf_dev_tool-main/vda5050_gui/vda5050_gui.py"

SLOTS = {
    "P4_LS1": (6.833, 96.0),
    "P4_LS2": (3.2, 95.0),
    "P4_LS3": (3.2, 90.7),
    "P4_RS1": (73.7274, 96.0),
    "P4_RS2": (77.3, 95.0),
    "P4_RS3": (77.3, 90.7),
}


class PassingBayStagingConfigurationTests(unittest.TestCase):
    def test_visualizer_actions_are_main_window_methods(self) -> None:
        tree = ast.parse(VISUALIZER_GUI_PATH.read_text(encoding="utf-8"))
        simulation_tab = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "SimulationTab"
        )
        simulation_methods = {
            node.name for node in simulation_tab.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        self.assertTrue(
            {
                "_save_config",
                "_selected_row",
                "_start_all",
                "_execute_command",
                "shutdown",
            }.issubset(simulation_methods)
        )

        main_window = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "MainWindow"
        )
        methods = {
            node.name for node in main_window.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        self.assertTrue(
            {
                "_refresh_agv_filter",
                "_on_agv_filter_toggled",
                "_open_file",
                "_connect_mqtt",
                "_on_index_changed",
                "closeEvent",
            }.issubset(methods)
        )

    def test_staging_runtime_files_exist(self) -> None:
        for path in (MAP_PATH, SIMULATOR_PATH, FLEET_PATH, COMPOSE_PATH):
            self.assertTrue(path.is_file(), path)

    def test_graph_has_six_distinct_slots_and_bidirectional_spurs(self) -> None:
        data = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))
        level = data["levels"]["L1"]
        vertices = level["vertices"]
        index = {
            str(vertex[2]["name"]): position
            for position, vertex in enumerate(vertices)
        }

        self.assertEqual(set(SLOTS), set(SLOTS) & set(index))
        self.assertEqual(len(set(SLOTS.values())), 6)
        for name, (expected_x, expected_y) in SLOTS.items():
            vertex = vertices[index[name]]
            self.assertAlmostEqual(float(vertex[0]), expected_x, places=3)
            self.assertAlmostEqual(float(vertex[1]), expected_y, places=3)

        lanes = {(int(lane[0]), int(lane[1])) for lane in level["lanes"]}
        for name in ("P4_LS1", "P4_LS2", "P4_LS3"):
            self.assertIn((index[name], index["2101"]), lanes)
            self.assertIn((index["2101"], index[name]), lanes)
        for name in ("P4_RS1", "P4_RS2", "P4_RS3"):
            self.assertIn((index[name], index["2108"]), lanes)
            self.assertIn((index["2108"], index[name]), lanes)

        self.assertIn((index["2105"], index["6137"]), lanes)
        self.assertIn((index["6137"], index["2105"]), lanes)

    def test_simulator_robots_start_at_distinct_slot_coordinates(self) -> None:
        scenario = yaml.safe_load(SIMULATOR_PATH.read_text(encoding="utf-8"))
        positions = {
            robot["serial_number"]: (
                float(robot["initial_position"]["x"]),
                float(robot["initial_position"]["y"]),
            )
            for robot in scenario["robots"]
        }
        expected = {
            "AGV_A1": SLOTS["P4_LS1"],
            "AGV_A2": SLOTS["P4_LS2"],
            "AGV_B1": SLOTS["P4_RS1"],
            "AGV_B2": SLOTS["P4_RS2"],
            "AGV_B3": SLOTS["P4_RS3"],
        }
        self.assertEqual(positions, expected)
        self.assertEqual(len(set(positions.values())), len(positions))

    def test_fleet_and_compose_select_the_staging_stack(self) -> None:
        fleet = yaml.safe_load(FLEET_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            set(fleet["rmf_fleet"]["robots"]),
            {"AGV_A1", "AGV_A2", "AGV_B1", "AGV_B2", "AGV_B3"},
        )

        compose = yaml.safe_load(COMPOSE_PATH.read_text(encoding="utf-8"))
        command = " ".join(
            compose["services"]["vda5050_fleet_adapter"]["command"]
        )
        self.assertIn("config/p4_staging.yaml", command)
        self.assertIn("map/p4_passing_bay_staging.yaml", command)

    def test_arbiter_models_six_capacity_one_slots_without_robot_policy(self) -> None:
        raw_text = ARBITER_PATH.read_text(encoding="utf-8")
        for robot_id in ("AGV_A1", "AGV_A2", "AGV_B1", "AGV_B2", "AGV_B3"):
            self.assertNotIn(robot_id, raw_text)

        traffic = yaml.safe_load(raw_text)["traffic_control"]
        expected = {
            "HB_LEFT_SLOT_1": "P4_LS1",
            "HB_LEFT_SLOT_2": "P4_LS2",
            "HB_LEFT_SLOT_3": "P4_LS3",
            "HB_RIGHT_SLOT_1": "P4_RS1",
            "HB_RIGHT_SLOT_2": "P4_RS2",
            "HB_RIGHT_SLOT_3": "P4_RS3",
        }
        for hb_id, node_id in expected.items():
            bay = traffic["holding_bays"][hb_id]
            self.assertEqual(str(bay["node_id"]), node_id)
            self.assertEqual(int(bay["capacity"]), 1)

        centers = [
            (
                float(traffic["holding_bays"][hb_id]["geometry"]["circle"]["x"]),
                float(traffic["holding_bays"][hb_id]["geometry"]["circle"]["y"]),
            )
            for hb_id in expected
        ]
        self.assertEqual(len(set(centers)), 6)
        CorridorRegistry.from_yaml(ARBITER_PATH)

    def test_every_arbiter_edge_exists_in_the_staging_graph(self) -> None:
        graph = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))["levels"]["L1"]
        names = [str(vertex[2]["name"]) for vertex in graph["vertices"]]
        graph_edges = {
            f"{names[int(lane[0])]}>{names[int(lane[1])]}"
            for lane in graph["lanes"]
        }
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        blocks = [
            block
            for group in traffic["groups"].values()
            for block in group["blocks"]
        ]
        for block in blocks:
            configured = set(block.get("edges_a_to_b", [])) | set(
                block.get("edges_b_to_a", [])
            )
            self.assertTrue(configured, block["id"])
            self.assertLessEqual(configured, graph_edges, block["id"])

    def test_staging_routes_keep_conflict_policy_in_configuration(self) -> None:
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        blocks = {
            block["id"]: block
            for group in traffic["groups"].values()
            for block in group["blocks"]
        }
        self.assertEqual(
            {
                block["direction_domain"]
                for block_id, block in blocks.items()
                if "_TO_GATE" in block_id
            },
            {"P4_WEST_ADVANCE"},
        )
        self.assertEqual(
            {
                block["direction_domain"]
                for block_id, block in blocks.items()
                if "_TO_GATE" not in block_id
            },
            {"P4_PASSING_EVENT"},
        )

        routes = {route["id"]: route for route in traffic["routes"]}
        forward = [
            route for route in routes.values()
            if route["start_nodes"][0].startswith("P4_LS")
        ]
        for route in forward:
            self.assertEqual(route["steps"][1]["release_node"], "2106")

        reverse_via = [
            route for route_id, route in routes.items()
            if route_id.endswith("VIA_SIDE")
        ]
        for route in reverse_via:
            self.assertTrue(
                route["steps"][1]["requires_opposite_routes_cleared"]
            )

        for slot in (2, 3):
            via = routes[f"P4_RS{slot}_TO_LS{slot}_VIA_SIDE"]
            direct = routes[f"P4_RS{slot}_TO_LS{slot}_DIRECT_WHEN_CLEAR"]
            self.assertEqual(via["start_nodes"], [f"P4_RS{slot}"])
            self.assertEqual(via["goal_nodes"], [f"P4_LS{slot}"])
            self.assertTrue(direct["requires_no_opposite_jobs"])

    def test_release_node_exit_is_outside_all_staging_conflict_geometry(self) -> None:
        registry = CorridorRegistry.from_yaml(ARBITER_PATH)
        # 2107 is east of the configured 2106 release boundary. A robot whose
        # retained grant is CLEARED must not fall into another overlapping
        # passing block while it continues to its physical destination slot.
        matches = registry.blocks_for_position(66.612, 92.871)
        self.assertEqual(
            [
                block_id
                for block_id in matches
                if registry.blocks[block_id].direction_domain
                == "P4_PASSING_EVENT"
            ],
            [],
        )

    def test_staging_scripts_are_repository_relative_and_select_runtime_data(self) -> None:
        for path in (
            START_2V2_PATH,
            DISPATCH_2V2_PATH,
            START_1V3_PATH,
            DISPATCH_1V3_PATH,
            VISUALIZER_PATH,
        ):
            self.assertTrue(path.is_file(), path)
            content = path.read_text(encoding="utf-8")
            self.assertIn("BASH_SOURCE", content)
            self.assertNotIn("/home/han30230", content)

        for path in (START_2V2_PATH, START_1V3_PATH):
            content = path.read_text(encoding="utf-8")
            self.assertIn("PASSING_BAY_SIMULATOR_SCENARIO", content)
            self.assertIn("PASSING_BAY_COMPOSE_FILE", content)
            self.assertIn("PASSING_BAY_ARBITER_CONFIG", content)
            self.assertIn("PASSING_BAY_RUNTIME_NAME", content)
            self.assertIn("p4_staging_scenario.yaml", content)
            self.assertIn("docker-compose.p4-passing-bay-staging.yml", content)
            self.assertIn("corridor_blocks_p4_passing_bay_staging.yaml", content)

        self.assertIn(
            '/start_p4_passing_bay.sh" AGV_A1 AGV_A2 AGV_B1 AGV_B2',
            START_2V2_PATH.read_text(encoding="utf-8"),
        )
        self.assertIn(
            '/start_p4_passing_bay.sh" AGV_A1 AGV_B1 AGV_B2 AGV_B3',
            START_1V3_PATH.read_text(encoding="utf-8"),
        )

    def test_staging_dispatches_use_distinct_physical_destinations(self) -> None:
        def requests(path: Path) -> list[str]:
            return [
                line.strip()
                for line in path.read_text(encoding="utf-8").splitlines()
                if "t4_dispatch_via_arbiter.sh" in line
            ]

        self.assertEqual(
            requests(DISPATCH_2V2_PATH),
            [
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 P4_RS1',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A2 P4_RS3',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 P4_LS1',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 P4_LS2',
            ],
        )
        self.assertEqual(
            requests(DISPATCH_1V3_PATH),
            [
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 P4_RS1',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 P4_LS1',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 P4_LS2',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B3 P4_LS3',
            ],
        )


if __name__ == "__main__":
    unittest.main()
