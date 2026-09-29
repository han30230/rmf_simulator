from __future__ import annotations

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"
MAP_PATH = ADAPTER / "map/p4_passing_bay.yaml"
ARBITER_PATH = ROOT / "config/corridor_blocks_p4_passing_bay.yaml"
COMPOSE_PATH = ROOT / "rmf_platform-main/docker-compose.p4-passing-bay.yml"
DISPATCH_PATH = ROOT / "scripts/t4_dispatch_passing_bay.sh"
START_2V1_PATH = ROOT / "scripts/start_p4_passing_bay_2v1.sh"
DISPATCH_2V1_PATH = ROOT / "scripts/t4_dispatch_passing_bay_2v1.sh"
START_2V2_PATH = ROOT / "scripts/start_p4_passing_bay_2v2.sh"
DISPATCH_2V2_PATH = ROOT / "scripts/t4_dispatch_passing_bay_2v2.sh"


def contains(geometry: dict, x: float, y: float) -> bool:
    bounds = geometry["bounds"]
    return (
        float(bounds["min_x"]) <= x <= float(bounds["max_x"])
        and float(bounds["min_y"]) <= y <= float(bounds["max_y"])
    )


class PassingBayConfigurationTests(unittest.TestCase):
    def test_required_runtime_files_exist(self) -> None:
        for path in (MAP_PATH, ARBITER_PATH, COMPOSE_PATH, DISPATCH_PATH):
            self.assertTrue(path.is_file(), path)

    def test_two_against_one_runtime_selects_all_three_robots(self) -> None:
        self.assertTrue(START_2V1_PATH.is_file(), START_2V1_PATH)
        self.assertTrue(DISPATCH_2V1_PATH.is_file(), DISPATCH_2V1_PATH)

        start = START_2V1_PATH.read_text(encoding="utf-8")
        self.assertIn(
            "PASSING_BAY_DISPATCH_SCRIPT=t4_dispatch_passing_bay_2v1.sh",
            start,
        )
        self.assertIn(
            '/start_p4_passing_bay.sh" AGV_A1 AGV_B1 AGV_B2',
            start,
        )

        dispatch = DISPATCH_2V1_PATH.read_text(encoding="utf-8")
        requests = [
            line.strip()
            for line in dispatch.splitlines()
            if "t4_dispatch_via_arbiter.sh" in line
        ]
        self.assertEqual(
            requests,
            [
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 2108',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 2101',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 2101',
            ],
        )

    def test_two_against_two_runtime_selects_all_four_robots(self) -> None:
        self.assertTrue(START_2V2_PATH.is_file(), START_2V2_PATH)
        self.assertTrue(DISPATCH_2V2_PATH.is_file(), DISPATCH_2V2_PATH)

        start = START_2V2_PATH.read_text(encoding="utf-8")
        self.assertIn(
            "PASSING_BAY_DISPATCH_SCRIPT=t4_dispatch_passing_bay_2v2.sh",
            start,
        )
        self.assertIn(
            '/start_p4_passing_bay.sh" AGV_A1 AGV_A2 AGV_B1 AGV_B2',
            start,
        )

        dispatch = DISPATCH_2V2_PATH.read_text(encoding="utf-8")
        requests = [
            line.strip()
            for line in dispatch.splitlines()
            if "t4_dispatch_via_arbiter.sh" in line
        ]
        self.assertEqual(
            requests,
            [
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A1 2108',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_A2 2108',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B1 2101',
                '"${script_dir}/t4_dispatch_via_arbiter.sh" AGV_B2 2101',
            ],
        )

    def test_side_bay_is_bidirectionally_connected_to_2105(self) -> None:
        data = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))
        level = data["levels"]["L1"]
        vertices = level["vertices"]
        names = [str(vertex[2]["name"]) for vertex in vertices]
        index = {name: position for position, name in enumerate(names)}

        self.assertIn("6137", index)
        side = vertices[index["6137"]]
        self.assertAlmostEqual(float(side[0]), 42.508, places=3)
        self.assertAlmostEqual(float(side[1]), 91.100, places=3)

        lanes = {(int(lane[0]), int(lane[1])) for lane in level["lanes"]}
        self.assertIn((index["2105"], index["6137"]), lanes)
        self.assertIn((index["6137"], index["2105"]), lanes)

    def test_side_bay_is_outside_all_managed_block_geometries(self) -> None:
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        side_circle = traffic["holding_bays"]["HB_MIDDLE_SIDE"]["geometry"][
            "circle"
        ]
        side_x = float(side_circle["x"])
        side_y = float(side_circle["y"])

        blocks = [
            block
            for group in traffic["groups"].values()
            for block in group["blocks"]
        ]
        for block in blocks:
            self.assertFalse(
                contains(block["geometry"], side_x, side_y),
                f'{block["id"]} overlaps the side-bay center',
            )

        self.assertEqual(
            int(traffic["holding_bays"]["HB_MIDDLE_SIDE"]["capacity"]),
            1,
        )

    def test_passing_blocks_share_one_direction_domain(self) -> None:
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        blocks = {
            block["id"]: block
            for group in traffic["groups"].values()
            for block in group["blocks"]
        }
        shared = {
            "P4_EAST_TO_SIDE",
            "P4_GATE_TO_RIGHT",
            "P4_RIGHT_TO_LEFT_DIRECT",
            "P4_SIDE_TO_LEFT",
        }

        self.assertEqual(
            {blocks[block_id]["direction_domain"] for block_id in shared},
            {"P4_PASSING_EVENT"},
        )
        self.assertNotEqual(
            blocks["P4_WEST_ADVANCE"]["direction_domain"],
            "P4_PASSING_EVENT",
        )
        self.assertTrue(blocks["P4_EAST_TO_SIDE"]["require_source_hb_unreserved"])

    def test_clear_corridor_has_a_guarded_direct_right_to_left_route(self) -> None:
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        blocks = {
            block["id"]: block
            for group in traffic["groups"].values()
            for block in group["blocks"]
        }
        direct = blocks["P4_RIGHT_TO_LEFT_DIRECT"]
        self.assertEqual((direct["entry_a"], direct["entry_b"]), ("HB_LEFT", "HB_RIGHT"))
        self.assertTrue(direct["require_source_hb_unreserved"])
        self.assertEqual(
            direct["edges_b_to_a"],
            [
                "2108>2107",
                "2107>2106",
                "2106>2105",
                "2105>2104",
                "2104>2103",
                "2103>2102",
                "2102>2101",
            ],
        )

        routes = {route["id"]: route for route in traffic["routes"]}
        route = routes["P4_RIGHT_TO_LEFT_DIRECT_WHEN_CLEAR"]
        self.assertTrue(route["requires_no_opposite_jobs"])
        self.assertEqual(len(route["steps"]), 1)
        self.assertEqual(route["steps"][0]["block_id"], "P4_RIGHT_TO_LEFT_DIRECT")
        self.assertEqual(route["steps"][0]["goal_node"], "2101")

    def test_managed_routes_have_the_expected_two_steps(self) -> None:
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        routes = {route["id"]: route for route in traffic["routes"]}

        forward = routes["P4_LEFT_TO_RIGHT_VIA_GATE"]
        reverse = routes["P4_RIGHT_TO_LEFT_VIA_SIDE"]
        self.assertEqual(
            [step["goal_node"] for step in forward["steps"]],
            ["2104", "2108"],
        )
        self.assertEqual(
            [step["goal_node"] for step in reverse["steps"]],
            ["6137", "2101"],
        )
        self.assertEqual(
            [step["source_hb"] for step in forward["steps"]],
            ["HB_LEFT", "HB_WEST_GATE"],
        )
        self.assertEqual(
            [step["source_hb"] for step in reverse["steps"]],
            ["HB_RIGHT", "HB_MIDDLE_SIDE"],
        )
        self.assertTrue(reverse["steps"][1]["requires_opposite_routes_cleared"])

    def test_forward_route_releases_only_after_the_central_conflict(self) -> None:
        traffic = yaml.safe_load(ARBITER_PATH.read_text(encoding="utf-8"))[
            "traffic_control"
        ]
        routes = {route["id"]: route for route in traffic["routes"]}
        forward = routes["P4_LEFT_TO_RIGHT_VIA_GATE"]
        self.assertEqual(forward["steps"][1]["release_node"], "2106")

        blocks = {
            block["id"]: block
            for group in traffic["groups"].values()
            for block in group["blocks"]
        }
        gate_circle = traffic["holding_bays"]["HB_WEST_GATE"]["geometry"][
            "circle"
        ]
        gate_center_x = float(gate_circle["x"])
        gate_east_edge = gate_center_x + float(gate_circle["radius"])
        for block_id in ("P4_EAST_TO_SIDE", "P4_GATE_TO_RIGHT"):
            bounds = blocks[block_id]["geometry"]["bounds"]
            # The passing geometry must begin inside the east half of the gate
            # bay so state updates cannot fall through to an opposing block.
            self.assertGreaterEqual(float(bounds["min_x"]), gate_center_x)
            self.assertLessEqual(float(bounds["min_x"]), gate_east_edge)
            self.assertEqual(float(bounds["max_x"]), 54.818)

        side_to_left = blocks["P4_SIDE_TO_LEFT"]["geometry"]["bounds"]
        self.assertEqual(float(side_to_left["min_x"]), 7.6)
        self.assertEqual(float(side_to_left["max_x"]), 54.818)

    def test_compose_and_dispatch_select_passing_bay_runtime(self) -> None:
        compose = yaml.safe_load(COMPOSE_PATH.read_text(encoding="utf-8"))
        command = " ".join(
            compose["services"]["vda5050_fleet_adapter"]["command"]
        )
        self.assertIn("map/p4_passing_bay.yaml", command)
        self.assertIn("config/p4_edit_before.yaml", command)

        dispatch = DISPATCH_PATH.read_text(encoding="utf-8")
        self.assertIn("AGV_A1 2108", dispatch)
        self.assertIn("AGV_B1 2101", dispatch)
        self.assertNotIn("AGV_B2", dispatch)


if __name__ == "__main__":
    unittest.main()
