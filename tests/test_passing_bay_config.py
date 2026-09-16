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
