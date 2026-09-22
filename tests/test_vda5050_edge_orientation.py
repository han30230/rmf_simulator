from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
ADAPTER_ROOT = (
    ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"
)
sys.path.insert(0, str(ADAPTER_ROOT))

from vda5050_fleet_adapter.usecase.graph_utils import (  # noqa: E402
    build_vda5050_nodes_edges,
)


class Vda5050EdgeOrientationTests(unittest.TestCase):
    def test_reverse_route_edges_face_their_direction_of_travel(self) -> None:
        nav_nodes = {
            "6137": {"x": 42.508, "y": 91.100, "attributes": {}},
            "2105": {"x": 42.508, "y": 92.871, "attributes": {}},
            "2104": {"x": 30.424, "y": 92.871, "attributes": {}},
        }
        nav_edges = {
            "side_to_main": {
                "start": "6137",
                "end": "2105",
                "attributes": {},
            },
            "main_to_west": {
                "start": "2105",
                "end": "2104",
                "attributes": {},
            },
        }

        _, edges = build_vda5050_nodes_edges(
            ["6137", "2105", "2104"],
            nav_nodes,
            "L1",
            base_end_index=2,
            edges=nav_edges,
        )

        self.assertEqual([edge.edge_id for edge in edges], [
            "side_to_main",
            "main_to_west",
        ])
        self.assertEqual(edges[0].orientation, math.pi / 2)
        self.assertEqual(edges[1].orientation, math.pi)


if __name__ == "__main__":
    unittest.main()
