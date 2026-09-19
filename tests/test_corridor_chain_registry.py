from __future__ import annotations

import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.models import Direction


def chain_config() -> dict:
    return {
        "traffic_control": {"enabled": True},
        "holding_bays": {
            "LEFT_1": {"node_id": "L1", "capacity": 1},
            "LEFT_2": {"node_id": "L2", "capacity": 1},
            "SIDE_1": {"node_id": "S1", "capacity": 1},
            "SIDE_2": {"node_id": "S2", "capacity": 1},
            "RIGHT_1": {"node_id": "R1", "capacity": 1},
            "RIGHT_2": {"node_id": "R2", "capacity": 1},
        },
        "safe_stop_groups": {
            "LEFT": {"members": ["LEFT_1", "LEFT_2"]},
            "SIDE_1_STOP": {"members": ["SIDE_1"]},
            "SIDE_2_STOP": {"members": ["SIDE_2"]},
            "RIGHT": {"members": ["RIGHT_1", "RIGHT_2"]},
        },
        "blocks": [
            {
                "id": "C1",
                "entry_a": "LEFT",
                "entry_b": "SIDE_1_STOP",
                "direction_domain": "C1",
            },
            {
                "id": "C2",
                "entry_a": "SIDE_1_STOP",
                "entry_b": "SIDE_2_STOP",
                "direction_domain": "C2",
            },
            {
                "id": "C3",
                "entry_a": "SIDE_2_STOP",
                "entry_b": "RIGHT",
                "direction_domain": "C3",
            },
        ],
        "corridor_chains": [
            {
                "id": "MAIN",
                "blocks": ["C1", "C2", "C3"],
                "safe_stops": ["LEFT", "SIDE_1_STOP", "SIDE_2_STOP", "RIGHT"],
                "max_active_robots": 4,
            }
        ],
    }


class CorridorChainRegistryTests(unittest.TestCase):
    def test_resolves_forward_and_reverse_paths_between_physical_slots(self) -> None:
        registry = CorridorRegistry.from_dict(chain_config())

        forward = registry.resolve_chain_path("L2", "R1")
        self.assertIsNotNone(forward)
        assert forward is not None
        self.assertEqual(forward.chain_id, "MAIN")
        self.assertEqual(forward.direction, Direction.A_TO_B)
        self.assertEqual(forward.block_ids, ("C1", "C2", "C3"))
        self.assertEqual(
            forward.safe_stop_ids,
            ("LEFT", "SIDE_1_STOP", "SIDE_2_STOP", "RIGHT"),
        )
        self.assertEqual(forward.source_group, "LEFT")
        self.assertEqual(forward.destination_group, "RIGHT")
        self.assertEqual(forward.source_slot, "LEFT_2")
        self.assertEqual(forward.destination_slot, "RIGHT_1")

        reverse = registry.resolve_chain_path("R2", "S1")
        self.assertIsNotNone(reverse)
        assert reverse is not None
        self.assertEqual(reverse.direction, Direction.B_TO_A)
        self.assertEqual(reverse.block_ids, ("C3", "C2"))
        self.assertEqual(
            reverse.safe_stop_ids,
            ("RIGHT", "SIDE_2_STOP", "SIDE_1_STOP"),
        )
        self.assertEqual(reverse.source_slot, "RIGHT_2")
        self.assertEqual(reverse.destination_slot, "SIDE_1")

    def test_returns_none_for_a_node_outside_the_chain(self) -> None:
        config = chain_config()
        config["holding_bays"]["OTHER"] = {"node_id": "X", "capacity": 1}
        registry = CorridorRegistry.from_dict(config)
        self.assertIsNone(registry.resolve_chain_path("X", "R1"))

    def test_rejects_invalid_chain_shape(self) -> None:
        config = chain_config()
        config["corridor_chains"][0]["safe_stops"] = ["LEFT", "RIGHT"]
        with self.assertRaisesRegex(ValueError, "one more safe stop"):
            CorridorRegistry.from_dict(config)

    def test_rejects_unknown_or_duplicate_group_members(self) -> None:
        unknown = chain_config()
        unknown["safe_stop_groups"]["LEFT"]["members"].append("MISSING")
        with self.assertRaisesRegex(ValueError, "unknown holding bay"):
            CorridorRegistry.from_dict(unknown)

        duplicate = chain_config()
        duplicate["safe_stop_groups"]["RIGHT"]["members"].append("LEFT_1")
        with self.assertRaisesRegex(ValueError, "more than one safe-stop group"):
            CorridorRegistry.from_dict(duplicate)

    def test_rejects_endpoint_mismatch_and_non_positive_limit(self) -> None:
        mismatch = chain_config()
        mismatch["blocks"][1]["entry_a"] = "LEFT"
        with self.assertRaisesRegex(ValueError, "endpoints"):
            CorridorRegistry.from_dict(mismatch)

        invalid_limit = chain_config()
        invalid_limit["corridor_chains"][0]["max_active_robots"] = 0
        with self.assertRaisesRegex(ValueError, "max_active_robots"):
            CorridorRegistry.from_dict(invalid_limit)


if __name__ == "__main__":
    unittest.main()
