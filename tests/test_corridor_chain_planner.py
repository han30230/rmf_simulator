from __future__ import annotations

import unittest

from traffic_control.corridor_chain import CorridorChainPlanner
from traffic_control.corridor_registry import CorridorRegistry

from tests.test_corridor_chain_registry import chain_config


class CorridorChainPlannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = CorridorRegistry.from_dict(chain_config())
        path = self.registry.resolve_chain_path("L2", "R1")
        assert path is not None
        self.path = path
        self.planner = CorridorChainPlanner(self.registry)

    def test_clear_chain_uses_one_leg_to_the_requested_final_slot(self) -> None:
        leg = self.planner.plan(
            self.path,
            lambda block_ids, direction, slot: True,
        )

        self.assertIsNotNone(leg)
        assert leg is not None
        self.assertEqual(leg.block_ids, ("C1", "C2", "C3"))
        self.assertEqual(leg.destination_group, "RIGHT")
        self.assertEqual(leg.destination_slot, "RIGHT_1")
        self.assertEqual(leg.goal_node, "R1")

    def test_conflict_selects_the_farthest_reachable_side_bay(self) -> None:
        leg = self.planner.plan(
            self.path,
            lambda block_ids, direction, slot: "C3" not in block_ids,
        )

        self.assertIsNotNone(leg)
        assert leg is not None
        self.assertEqual(leg.block_ids, ("C1", "C2"))
        self.assertEqual(leg.destination_group, "SIDE_2_STOP")
        self.assertEqual(leg.destination_slot, "SIDE_2")
        self.assertEqual(leg.goal_node, "S2")

    def test_nearer_side_bay_is_used_when_the_middle_block_conflicts(self) -> None:
        leg = self.planner.plan(
            self.path,
            lambda block_ids, direction, slot: "C2" not in block_ids,
        )

        self.assertIsNotNone(leg)
        assert leg is not None
        self.assertEqual(leg.block_ids, ("C1",))
        self.assertEqual(leg.destination_group, "SIDE_1_STOP")
        self.assertEqual(leg.destination_slot, "SIDE_1")
        self.assertEqual(leg.goal_node, "S1")

    def test_no_reachable_safe_stop_returns_none(self) -> None:
        leg = self.planner.plan(
            self.path,
            lambda block_ids, direction, slot: False,
        )
        self.assertIsNone(leg)

    def test_intermediate_group_tries_physical_slots_in_configuration_order(self) -> None:
        config = chain_config()
        config["holding_bays"]["SIDE_2_B"] = {
            "node_id": "S2B",
            "capacity": 1,
        }
        config["safe_stop_groups"]["SIDE_2_STOP"]["members"].append("SIDE_2_B")
        registry = CorridorRegistry.from_dict(config)
        path = registry.resolve_chain_path("L2", "R1")
        assert path is not None
        attempts: list[str] = []

        def available(block_ids, direction, slot):
            attempts.append(slot)
            if "C3" in block_ids:
                return False
            return slot == "SIDE_2_B"

        leg = CorridorChainPlanner(registry).plan(path, available)

        self.assertIsNotNone(leg)
        assert leg is not None
        self.assertEqual(leg.destination_slot, "SIDE_2_B")
        self.assertEqual(attempts[:3], ["RIGHT_1", "SIDE_2", "SIDE_2_B"])


if __name__ == "__main__":
    unittest.main()
