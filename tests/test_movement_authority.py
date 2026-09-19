from __future__ import annotations

import unittest

from traffic_control.corridor_chain import CorridorChainPlanner, PlannedAuthority
from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import Decision, Direction

from tests.test_corridor_chain_registry import chain_config


def components(*, block_capacity: int = 1):
    config = chain_config()
    for block in config["blocks"]:
        block["capacity"] = block_capacity
    registry = CorridorRegistry.from_dict(config)
    return registry, DirectionArbiter(registry), CorridorChainPlanner(registry)


def planned(
    registry: CorridorRegistry,
    planner: CorridorChainPlanner,
    start: str,
    goal: str,
) -> PlannedAuthority:
    path = registry.resolve_chain_path(start, goal)
    assert path is not None
    result = planner.plan(path, lambda block_ids, direction, slot: True)
    assert result is not None
    return result


class MovementAuthorityTests(unittest.TestCase):
    def test_reserves_the_complete_interval_and_destination_atomically(self) -> None:
        registry, arbiter, planner = components()
        authority = planned(registry, planner, "L1", "R1")

        decision = arbiter.request_authority(
            authority,
            robot_id="A1",
            request_time=1.0,
        )

        self.assertEqual(decision, Decision.ADMIT)
        granted = arbiter.authority_for_robot("A1")
        self.assertIsNotNone(granted)
        assert granted is not None
        self.assertEqual(granted.unreleased_blocks, ("C1", "C2", "C3"))
        for block_id in granted.block_ids:
            self.assertIn("A1", registry.blocks[block_id].reservations)
        self.assertEqual(registry.holding_bays["RIGHT_1"].reservations, {"A1"})

    def test_conflict_on_last_block_leaves_no_partial_block_reservation(self) -> None:
        registry, arbiter, planner = components()
        blocker = planned(registry, planner, "R2", "S2")
        self.assertEqual(
            arbiter.request_authority(blocker, robot_id="B1", request_time=1.0),
            Decision.ADMIT,
        )

        candidate = planned(registry, planner, "L1", "R1")
        self.assertEqual(
            arbiter.request_authority(candidate, robot_id="A1", request_time=2.0),
            Decision.WAIT,
        )

        self.assertNotIn("A1", registry.blocks["C1"].reservations)
        self.assertNotIn("A1", registry.blocks["C2"].reservations)
        self.assertNotIn("A1", registry.blocks["C3"].reservations)
        self.assertNotIn("A1", registry.holding_bays["RIGHT_1"].reservations)

    def test_same_direction_authorities_pipeline_with_configured_capacity(self) -> None:
        registry, arbiter, planner = components(block_capacity=2)
        a1 = planned(registry, planner, "L1", "R1")
        a2 = planned(registry, planner, "L2", "R2")

        self.assertEqual(
            arbiter.request_authority(a1, robot_id="A1", request_time=1.0),
            Decision.ADMIT,
        )
        self.assertEqual(
            arbiter.request_authority(a2, robot_id="A2", request_time=2.0),
            Decision.ADMIT,
        )
        self.assertEqual(set(registry.blocks["C2"].reservations), {"A1", "A2"})

    def test_opposite_directions_can_reserve_disjoint_end_segments(self) -> None:
        registry, arbiter, planner = components()
        eastbound = planned(registry, planner, "L1", "S1")
        westbound = planned(registry, planner, "R1", "S2")

        self.assertEqual(
            arbiter.request_authority(eastbound, robot_id="A1", request_time=1.0),
            Decision.ADMIT,
        )
        self.assertEqual(
            arbiter.request_authority(westbound, robot_id="B1", request_time=2.0),
            Decision.ADMIT,
        )
        self.assertEqual(registry.blocks["C1"].state.value, "A_TO_B")
        self.assertEqual(registry.blocks["C3"].state.value, "B_TO_A")

    def test_opposite_overlapping_interval_waits_without_partial_reservation(self) -> None:
        registry, arbiter, planner = components()
        eastbound = planned(registry, planner, "L1", "S2")
        westbound = planned(registry, planner, "R1", "S1")

        self.assertEqual(
            arbiter.request_authority(eastbound, robot_id="A1", request_time=1.0),
            Decision.ADMIT,
        )
        self.assertEqual(
            arbiter.request_authority(westbound, robot_id="B1", request_time=2.0),
            Decision.WAIT,
        )
        self.assertNotIn("B1", registry.blocks["C3"].reservations)
        self.assertNotIn("B1", registry.blocks["C2"].reservations)
        self.assertEqual(arbiter.decision_for_authority("B1"), Decision.WAIT)

    def test_cancellation_releases_every_resource_in_the_authority(self) -> None:
        registry, arbiter, planner = components()
        candidate = planned(registry, planner, "L1", "R1")
        arbiter.request_authority(candidate, robot_id="A1", request_time=1.0)

        self.assertTrue(arbiter.cancel_authority("A1"))

        self.assertIsNone(arbiter.authority_for_robot("A1"))
        for block in registry.blocks.values():
            self.assertNotIn("A1", block.reservations)
        self.assertNotIn("A1", registry.holding_bays["RIGHT_1"].reservations)

    def test_arrival_grants_the_oldest_waiting_opposite_authority(self) -> None:
        registry, arbiter, planner = components()
        eastbound = planned(registry, planner, "L1", "S2")
        westbound = planned(registry, planner, "R1", "S1")
        arbiter.request_authority(eastbound, robot_id="A1", request_time=1.0)
        arbiter.request_authority(westbound, robot_id="B1", request_time=2.0)

        self.assertTrue(arbiter.mark_authority_arrived("A1"))

        self.assertEqual(arbiter.decision_for_authority("B1"), Decision.ADMIT)
        self.assertIn("B1", registry.blocks["C2"].reservations)
        self.assertEqual(registry.holding_bays["SIDE_1"].reservations, {"B1"})


if __name__ == "__main__":
    unittest.main()
