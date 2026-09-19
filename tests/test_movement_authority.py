from __future__ import annotations

import unittest

from traffic_control.corridor_chain import CorridorChainPlanner, PlannedAuthority
from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import Decision, Direction
from traffic_control.robot_tracker import RobotTracker

from tests.test_corridor_chain_registry import chain_config


def components(*, block_capacity: int = 1):
    config = chain_config()
    for block in config["blocks"]:
        block["capacity"] = block_capacity
    registry = CorridorRegistry.from_dict(config)
    return registry, DirectionArbiter(registry), CorridorChainPlanner(registry)


def tracking_components():
    config = chain_config()
    positions = {
        "LEFT_1": (-1.0, 1.0),
        "LEFT_2": (-1.0, 2.0),
        "SIDE_1": (10.0, 2.0),
        "SIDE_2": (20.0, 2.0),
        "RIGHT_1": (31.0, 1.0),
        "RIGHT_2": (31.0, 2.0),
    }
    for hb_id, (x, y) in positions.items():
        config["holding_bays"][hb_id]["geometry"] = {
            "circle": {"x": x, "y": y, "radius": 0.4}
        }
    for block, bounds, forward_release, reverse_release in zip(
        config["blocks"],
        ((0.0, 10.0), (10.0, 20.0), (20.0, 30.0)),
        ("N1", "N2", "N3"),
        ("N0", "N1", "N2"),
    ):
        low, high = bounds
        block["geometry"] = {
            "bounds": {
                "min_x": low,
                "max_x": high,
                "min_y": -0.5,
                "max_y": 0.5,
            }
        }
        block["release_node_a_to_b"] = forward_release
        block["release_node_b_to_a"] = reverse_release
    registry = CorridorRegistry.from_dict(config)
    arbiter = DirectionArbiter(registry)
    tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)
    return registry, arbiter, CorridorChainPlanner(registry), tracker


def state(node: str, x: float, y: float, *, driving: bool = True) -> dict:
    return {
        "lastNodeId": node,
        "driving": driving,
        "agvPosition": {"x": x, "y": y, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [],
    }


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

    def test_tracker_releases_blocks_in_order_and_keeps_destination_reserved(self) -> None:
        registry, arbiter, planner, tracker = tracking_components()
        tracker.ingest_state("A1", state("L1", -1.0, 1.0, driving=False), received_at=1.0)
        authority = planned(registry, planner, "L1", "R1")
        arbiter.request_authority(authority, robot_id="A1", request_time=1.0)

        tracker.ingest_state("A1", state("N0", 5.0, 0.0), received_at=2.0)
        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "C1")

        tracker.ingest_state("A1", state("N1", 10.0, 0.0), received_at=3.0)
        granted = arbiter.authority_for_robot("A1")
        assert granted is not None
        self.assertEqual(granted.unreleased_blocks, ("C2", "C3"))
        self.assertEqual(registry.holding_bays["RIGHT_1"].reservations, {"A1"})

        tracker.ingest_state("A1", state("", 15.0, 0.0), received_at=4.0)
        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "C2")
        tracker.ingest_state("A1", state("N2", 20.0, 0.0), received_at=5.0)
        self.assertEqual(granted.unreleased_blocks, ("C3",))
        self.assertEqual(registry.holding_bays["RIGHT_1"].reservations, {"A1"})

        tracker.ingest_state("A1", state("", 25.0, 0.0), received_at=6.0)
        tracker.ingest_state("A1", state("R1", 31.0, 1.0, driving=False), received_at=7.0)

        self.assertIsNone(arbiter.authority_for_robot("A1"))
        self.assertEqual(registry.holding_bays["RIGHT_1"].occupants, {"A1"})
        for block in registry.blocks.values():
            self.assertNotIn("A1", block.occupants)
            self.assertNotIn("A1", block.reservations)

    def test_timeout_faults_every_unreleased_authority_block(self) -> None:
        registry, arbiter, planner, tracker = tracking_components()
        tracker.ingest_state("A1", state("L1", -1.0, 1.0, driving=False), received_at=1.0)
        authority = planned(registry, planner, "L1", "R1")
        arbiter.request_authority(authority, robot_id="A1", request_time=1.0)
        tracker.ingest_state("A1", state("N0", 5.0, 0.0), received_at=2.0)

        self.assertEqual(tracker.expire_stale(now=8.0), ["A1"])

        self.assertEqual(
            {block_id for block_id, block in registry.blocks.items() if block.fault_reason},
            {"C1", "C2", "C3"},
        )

    def test_final_block_stays_reserved_until_side_bay_arrival(self) -> None:
        registry, arbiter, planner, tracker = tracking_components()
        tracker.ingest_state("A1", state("L1", -1.0, 1.0, driving=False), received_at=1.0)
        authority = planned(registry, planner, "L1", "S2")
        arbiter.request_authority(authority, robot_id="A1", request_time=1.0)

        tracker.ingest_state("A1", state("N0", 5.0, 0.0), received_at=2.0)
        tracker.ingest_state("A1", state("N1", 10.0, 0.0), received_at=3.0)
        tracker.ingest_state("A1", state("", 15.0, 0.0), received_at=4.0)
        tracker.ingest_state("A1", state("N2", 20.0, 0.0), received_at=5.0)

        granted = arbiter.authority_for_robot("A1")
        assert granted is not None
        self.assertEqual(granted.unreleased_blocks, ("C2",))
        self.assertIn("A1", registry.blocks["C2"].occupants)

        tracker.ingest_state("A1", state("S2", 20.0, 2.0, driving=False), received_at=6.0)
        self.assertIsNone(arbiter.authority_for_robot("A1"))
        self.assertEqual(registry.holding_bays["SIDE_2"].occupants, {"A1"})


if __name__ == "__main__":
    unittest.main()
