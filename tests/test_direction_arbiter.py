from __future__ import annotations

import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import BlockState, Decision, Direction


def make_registry(*, block_capacity: int = 1) -> CorridorRegistry:
    return CorridorRegistry.from_dict(
        {
            "traffic_control": {"enabled": True},
            "holding_bays": {
                "HB0": {"node_id": "N0", "capacity": 2},
                "HB1": {"node_id": "N1", "capacity": 2},
                "HB2": {"node_id": "N2", "capacity": 2},
                "HB3": {"node_id": "N3", "capacity": 2},
            },
            "blocks": [
                {
                    "id": "TOP_1",
                    "group_id": "TOP",
                    "direction_domain": "TOP",
                    "entry_a": "HB0",
                    "entry_b": "HB1",
                    "capacity": block_capacity,
                },
                {
                    "id": "TOP_2",
                    "group_id": "TOP",
                    "direction_domain": "TOP",
                    "entry_a": "HB1",
                    "entry_b": "HB2",
                    "capacity": block_capacity,
                },
                {
                    "id": "YARD_1",
                    "group_id": "YARD",
                    "direction_domain": "YARD",
                    "entry_a": "HB2",
                    "entry_b": "HB3",
                    "capacity": block_capacity,
                },
            ],
        }
    )


class DirectionArbiterTests(unittest.TestCase):
    def test_route_direction_must_match_block_endpoints(self) -> None:
        with self.assertRaisesRegex(ValueError, "does not match"):
            CorridorRegistry.from_dict(
                {
                    "holding_bays": {
                        "HB0": {"node_id": "N0"},
                        "HB1": {"node_id": "N1"},
                    },
                    "blocks": [
                        {"id": "TOP_1", "entry_a": "HB0", "entry_b": "HB1"}
                    ],
                    "routes": [
                        {
                            "start_nodes": ["N0"],
                            "goal_nodes": ["N1"],
                            "steps": [
                                {
                                    "block_id": "TOP_1",
                                    "direction": "A_TO_B",
                                    "source_hb": "HB1",
                                    "destination_hb": "HB0",
                                    "goal_node": "N1",
                                }
                            ],
                        }
                    ],
                }
            )

    def test_nested_spec_configuration_defaults_to_independent_blocks(self) -> None:
        registry = CorridorRegistry.from_dict(
            {
                "traffic_control": {
                    "enabled": True,
                    "holding_bays": {
                        "HB0": {"node_id": "N0"},
                        "HB1": {"node_id": "N1"},
                        "HB2": {"node_id": "N2"},
                        "HB3": {"node_id": "N3"},
                    },
                    "groups": {
                        "TOP": {
                            "blocks": [
                                {"id": "TOP_1", "entry_a": "HB0", "entry_b": "HB1"},
                                {"id": "TOP_2", "entry_a": "HB1", "entry_b": "HB2"},
                                {"id": "TOP_3", "entry_a": "HB2", "entry_b": "HB3"},
                            ]
                        }
                    },
                }
            }
        )

        self.assertEqual(registry.blocks["TOP_1"].group_id, "TOP")
        self.assertEqual(registry.blocks["TOP_1"].direction_domain, "TOP_1")
        self.assertEqual(registry.blocks["TOP_2"].direction_domain, "TOP_2")

        arbiter = DirectionArbiter(registry)
        self.assertIs(
            arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1"),
            Decision.ADMIT,
        )
        self.assertIs(
            arbiter.request("B1", "TOP_3", Direction.B_TO_A, "HB2"),
            Decision.ADMIT,
        )

    def test_first_direction_is_admitted_and_opposite_waits(self) -> None:
        arbiter = DirectionArbiter(make_registry())

        self.assertIs(
            arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1", source_hb="HB0"),
            Decision.ADMIT,
        )
        self.assertIs(
            arbiter.request("B1", "TOP_1", Direction.B_TO_A, "HB0", source_hb="HB1"),
            Decision.WAIT,
        )

        status = arbiter.snapshot()
        self.assertEqual(status["domains"]["TOP"]["active_direction"], "A_TO_B")
        self.assertTrue(status["domains"]["TOP"]["batch_closed"])
        self.assertEqual(status["blocks"]["TOP_1"]["waiting"]["B_TO_A"], ["B1"])


    def test_block_and_destination_holding_bay_are_reserved_atomically(self) -> None:
        arbiter = DirectionArbiter(make_registry())
        arbiter.occupy_holding_bay("HB1", "PARKED")
        arbiter.occupy_holding_bay("HB1", "PARKED_2")

        decision = arbiter.request(
            "A1", "TOP_1", Direction.A_TO_B, "HB1", source_hb="HB0"
        )

        self.assertIs(decision, Decision.WAIT)
        status = arbiter.snapshot()
        self.assertEqual(status["blocks"]["TOP_1"]["reservations"], [])
        self.assertEqual(status["holding_bays"]["HB1"]["reservations"], [])


    def test_opposite_waiter_closes_current_batch(self) -> None:
        arbiter = DirectionArbiter(make_registry(block_capacity=2))
        self.assertIs(arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1"), Decision.ADMIT)
        self.assertIs(arbiter.request("B1", "TOP_1", Direction.B_TO_A, "HB0"), Decision.WAIT)

        self.assertIs(arbiter.request("A2", "TOP_1", Direction.A_TO_B, "HB1"), Decision.WAIT)

        status = arbiter.snapshot()
        self.assertTrue(status["domains"]["TOP"]["batch_closed"])
        self.assertEqual(status["blocks"]["TOP_1"]["waiting"]["A_TO_B"], ["A2"])


    def test_clear_domain_switches_direction_and_grants_oldest_opposite(self) -> None:
        arbiter = DirectionArbiter(make_registry())
        self.assertIs(arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1"), Decision.ADMIT)
        self.assertIs(arbiter.request("B1", "TOP_1", Direction.B_TO_A, "HB0"), Decision.WAIT)
        arbiter.mark_entered("A1", "TOP_1")

        arbiter.mark_exited("A1", "TOP_1")

        self.assertIs(arbiter.decision_for("B1", "TOP_1"), Decision.ADMIT)
        status = arbiter.snapshot()
        self.assertEqual(status["domains"]["TOP"]["active_direction"], "B_TO_A")
        self.assertEqual(status["blocks"]["TOP_1"]["reservations"], ["B1"])


    def test_independent_direction_domains_can_run_opposite_directions(self) -> None:
        arbiter = DirectionArbiter(make_registry())

        self.assertIs(arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1"), Decision.ADMIT)
        self.assertIs(arbiter.request("B1", "YARD_1", Direction.B_TO_A, "HB2"), Decision.ADMIT)

        status = arbiter.snapshot()
        self.assertEqual(status["domains"]["TOP"]["active_direction"], "A_TO_B")
        self.assertEqual(status["domains"]["YARD"]["active_direction"], "B_TO_A")


    def test_fault_inside_block_is_fail_closed(self) -> None:
        arbiter = DirectionArbiter(make_registry())
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        arbiter.mark_entered("A1", "TOP_1")

        arbiter.fault("A1", reason="telemetry_timeout")

        self.assertIs(arbiter.request("B1", "TOP_1", Direction.B_TO_A, "HB0"), Decision.BLOCKED)
        status = arbiter.snapshot()
        self.assertEqual(status["blocks"]["TOP_1"]["state"], BlockState.BLOCKED.value)
        self.assertEqual(status["blocks"]["TOP_1"]["occupants"], ["A1"])
        self.assertEqual(status["blocks"]["TOP_1"]["fault_reason"], "telemetry_timeout")


    def test_reset_requires_force_when_robot_is_inside(self) -> None:
        arbiter = DirectionArbiter(make_registry())
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        arbiter.mark_entered("A1", "TOP_1")

        self.assertFalse(arbiter.reset())
        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], ["A1"])

        self.assertTrue(arbiter.reset(force=True))
        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], [])


    def test_unknown_block_is_rejected_without_partial_reservation(self) -> None:
        arbiter = DirectionArbiter(make_registry())

        with self.assertRaises(KeyError):
            arbiter.request("A1", "MISSING", Direction.A_TO_B, "HB1")

        self.assertEqual(arbiter.snapshot()["holding_bays"]["HB1"]["reservations"], [])
