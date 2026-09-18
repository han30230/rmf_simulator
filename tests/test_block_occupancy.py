from __future__ import annotations

import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import Direction
from traffic_control.robot_tracker import RobotTracker


def make_components() -> tuple[DirectionArbiter, RobotTracker]:
    registry = CorridorRegistry.from_dict(
        {
            "holding_bays": {
                "HB0": {
                    "node_id": "N0",
                    "capacity": 2,
                    "geometry": {"circle": {"x": 0.0, "y": 0.0, "radius": 0.5}},
                },
                "HB1": {
                    "node_id": "N1",
                    "capacity": 2,
                    "geometry": {"circle": {"x": 10.0, "y": 0.0, "radius": 0.5}},
                },
            },
            "blocks": [
                {
                    "id": "TOP_1",
                    "group_id": "TOP",
                    "direction_domain": "TOP",
                    "entry_a": "HB0",
                    "entry_b": "HB1",
                    "geometry": {
                        "bounds": {
                            "min_x": 0.6,
                            "max_x": 9.4,
                            "min_y": -1.0,
                            "max_y": 1.0,
                        }
                    },
                    "edges_a_to_b": ["E01"],
                    "edges_b_to_a": ["E10"],
                }
            ],
        }
    )
    arbiter = DirectionArbiter(registry)
    return arbiter, RobotTracker(registry, arbiter, telemetry_timeout=5.0)


def state(x: float, *, last_node: str = "", driving: bool = True) -> dict:
    return {
        "serialNumber": "A1",
        "lastNodeId": last_node,
        "driving": driving,
        "agvPosition": {"x": x, "y": 0.0, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [{"edgeId": "E01", "sequenceId": 1, "released": True}],
    }


def overlapping_source_components(direction: Direction) -> tuple[DirectionArbiter, RobotTracker]:
    registry = CorridorRegistry.from_dict({
        "holding_bays": {
            name: {
                "node_id": node,
                "geometry": {"circle": {"x": x, "y": 0.0, "radius": 0.5}},
            }
            for name, node, x in [("LEFT", "L", 0.0), ("RIGHT", "R", 10.0),
                                  ("UNRELATED", "U", 12.0)]
        },
        "blocks": [
            {
                "id": name, "entry_a": "LEFT", "entry_b": "RIGHT",
                "geometry": {"bounds": {"min_x": low, "max_x": high,
                                         "min_y": -1.0, "max_y": 1.0}},
            }
            for name, low, high in [("OVERLAP", -2.0, 14.0), ("GRANTED", 0.4, 9.6)]
        ],
        "routes": [{
            "id": "cross", "start_nodes": ["L", "R"], "goal_nodes": ["L", "R"],
            "steps": [{
                "block_id": "GRANTED", "direction": direction.value,
                "destination_hb": "RIGHT" if direction is Direction.A_TO_B else "LEFT",
                "goal_node": "R" if direction is Direction.A_TO_B else "L",
                # Exercise the loader's existing direction-based source inference.
            }],
        }],
    })
    arbiter = DirectionArbiter(registry)
    return arbiter, RobotTracker(registry, arbiter)


class BlockOccupancyTests(unittest.TestCase):
    def test_grant_preserves_source_bay_until_departure_in_both_directions(self) -> None:
        for direction, source, source_x, inside_x in [
            (Direction.A_TO_B, "LEFT", 0.0, 1.0),
            (Direction.B_TO_A, "RIGHT", 10.0, 9.0),
        ]:
            with self.subTest(direction=direction):
                arbiter, tracker = overlapping_source_components(direction)
                robot = "test_robot"
                tracker.ingest_state(robot, state(source_x, driving=False), received_at=1.0)
                step = tracker.registry.routes[0].steps[0]
                self.assertEqual(step.source_hb, source)
                decision = arbiter.request(
                    robot, step.block_id, step.direction, step.destination_hb,
                    source_hb=step.source_hb,
                )
                self.assertEqual(decision.value, "ADMIT")
                for now, x in [(2.0, source_x), (3.0, source_x),
                               (4.0, 0.45 if source_x == 0.0 else 9.55)]:
                    tracker.ingest_state(robot, state(x, driving=False), received_at=now)
                    self.assertEqual(tracker.snapshot()[robot]["current_hb"], source)
                    self.assertIsNone(tracker.snapshot()[robot]["current_block"])
                    snapshot = arbiter.snapshot()
                    self.assertEqual(snapshot["blocks"]["GRANTED"]["reservations"], [robot])
                    for block in snapshot["blocks"].values():
                        self.assertIsNone(block["fault_reason"])
                        self.assertEqual(block["occupants"], [])

                tracker.ingest_state(robot, state(inside_x), received_at=5.0)
                self.assertEqual(tracker.snapshot()[robot]["current_block"], "GRANTED")
                self.assertIsNone(tracker.snapshot()[robot]["current_hb"])
                snapshot = arbiter.snapshot()
                self.assertEqual(snapshot["blocks"]["GRANTED"]["occupants"], [robot])
                self.assertEqual(snapshot["holding_bays"][source]["occupants"], [])
                self.assertIsNone(snapshot["blocks"]["OVERLAP"]["fault_reason"])

    def test_source_grant_does_not_hide_unreserved_entry_or_unrelated_bay(self) -> None:
        for x in (-1.0, 12.0):
            with self.subTest(x=x):
                arbiter, tracker = overlapping_source_components(Direction.A_TO_B)
                tracker.ingest_state("test_robot", state(0.0, driving=False), received_at=1.0)
                arbiter.request("test_robot", "GRANTED", Direction.A_TO_B, "RIGHT", source_hb="LEFT")
                tracker.ingest_state("test_robot", state(x), received_at=2.0)
                self.assertEqual(tracker.snapshot()["test_robot"]["current_block"], "OVERLAP")
                block = arbiter.snapshot()["blocks"]["OVERLAP"]
                self.assertEqual(block["state"], "BLOCKED")
                self.assertEqual(block["fault_reason"], "unreserved_robot_detected_inside")

    def test_intermediate_holding_bay_does_not_finalize_active_grant(self) -> None:
        registry = CorridorRegistry.from_dict(
            {
                "holding_bays": {
                    "HB0": {
                        "node_id": "N0",
                        "geometry": {"circle": {"x": 0.0, "y": 0.0, "radius": 0.5}},
                    },
                    "HB_MID": {
                        "node_id": "MID",
                        "geometry": {"circle": {"x": 7.0, "y": 0.0, "radius": 0.5}},
                    },
                    "HB1": {
                        "node_id": "N1",
                        "geometry": {"circle": {"x": 10.0, "y": 0.0, "radius": 0.5}},
                    },
                },
                "blocks": [
                    {
                        "id": "TOP_1",
                        "entry_a": "HB0",
                        "entry_b": "HB1",
                        "geometry": {
                            "bounds": {
                                "min_x": 0.6,
                                "max_x": 6.4,
                                "min_y": -1.0,
                                "max_y": 1.0,
                            }
                        },
                    }
                ],
            }
        )
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter)
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1", source_hb="HB0")
        tracker.ingest_state("A1", state(5.0), received_at=1.0)

        tracker.ingest_state(
            "A1", state(7.0, last_node="MID", driving=False), received_at=2.0
        )

        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "TOP_1")
        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], ["A1"])
        self.assertEqual(arbiter.snapshot()["holding_bays"]["HB1"]["occupants"], [])

    def test_exact_release_node_clears_block_while_robot_keeps_driving(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request(
            "A1",
            "TOP_1",
            Direction.A_TO_B,
            "HB1",
            source_hb="HB0",
            release_node="N_CLEAR",
        )
        tracker.ingest_state(
            "A1", state(5.0, last_node="N_BEFORE"), received_at=1.0
        )
        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "TOP_1")

        tracker.ingest_state(
            "A1",
            state(9.0, last_node="N_CLEAR", driving=True),
            received_at=2.0,
        )

        snapshot = tracker.snapshot()["A1"]
        self.assertIsNone(snapshot["current_block"])
        self.assertTrue(snapshot["driving"])
        status = arbiter.snapshot()
        self.assertEqual(status["blocks"]["TOP_1"]["occupants"], [])
        self.assertEqual(status["holding_bays"]["HB1"]["reservations"], ["A1"])

        tracker.ingest_state(
            "A1",
            state(9.0, last_node="N_CLEAR", driving=True),
            received_at=2.5,
        )
        self.assertIsNone(tracker.snapshot()["A1"]["current_block"])
        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], [])

    def test_missing_release_node_telemetry_remains_fail_closed(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request(
            "A1",
            "TOP_1",
            Direction.A_TO_B,
            "HB1",
            source_hb="HB0",
            release_node="N_CLEAR",
        )
        tracker.ingest_state(
            "A1", state(5.0, last_node="N_BEFORE"), received_at=1.0
        )
        tracker.ingest_state(
            "A1", state(50.0, last_node="N_BEFORE"), received_at=2.0
        )

        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "TOP_1")
        self.assertEqual(tracker.expire_stale(now=8.0), ["A1"])
        status = arbiter.snapshot()["blocks"]["TOP_1"]
        self.assertEqual(status["state"], "BLOCKED")
        self.assertEqual(status["occupants"], ["A1"])

    def test_unreserved_robot_at_holding_bay_wins_over_overlapping_block(self) -> None:
        registry = CorridorRegistry.from_dict(
            {
                "holding_bays": {
                    "HB_SAFE": {
                        "node_id": "SAFE",
                        "capacity": 1,
                        "geometry": {
                            "circle": {"x": 5.0, "y": 0.0, "radius": 0.5}
                        },
                    },
                    "HB_END": {"node_id": "END", "capacity": 1},
                },
                "blocks": [
                    {
                        "id": "OVERLAP",
                        "entry_a": "HB_SAFE",
                        "entry_b": "HB_END",
                        "geometry": {
                            "bounds": {
                                "min_x": 1.0,
                                "max_x": 9.0,
                                "min_y": -1.0,
                                "max_y": 1.0,
                            }
                        },
                    }
                ],
            }
        )
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter)

        tracker.ingest_state(
            "A1", state(5.0, last_node="SAFE", driving=False), received_at=1.0
        )

        self.assertEqual(tracker.current_safe_node("A1"), "SAFE")
        self.assertIsNone(tracker.snapshot()["A1"]["current_block"])
        self.assertIsNone(arbiter.snapshot()["blocks"]["OVERLAP"]["fault_reason"])

    def test_overlapping_geometry_prefers_the_robot_granted_block(self) -> None:
        registry = CorridorRegistry.from_dict(
            {
                "holding_bays": {
                    "HB0": {"node_id": "N0", "capacity": 2},
                    "HB1": {"node_id": "N1", "capacity": 2},
                    "HB2": {"node_id": "N2", "capacity": 2},
                },
                "blocks": [
                    {
                        "id": "FIRST_MATCH",
                        "entry_a": "HB0",
                        "entry_b": "HB1",
                        "geometry": {
                            "bounds": {
                                "min_x": 1.0,
                                "max_x": 9.0,
                                "min_y": -1.0,
                                "max_y": 1.0,
                            }
                        },
                    },
                    {
                        "id": "GRANTED_MATCH",
                        "entry_a": "HB0",
                        "entry_b": "HB2",
                        "geometry": {
                            "bounds": {
                                "min_x": 1.0,
                                "max_x": 9.0,
                                "min_y": -1.0,
                                "max_y": 1.0,
                            }
                        },
                    },
                ],
            }
        )
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter)
        arbiter.request("A1", "GRANTED_MATCH", Direction.A_TO_B, "HB2")

        tracker.ingest_state("A1", state(5.0), received_at=1.0)

        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "GRANTED_MATCH")
        self.assertEqual(
            arbiter.snapshot()["blocks"]["GRANTED_MATCH"]["occupants"],
            ["A1"],
        )
        self.assertIsNone(
            arbiter.snapshot()["blocks"]["FIRST_MATCH"]["fault_reason"]
        )

    def test_position_crossing_marks_enter_and_exit(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1", source_hb="HB0")

        tracker.ingest_state("A1", state(5.0), received_at=1.0)
        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], ["A1"])

        tracker.ingest_state("A1", state(10.0, last_node="N1", driving=False), received_at=2.0)
        status = arbiter.snapshot()
        self.assertEqual(status["blocks"]["TOP_1"]["occupants"], [])
        self.assertEqual(status["holding_bays"]["HB1"]["occupants"], ["A1"])
        self.assertEqual(tracker.current_safe_node("A1"), "N1")

    def test_last_node_alone_does_not_clear_robot_still_inside_geometry(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        tracker.ingest_state("A1", state(5.0), received_at=1.0)

        tracker.ingest_state("A1", state(5.0, last_node="N1", driving=False), received_at=2.0)

        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], ["A1"])

    def test_initial_holding_bay_position_infers_safe_node(self) -> None:
        _, tracker = make_components()

        tracker.ingest_state("A1", state(0.0, last_node="", driving=False), received_at=1.0)

        self.assertEqual(tracker.current_safe_node("A1"), "N0")

    def test_missing_position_does_not_create_false_exit(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        tracker.ingest_state("A1", state(5.0), received_at=1.0)

        no_position = state(5.0, last_node="N1", driving=False)
        no_position.pop("agvPosition")
        tracker.ingest_state("A1", no_position, received_at=2.0)

        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], ["A1"])

    def test_edge_state_can_confirm_entry_when_position_is_missing(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        no_position = state(5.0)
        no_position.pop("agvPosition")

        tracker.ingest_state("A1", no_position, received_at=1.0)

        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], ["A1"])

    def test_unknown_position_after_entry_remains_unresolved_inside(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        tracker.ingest_state("A1", state(5.0), received_at=1.0)

        tracker.ingest_state("A1", state(50.0), received_at=2.0)

        self.assertEqual(tracker.snapshot()["A1"]["current_block"], "TOP_1")
        self.assertEqual(tracker.expire_stale(now=8.0), ["A1"])


if __name__ == "__main__":
    unittest.main()
