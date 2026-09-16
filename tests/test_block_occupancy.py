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


class BlockOccupancyTests(unittest.TestCase):
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
