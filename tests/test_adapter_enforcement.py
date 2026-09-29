from __future__ import annotations

import unittest

from traffic_control.adapter_enforcement import (
    AdapterAdmissionController,
    PathIntentResolver,
)
from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import Decision, Direction
from traffic_control.robot_tracker import RobotTracker


def make_registry() -> CorridorRegistry:
    return CorridorRegistry.from_dict(
        {
            "traffic_control": {"enabled": True},
            "holding_bays": {
                "HB_A": {
                    "node_id": "A",
                    "geometry": {
                        "circle": {"x": 0.0, "y": 0.0, "radius": 0.4}
                    },
                },
                "HB_B": {
                    "node_id": "D",
                    "geometry": {
                        "circle": {"x": 3.0, "y": 0.0, "radius": 0.4}
                    },
                },
            },
            "blocks": [
                {
                    "id": "CORRIDOR",
                    "entry_a": "HB_A",
                    "entry_b": "HB_B",
                    "direction_domain": "CORRIDOR",
                    "geometry": {
                        "bounds": {
                            "min_x": 0.5,
                            "max_x": 2.5,
                            "min_y": -0.5,
                            "max_y": 0.5,
                        }
                    },
                    "edges_a_to_b": ["A>B", "B>C", "C>D"],
                    "edges_b_to_a": ["D>C", "C>B", "B>A"],
                }
            ],
        }
    )


def state(node: str, x: float, y: float, *, driving: bool) -> dict:
    return {
        "lastNodeId": node,
        "driving": driving,
        "agvPosition": {"x": x, "y": y, "theta": 0.0, "mapId": "L1"},
        "nodeStates": [],
        "edgeStates": [],
    }


class AdapterEnforcementTests(unittest.TestCase):
    def test_resolves_partial_park_path_without_fake_destination_bay(self) -> None:
        registry = make_registry()
        intent = PathIntentResolver(registry).resolve(["D", "C", "X"])

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.block_ids, ("CORRIDOR",))
        self.assertEqual(intent.direction, Direction.B_TO_A)
        self.assertEqual(intent.source_hb, "HB_B")
        self.assertIsNone(intent.destination_hb)
        self.assertEqual(intent.managed_edges, ("D>C",))

    def test_repeated_hsm_retry_is_idempotent(self) -> None:
        registry = make_registry()
        arbiter = DirectionArbiter(registry)
        controller = AdapterAdmissionController(registry, arbiter)

        first = controller.admit(
            robot_id="R1",
            movement_key="order_53",
            path=["D", "C", "X"],
        )
        second = controller.admit(
            robot_id="R1",
            movement_key="order_53",
            path=["D", "C", "X"],
        )

        self.assertEqual(first["decision"], Decision.ADMIT.value)
        self.assertEqual(second["decision"], Decision.ADMIT.value)
        self.assertEqual(list(registry.blocks["CORRIDOR"].reservations), ["R1"])
        authority = arbiter.authority_for_robot("R1")
        assert authority is not None
        self.assertTrue(authority.adapter_managed)
        self.assertEqual(authority.request_key, "order_53")
        self.assertIsNone(authority.destination_slot)

    def test_opposite_adapter_path_waits_then_retries_to_admit(self) -> None:
        registry = make_registry()
        arbiter = DirectionArbiter(registry)
        controller = AdapterAdmissionController(registry, arbiter)

        self.assertEqual(
            controller.admit(
                robot_id="A1",
                movement_key="east",
                path=["A", "B", "C", "D"],
            )["decision"],
            Decision.ADMIT.value,
        )
        self.assertEqual(
            controller.admit(
                robot_id="B1",
                movement_key="west",
                path=["D", "C", "B", "A"],
            )["decision"],
            Decision.WAIT.value,
        )

        arbiter.mark_entered("A1", "CORRIDOR")
        self.assertTrue(arbiter.mark_authority_arrived("A1"))

        self.assertEqual(
            controller.admit(
                robot_id="B1",
                movement_key="west",
                path=["D", "C", "B", "A"],
            )["decision"],
            Decision.ADMIT.value,
        )

    def test_taskgate_authority_is_reused_by_adapter_enforcement(self) -> None:
        registry = make_registry()
        arbiter = DirectionArbiter(registry)
        controller = AdapterAdmissionController(registry, arbiter)

        self.assertIs(
            arbiter.request(
                "R1",
                "CORRIDOR",
                Direction.A_TO_B,
                "HB_B",
                source_hb="HB_A",
            ),
            Decision.ADMIT,
        )

        result = controller.admit(
            robot_id="R1",
            movement_key="rmf-order",
            path=["A", "B", "C", "D"],
        )

        self.assertEqual(result["decision"], Decision.ADMIT.value)
        self.assertEqual(list(registry.blocks["CORRIDOR"].reservations), ["R1"])

    def test_partial_authority_clears_after_stopped_unmanaged_exit(self) -> None:
        registry = make_registry()
        arbiter = DirectionArbiter(registry)
        tracker = RobotTracker(registry, arbiter, telemetry_timeout=5.0)
        controller = AdapterAdmissionController(registry, arbiter)

        tracker.ingest_state(
            "R1", state("D", 3.0, 0.0, driving=False), received_at=1.0
        )
        self.assertEqual(
            controller.admit(
                robot_id="R1",
                movement_key="park-side",
                path=["D", "C", "X"],
            )["decision"],
            Decision.ADMIT.value,
        )

        tracker.ingest_state(
            "R1", state("C", 2.0, 0.0, driving=True), received_at=2.0
        )
        self.assertEqual(
            tracker.snapshot()["R1"]["current_block"], "CORRIDOR"
        )
        self.assertNotIn("R1", registry.holding_bays["HB_B"].occupants)

        tracker.ingest_state(
            "R1", state("X", 2.0, 1.0, driving=False), received_at=3.0
        )

        self.assertIsNone(arbiter.authority_for_robot("R1"))
        self.assertIsNone(tracker.snapshot()["R1"]["current_block"])
        self.assertNotIn("R1", registry.holding_bays["HB_A"].occupants)
        self.assertNotIn("R1", registry.holding_bays["HB_B"].occupants)

    def test_cancel_only_removes_matching_adapter_authority(self) -> None:
        registry = make_registry()
        arbiter = DirectionArbiter(registry)
        controller = AdapterAdmissionController(registry, arbiter)

        controller.admit(
            robot_id="R1",
            movement_key="order-a",
            path=["D", "C", "X"],
        )

        self.assertFalse(
            controller.cancel(
                robot_id="R1", movement_key="different"
            )["cancelled"]
        )
        self.assertIsNotNone(arbiter.authority_for_robot("R1"))
        self.assertTrue(
            controller.cancel(
                robot_id="R1", movement_key="order-a"
            )["cancelled"]
        )
        self.assertIsNone(arbiter.authority_for_robot("R1"))


if __name__ == "__main__":
    unittest.main()
