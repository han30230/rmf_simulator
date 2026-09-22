from __future__ import annotations

import unittest

from traffic_control.deployment import RobotDeploymentConfig
from traffic_control.eligibility import RobotEligibilityPolicy
from traffic_control.models import BlockState, Decision, Direction
from traffic_control.robot_tracker import RobotTracker

from tests.test_block_occupancy import make_components, state


class FaultSafetyTests(unittest.TestCase):
    @staticmethod
    def operational_tracker(arbiter) -> RobotTracker:
        policy = RobotEligibilityPolicy(
            robots={
                "A1": RobotDeploymentConfig(
                    manufacturer="vendor",
                    serial_number="A1",
                    allowed_map_ids=("L1",),
                )
            },
            state_timeout=5.0,
            connection_timeout=5.0,
        )
        return RobotTracker(
            arbiter.registry,
            arbiter,
            telemetry_timeout=5.0,
            eligibility_policy=policy,
        )

    def test_telemetry_timeout_inside_block_blocks_opposite_direction(self) -> None:
        arbiter, tracker = make_components()
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        tracker.ingest_state("A1", state(5.0), received_at=1.0)

        expired = tracker.expire_stale(now=7.0)

        self.assertEqual(expired, ["A1"])
        self.assertEqual(
            arbiter.snapshot()["blocks"]["TOP_1"]["state"],
            BlockState.BLOCKED.value,
        )
        self.assertIs(
            arbiter.request("B1", "TOP_1", Direction.B_TO_A, "HB0"),
            Decision.BLOCKED,
        )

    def test_timeout_outside_block_does_not_block_corridor(self) -> None:
        arbiter, tracker = make_components()
        tracker.ingest_state("A1", state(0.0, driving=False), received_at=1.0)

        expired = tracker.expire_stale(now=7.0)

        self.assertEqual(expired, [])
        self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["state"], "FREE")

    def test_offline_inside_block_faults_unreleased_authority(self) -> None:
        arbiter, _ = make_components()
        tracker = self.operational_tracker(arbiter)
        arbiter.request("A1", "TOP_1", Direction.A_TO_B, "HB1")
        tracker.ingest_connection(
            "A1",
            {
                "headerId": 1,
                "timestamp": "2026-09-20T00:00:00Z",
                "manufacturer": "vendor",
                "serialNumber": "A1",
                "connectionState": "ONLINE",
            },
            received_at=1.0,
        )
        inside = state(5.0)
        inside.update({
            "headerId": 1,
            "timestamp": "2026-09-20T00:00:01Z",
            "manufacturer": "vendor",
            "serialNumber": "A1",
            "operatingMode": "AUTOMATIC",
            "paused": False,
            "safetyState": {"eStop": "NONE", "fieldViolation": False},
            "errors": [],
        })
        inside["agvPosition"].update({
            "mapId": "L1", "positionInitialized": True
        })
        tracker.ingest_state("A1", inside, received_at=1.0)

        tracker.ingest_connection(
            "A1",
            {
                "headerId": 2,
                "timestamp": "2026-09-20T00:00:02Z",
                "manufacturer": "vendor",
                "serialNumber": "A1",
                "connectionState": "OFFLINE",
            },
            received_at=2.0,
        )

        block = arbiter.snapshot()["blocks"]["TOP_1"]
        self.assertEqual(block["state"], BlockState.BLOCKED.value)
        self.assertEqual(block["fault_reason"], "connection.offline")

    def test_out_of_order_state_does_not_move_robot_backwards(self) -> None:
        arbiter, _ = make_components()
        tracker = self.operational_tracker(arbiter)
        recent = state(0.0, driving=False)
        recent.update({
            "headerId": 10,
            "timestamp": "2026-09-20T00:00:10Z",
            "manufacturer": "vendor",
            "serialNumber": "A1",
        })
        older = state(5.0)
        older.update({
            "headerId": 9,
            "timestamp": "2026-09-20T00:00:09Z",
            "manufacturer": "vendor",
            "serialNumber": "A1",
        })

        tracker.ingest_state("A1", recent, received_at=10.0)
        tracker.ingest_state("A1", older, received_at=11.0)

        snapshot = tracker.snapshot()["A1"]
        self.assertEqual(snapshot["last_node_id"], recent["lastNodeId"])
        self.assertEqual(snapshot["state_header_id"], 10)


if __name__ == "__main__":
    unittest.main()
