from __future__ import annotations

import unittest

from traffic_control.models import BlockState, Decision, Direction

from tests.test_block_occupancy import make_components, state


class FaultSafetyTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
