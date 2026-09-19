from __future__ import annotations

from urllib.error import HTTPError
import unittest

from traffic_control.task_gate import TaskGate
from traffic_control.models import Direction

from tests.test_movement_authority import state, tracking_components


def payload(robot: str, goal: str) -> dict:
    return {
        "type": "robot_task_request",
        "robot": robot,
        "fleet": "TOOL",
        "request": {
            "category": "patrol",
            "description": {"places": [goal], "rounds": 1},
        },
    }


class CorridorChainTaskGateTests(unittest.TestCase):
    def make_gate(self, forwarder):
        registry, arbiter, planner, tracker = tracking_components()
        tracker.ingest_state(
            "A1",
            state("L1", -1.0, 1.0, driving=False),
            received_at=1.0,
        )
        return registry, arbiter, tracker, TaskGate(
            registry,
            arbiter,
            tracker,
            forwarder,
        )

    def test_clear_chain_forwards_one_task_to_the_final_slot(self) -> None:
        forwarded: list[dict] = []
        registry, arbiter, tracker, gate = self.make_gate(
            lambda request: forwarded.append(request) or {"success": True}
        )

        result = gate.submit(payload("A1", "R1"))

        self.assertEqual(result["decision"], "ADMIT")
        self.assertEqual(
            forwarded[0]["request"]["description"]["places"],
            ["R1"],
        )
        authority = arbiter.authority_for_robot("A1")
        self.assertIsNotNone(authority)
        assert authority is not None
        self.assertEqual(authority.block_ids, ("C1", "C2", "C3"))
        job = gate.status()["jobs"][result["job_id"]]
        self.assertEqual(job["chain_id"], "MAIN")
        self.assertEqual(job["final_goal_node"], "R1")
        self.assertEqual(job["destination_slot"], "RIGHT_1")

    def test_conflict_sends_first_leg_to_side_bay_then_continues(self) -> None:
        forwarded: list[dict] = []
        registry, arbiter, tracker, gate = self.make_gate(
            lambda request: forwarded.append(request) or {"success": True}
        )
        registry.blocks["C3"].occupants["opposite"] = Direction.B_TO_A

        result = gate.submit(payload("A1", "R1"))

        self.assertEqual(result["decision"], "ADMIT")
        self.assertEqual(
            forwarded[0]["request"]["description"]["places"],
            ["S2"],
        )
        authority = arbiter.authority_for_robot("A1")
        assert authority is not None
        self.assertEqual(authority.block_ids, ("C1", "C2"))

        arbiter.mark_authority_arrived("A1")
        tracker.ingest_state(
            "A1",
            state("S2", 20.0, 2.0, driving=False),
            received_at=2.0,
        )
        registry.blocks["C3"].occupants.pop("opposite")
        gate.tick(now=2.0)

        self.assertEqual(
            [item["request"]["description"]["places"] for item in forwarded],
            [["S2"], ["R1"]],
        )
        next_authority = arbiter.authority_for_robot("A1")
        assert next_authority is not None
        self.assertEqual(next_authority.block_ids, ("C3",))

    def test_explicit_http_rejection_releases_the_whole_authority(self) -> None:
        def reject(request):
            raise HTTPError("http://rmf", 401, "Unauthorized", {}, None)

        registry, arbiter, tracker, gate = self.make_gate(reject)

        result = gate.submit(payload("A1", "R1"))

        self.assertEqual(result["decision"], "BLOCKED")
        self.assertIsNone(arbiter.authority_for_robot("A1"))
        self.assertFalse(registry.holding_bays["RIGHT_1"].reservations)
        self.assertTrue(all(not block.reservations for block in registry.blocks.values()))

    def test_uncertain_forwarding_response_retains_the_authority(self) -> None:
        def uncertain(request):
            raise TimeoutError("response lost")

        registry, arbiter, tracker, gate = self.make_gate(uncertain)

        result = gate.submit(payload("A1", "R1"))

        self.assertEqual(result["decision"], "FORWARD_UNKNOWN")
        self.assertIsNotNone(arbiter.authority_for_robot("A1"))
        self.assertEqual(registry.holding_bays["RIGHT_1"].reservations, {"A1"})


if __name__ == "__main__":
    unittest.main()
