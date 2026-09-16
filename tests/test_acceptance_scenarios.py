from __future__ import annotations

import unittest

from tests.test_dynamic_insert import make_gate, payload, telemetry


def move_to(gate, tracker, robot: str, x: float, node: str) -> None:
    start = tracker.current_safe_node(robot)
    middle = {
        ("N0", "N1"): 5.0,
        ("N1", "N0"): 5.0,
        ("N1", "N2"): 15.0,
        ("N2", "N1"): 15.0,
    }[(start, node)]
    tracker.ingest_state(robot, telemetry(middle, "", driving=True))
    tracker.ingest_state(robot, telemetry(x, node, driving=False))
    gate.tick()


class AcceptanceScenarioTests(unittest.TestCase):
    def test_same_direction_pipeline_releases_next_robot(self) -> None:
        gate, tracker, forwarder = make_gate()
        tracker.ingest_state("A1", telemetry(0.0, "N0"))
        tracker.ingest_state("A2", telemetry(0.0, "N0"))

        self.assertEqual(gate.submit(payload("A1", "N2"))["decision"], "ADMIT")
        self.assertEqual(gate.submit(payload("A2", "N2"))["decision"], "WAIT")

        move_to(gate, tracker, "A1", 10.0, "N1")
        tracker.ingest_state("A1", telemetry(15.0, "", driving=True))
        gate.tick()

        active_robots = {
            job["robot_id"]
            for job in gate.status()["jobs"].values()
            if job["status"] == "ACTIVE"
        }
        self.assertEqual(active_robots, {"A1", "A2"})
        self.assertEqual(
            [item["request"]["description"]["places"][0] for item in forwarder.payloads],
            ["N1", "N2", "N1"],
        )

    def test_2v2_finishes_without_opposite_same_block_occupancy(self) -> None:
        gate, tracker, _ = make_gate()
        for robot in ("A1", "A2"):
            tracker.ingest_state(robot, telemetry(0.0, "N0"))
        for robot in ("B1", "B2"):
            tracker.ingest_state(robot, telemetry(20.0, "N2"))

        for robot, goal in (("A1", "N2"), ("A2", "N2"), ("B1", "N0"), ("B2", "N0")):
            gate.submit(payload(robot, goal))

        node_x = {"N0": 0.0, "N1": 10.0, "N2": 20.0}
        for _ in range(20):
            jobs = gate.status()["jobs"]
            if all(job["status"] == "COMPLETE" for job in jobs.values()):
                break
            active = [job for job in jobs.values() if job["status"] == "ACTIVE"]
            self.assertTrue(active, gate.status())
            job = active[0]
            route = next(
                route
                for route in gate.arbiter.registry.routes
                if route.route_id == job["route_id"]
            )
            goal = route.steps[job["step_index"]].goal_node
            move_to(gate, tracker, job["robot_id"], node_x[goal], goal)
            for block in gate.arbiter.registry.blocks.values():
                self.assertLessEqual(len(set(block.occupants.values())), 1)

        statuses = {job["status"] for job in gate.status()["jobs"].values()}
        self.assertEqual(statuses, {"COMPLETE"})

    def test_3v3_finishes_without_deadlock(self) -> None:
        gate, tracker, _ = make_gate(endpoint_capacity=8)
        for robot in ("A1", "A2", "A3"):
            tracker.ingest_state(robot, telemetry(0.0, "N0"))
            gate.submit(payload(robot, "N2"))
        for robot in ("B1", "B2", "B3"):
            tracker.ingest_state(robot, telemetry(20.0, "N2"))
            gate.submit(payload(robot, "N0"))

        node_x = {"N0": 0.0, "N1": 10.0, "N2": 20.0}
        for _ in range(40):
            jobs = gate.status()["jobs"]
            if all(job["status"] == "COMPLETE" for job in jobs.values()):
                break
            active = [job for job in jobs.values() if job["status"] == "ACTIVE"]
            self.assertTrue(active, gate.status())
            job = active[0]
            route = next(
                route
                for route in gate.arbiter.registry.routes
                if route.route_id == job["route_id"]
            )
            goal = route.steps[job["step_index"]].goal_node
            move_to(gate, tracker, job["robot_id"], node_x[goal], goal)

        jobs = gate.status()["jobs"]
        self.assertEqual(len(jobs), 6)
        self.assertEqual({job["status"] for job in jobs.values()}, {"COMPLETE"})


if __name__ == "__main__":
    unittest.main()
