from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts/run_dynamic_corridor_scenario.py"
SCENARIO = ROOT / "config/dynamic_connected_corridor_chain_2v2.yaml"


def load_runner():
    spec = importlib.util.spec_from_file_location("dynamic_corridor_dispatch", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def snapshot(
    *,
    a1_hb: str | None = "CHAIN_LEFT_SLOT_1",
    b1_status: str | None = None,
    a2_status: str | None = None,
) -> dict:
    jobs = {}
    if b1_status:
        jobs["b1"] = {"robot_id": "AGV_B1", "status": b1_status}
    if a2_status:
        jobs["a2"] = {"robot_id": "AGV_A2", "status": a2_status}
    return {
        "jobs": jobs,
        "robots": {
            "AGV_A1": {"current_hb": a1_hb, "faulted": False},
            "AGV_A2": {"current_hb": "CHAIN_LEFT_SLOT_2", "faulted": False},
            "AGV_B1": {"current_hb": "CHAIN_RIGHT_SLOT_1", "faulted": False},
            "AGV_B2": {"current_hb": "CHAIN_RIGHT_SLOT_2", "faulted": False},
        },
        "arbiter": {
            "blocks": {"CHAIN_C1": {"fault_reason": None}},
        },
    }


class DynamicCorridorDispatchTests(unittest.TestCase):
    def test_dispatches_alternating_jobs_from_live_status_conditions(self) -> None:
        runner = load_runner()
        scenario = runner.load_scenario(SCENARIO)
        statuses = iter(
            [
                snapshot(),
                snapshot(a1_hb=None),
                snapshot(a1_hb=None, b1_status="ACTIVE"),
                snapshot(a1_hb=None, b1_status="ACTIVE", a2_status="ACTIVE"),
            ]
        )
        dispatched: list[tuple[str, str]] = []
        clock_value = [0.0]

        def clock() -> float:
            clock_value[0] += 0.1
            return clock_value[0]

        runner.run_scenario(
            scenario,
            status_reader=lambda: next(statuses),
            submitter=lambda robot, goal: dispatched.append((robot, goal)) or {
                "decision": "WAIT"
            },
            timeout=10.0,
            poll_interval=0.0,
            clock=clock,
            sleeper=lambda _: None,
        )

        self.assertEqual(
            dispatched,
            [
                ("AGV_A1", "CHAIN_R4"),
                ("AGV_B1", "CHAIN_L4"),
                ("AGV_A2", "CHAIN_R3"),
                ("AGV_B2", "CHAIN_L3"),
            ],
        )

    def test_active_job_is_not_treated_as_safely_replaceable(self) -> None:
        runner = load_runner()
        active = snapshot(b1_status="ACTIVE")
        waiting = snapshot(b1_status="WAITING")
        condition = {
            "type": "job_replaceable",
            "robot": "AGV_B1",
        }
        self.assertFalse(runner.condition_met(condition, active))
        self.assertTrue(runner.condition_met(condition, waiting))

    def test_each_condition_gets_a_full_timeout_budget(self) -> None:
        runner = load_runner()
        scenario = runner.load_scenario(SCENARIO)
        statuses = iter(
            [
                snapshot(a1_hb=None),
                snapshot(a1_hb=None, b1_status="ACTIVE"),
                snapshot(a1_hb=None, b1_status="ACTIVE", a2_status="ACTIVE"),
            ]
        )
        clock_value = [0.0]

        def slow_clock() -> float:
            clock_value[0] += 3.0
            return clock_value[0]

        runner.run_scenario(
            scenario,
            status_reader=lambda: next(statuses),
            submitter=lambda _robot, _goal: {"decision": "WAIT"},
            timeout=5.0,
            poll_interval=0.0,
            clock=slow_clock,
            sleeper=lambda _: None,
        )


if __name__ == "__main__":
    unittest.main()
