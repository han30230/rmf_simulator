from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/t4_dispatch_passing_bay_staging_dynamic_1v3.py"
WRAPPER_PATH = ROOT / "scripts/t4_dispatch_passing_bay_staging_dynamic_1v3.sh"


def load_runner():
    spec = importlib.util.spec_from_file_location("dynamic_staging_dispatch", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def status(*, a1_node: str, b1_hb: str | None) -> dict:
    return {
        "jobs": {},
        "robots": {
            "AGV_A1": {
                "last_node_id": a1_node,
                "current_hb": None,
                "faulted": False,
            },
            "AGV_B1": {
                "last_node_id": "6137" if b1_hb else "2106",
                "current_hb": b1_hb,
                "faulted": False,
            },
        },
        "arbiter": {
            "blocks": {
                "PASSING": {"fault_reason": None},
            },
        },
    }


class DynamicStagingDispatchTests(unittest.TestCase):
    def test_injects_trailing_jobs_from_live_status_conditions(self) -> None:
        runner = load_runner()
        statuses = iter(
            [
                status(a1_node="2104", b1_hb=None),
                status(a1_node="2104", b1_hb="HB_MIDDLE_SIDE"),
                status(a1_node="2104", b1_hb="HB_MIDDLE_SIDE"),
                status(a1_node="2105", b1_hb="HB_MIDDLE_SIDE"),
            ]
        )
        dispatched: list[tuple[str, str]] = []
        clock_value = [0.0]

        def clock() -> float:
            clock_value[0] += 0.1
            return clock_value[0]

        runner.run_dynamic_dispatch(
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
                ("AGV_A1", "P4_RS1"),
                ("AGV_B1", "P4_LS1"),
                ("AGV_B2", "P4_LS2"),
                ("AGV_B3", "P4_LS3"),
            ],
        )

    def test_dynamic_wrapper_is_repository_relative(self) -> None:
        content = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("BASH_SOURCE", content)
        self.assertIn(".venv/bin/python", content)
        self.assertIn(RUNNER_PATH.name, content)
        self.assertNotIn("/home/han30230", content)


if __name__ == "__main__":
    unittest.main()
