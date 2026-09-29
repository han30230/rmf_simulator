#!/usr/bin/env python3
"""Inject the staging 1v3 workload from observed traffic conditions."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Callable
from urllib.request import ProxyHandler, Request, build_opener


StatusReader = Callable[[], dict[str, Any]]
Submitter = Callable[[str, str], dict[str, Any]]


def _assert_safe(current: dict[str, Any]) -> None:
    faults = [
        block_id
        for block_id, block in current["arbiter"]["blocks"].items()
        if block["fault_reason"] is not None
    ]
    robot_faults = [
        robot_id
        for robot_id, robot in current["robots"].items()
        if robot["faulted"]
    ]
    blocked_jobs = [
        job["robot_id"]
        for job in current["jobs"].values()
        if job["status"] == "BLOCKED"
    ]
    if faults or robot_faults or blocked_jobs:
        raise RuntimeError(
            "unsafe runtime state: "
            f"block_faults={faults} robot_faults={robot_faults} "
            f"blocked_jobs={blocked_jobs}"
        )


def _wait_for(
    description: str,
    predicate: Callable[[dict[str, Any]], bool],
    *,
    status_reader: StatusReader,
    deadline: float,
    poll_interval: float,
    clock: Callable[[], float],
    sleeper: Callable[[float], None],
) -> dict[str, Any]:
    while clock() < deadline:
        current = status_reader()
        _assert_safe(current)
        if predicate(current):
            print(f"[DYNAMIC] condition={description}", flush=True)
            return current
        sleeper(poll_interval)
    raise TimeoutError(f"timeout waiting for {description}")


def run_dynamic_dispatch(
    *,
    status_reader: StatusReader,
    submitter: Submitter,
    timeout: float = 180.0,
    poll_interval: float = 0.25,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> None:
    deadline = clock() + timeout

    def submit(robot_id: str, goal_node: str) -> None:
        result = submitter(robot_id, goal_node)
        print(
            f"[DYNAMIC] submitted robot={robot_id} goal={goal_node} "
            f"decision={result.get('decision')}",
            flush=True,
        )
        if result.get("decision") == "BLOCKED":
            raise RuntimeError(f"dynamic dispatch blocked: {robot_id}: {result}")

    submit("AGV_A1", "P4_RS1")
    submit("AGV_B1", "P4_LS1")

    _wait_for(
        "B1_at_middle_side",
        lambda current: current["robots"]["AGV_B1"]["current_hb"]
        == "HB_MIDDLE_SIDE",
        status_reader=status_reader,
        deadline=deadline,
        poll_interval=poll_interval,
        clock=clock,
        sleeper=sleeper,
    )
    submit("AGV_B2", "P4_LS2")

    _wait_for(
        "A1_inside_passing_corridor",
        lambda current: current["robots"]["AGV_A1"]["last_node_id"] == "2105",
        status_reader=status_reader,
        deadline=deadline,
        poll_interval=poll_interval,
        clock=clock,
        sleeper=sleeper,
    )
    submit("AGV_B3", "P4_LS3")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate", default="http://127.0.0.1:8200")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--poll-interval", type=float, default=0.25)
    args = parser.parse_args()

    opener = build_opener(ProxyHandler({}))
    base_url = args.gate.rstrip("/")

    def read_status() -> dict[str, Any]:
        with opener.open(f"{base_url}/traffic/status", timeout=5) as response:
            return json.load(response)

    def submit_task(robot_id: str, goal_node: str) -> dict[str, Any]:
        payload = {
            "type": "robot_task_request",
            "robot": robot_id,
            "fleet": "TOOL",
            "request": {
                "unix_millis_earliest_start_time": 0,
                "category": "patrol",
                "priority": {"type": "default", "value": 0},
                "description": {"places": [goal_node], "rounds": 1},
            },
        }
        request = Request(
            f"{base_url}/tasks/robot_task",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with opener.open(request, timeout=10) as response:
            return json.load(response)

    run_dynamic_dispatch(
        status_reader=read_status,
        submitter=submit_task,
        timeout=args.timeout,
        poll_interval=args.poll_interval,
    )


if __name__ == "__main__":
    main()
