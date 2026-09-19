#!/usr/bin/env python3
"""Run a YAML-defined sequence of traffic jobs from live status conditions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any, Callable
from urllib.request import ProxyHandler, Request, build_opener

import yaml


StatusReader = Callable[[], dict[str, Any]]
Submitter = Callable[[str, str], dict[str, Any]]


def load_scenario(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not isinstance(raw.get("steps"), list):
        raise ValueError("dynamic scenario must contain a steps list")
    for step in raw["steps"]:
        if not isinstance(step, dict) or not step.get("robot") or not step.get("goal"):
            raise ValueError("each dynamic step requires robot and goal")
    return raw


def _job_status(current: dict[str, Any], robot_id: str) -> str | None:
    matches = [
        str(job.get("status"))
        for job in current.get("jobs", {}).values()
        if job.get("robot_id") == robot_id
    ]
    return matches[-1] if matches else None


def condition_met(condition: dict[str, Any] | None, current: dict[str, Any]) -> bool:
    if not condition:
        return True
    condition_type = condition.get("type")
    robot_id = str(condition.get("robot", ""))
    if condition_type == "robot_left_holding_bay":
        robot = current.get("robots", {}).get(robot_id)
        return robot is not None and robot.get("current_hb") != condition.get(
            "holding_bay"
        )
    if condition_type == "job_status":
        return _job_status(current, robot_id) == str(condition.get("status"))
    if condition_type == "job_replaceable":
        return _job_status(current, robot_id) in {"WAITING", "RETRY"}
    raise ValueError(f"unsupported dynamic condition: {condition_type}")


def _assert_safe(current: dict[str, Any]) -> None:
    block_faults = [
        block_id
        for block_id, block in current.get("arbiter", {}).get("blocks", {}).items()
        if block.get("fault_reason") is not None
    ]
    robot_faults = [
        robot_id
        for robot_id, robot in current.get("robots", {}).items()
        if robot.get("faulted")
    ]
    blocked_jobs = [
        job.get("robot_id")
        for job in current.get("jobs", {}).values()
        if job.get("status") == "BLOCKED"
    ]
    if block_faults or robot_faults or blocked_jobs:
        raise RuntimeError(
            "unsafe runtime state: "
            f"block_faults={block_faults} robot_faults={robot_faults} "
            f"blocked_jobs={blocked_jobs}"
        )


def _wait_for_condition(
    condition: dict[str, Any],
    *,
    status_reader: StatusReader,
    deadline: float,
    poll_interval: float,
    clock: Callable[[], float],
    sleeper: Callable[[float], None],
) -> None:
    while clock() < deadline:
        current = status_reader()
        _assert_safe(current)
        if condition_met(condition, current):
            return
        sleeper(poll_interval)
    raise TimeoutError(f"timeout waiting for condition {condition}")


def run_scenario(
    scenario: dict[str, Any],
    *,
    status_reader: StatusReader,
    submitter: Submitter,
    timeout: float = 360.0,
    poll_interval: float = 0.25,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> None:
    for step in scenario["steps"]:
        condition = step.get("when")
        if condition:
            _wait_for_condition(
                condition,
                status_reader=status_reader,
                deadline=clock() + timeout,
                poll_interval=poll_interval,
                clock=clock,
                sleeper=sleeper,
            )
        result = submitter(str(step["robot"]), str(step["goal"]))
        print(
            f"[DYNAMIC] robot={step['robot']} goal={step['goal']} "
            f"decision={result.get('decision')}",
            flush=True,
        )
        if result.get("decision") == "BLOCKED":
            raise RuntimeError(f"dynamic dispatch blocked: {step['robot']}: {result}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--gate", default="http://127.0.0.1:8200")
    parser.add_argument("--timeout", type=float, default=360.0)
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

    run_scenario(
        load_scenario(args.scenario),
        status_reader=read_status,
        submitter=submit_task,
        timeout=args.timeout,
        poll_interval=args.poll_interval,
    )


if __name__ == "__main__":
    main()
