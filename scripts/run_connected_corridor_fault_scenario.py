#!/usr/bin/env python3
"""Poll and verify fail-closed outcomes for connected-corridor fault runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any, Callable
from urllib.request import ProxyHandler, build_opener


SCENARIOS = {
    "inside_estop",
    "inside_state_loss",
    "safe_stop_manual",
    "restart_inside",
}


class FaultScenarioVerifier:
    def __init__(self, scenario: str, robot_id: str) -> None:
        if scenario not in SCENARIOS:
            raise ValueError(f"unsupported fault scenario: {scenario}")
        self.scenario = scenario
        self.robot_id = robot_id
        self._violations: list[str] = []
        self._complete = False
        self._recovery_seen = False
        self._samples: list[dict[str, Any]] = []
        self._last_summary: tuple[Any, ...] | None = None
        self.evidence: dict[str, Any] = {
            "scenario": scenario,
            "robot_id": robot_id,
            "block_fault_seen": False,
            "robot_fault_seen": False,
            "opposite_admitted": False,
            "recovery_sticky": False,
            "samples": self._samples,
        }

    @property
    def complete(self) -> bool:
        return self._complete and not self._violations

    @property
    def violations(self) -> tuple[str, ...]:
        return tuple(self._violations)

    def _violate(self, reason: str) -> None:
        if reason not in self._violations:
            self._violations.append(reason)

    def observe(
        self,
        status: dict[str, Any],
        *,
        log_segment: str = "",
        at: float | None = None,
    ) -> None:
        robot = status.get("robots", {}).get(self.robot_id, {})
        arbiter = status.get("arbiter", {})
        blocks = arbiter.get("blocks", {})
        authorities = arbiter.get("authorities", {})
        pending = arbiter.get("pending_authorities", {})
        eligibility = robot.get("eligibility", {})
        eligible = bool(eligibility.get("eligible", False))
        reasons = tuple(str(item) for item in eligibility.get("reasons", []))
        current_block = robot.get("current_block")
        block_faults = {
            block_id: block.get("fault_reason")
            for block_id, block in blocks.items()
            if block.get("fault_reason")
        }
        target_authority = self.robot_id in authorities or self.robot_id in pending
        other_authorities = sorted(
            (set(authorities) | set(pending)) - {self.robot_id}
        )
        robot_faulted = bool(robot.get("faulted"))
        readiness = status.get("readiness", {})

        self.evidence["block_fault_seen"] |= bool(block_faults) or "BLOCK_FAULT" in log_segment
        self.evidence["robot_fault_seen"] |= robot_faulted or "ROBOT_FAULT" in log_segment
        if self.evidence["block_fault_seen"] and other_authorities:
            self.evidence["opposite_admitted"] = True
            self._violate("authority_admitted_after_block_fault")

        if not eligible and current_block is None and target_authority:
            self._violate("ineligible_robot_received_authority")

        sample = {
            "at": time.monotonic() if at is None else float(at),
            "eligible": eligible,
            "eligibility_reasons": list(reasons),
            "current_block": current_block,
            "robot_faulted": robot_faulted,
            "block_faults": block_faults,
            "target_authority": target_authority,
            "other_authorities": other_authorities,
            "readiness": readiness,
        }
        summary = (
            eligible,
            reasons,
            current_block,
            robot_faulted,
            tuple(sorted(block_faults.items())),
            target_authority,
            tuple(other_authorities),
            readiness.get("ready"),
            readiness.get("reason"),
        )
        if summary != self._last_summary:
            self._samples.append(sample)
            self._last_summary = summary

        if self.scenario in {"inside_estop", "inside_state_loss"}:
            expected_reason = (
                "safety.estop" if self.scenario == "inside_estop" else "state.stale"
            )
            self._complete = (
                not eligible
                and expected_reason in reasons
                and robot_faulted
                and bool(block_faults)
                and target_authority
                and not other_authorities
            )
        elif self.scenario == "safe_stop_manual":
            if current_block is None and "mode.not_automatic" in reasons:
                if block_faults or robot_faulted:
                    self._violate("safe_stop_faulted_corridor")
                self._complete = not target_authority and not block_faults and not robot_faulted
        else:
            reason = readiness.get("reason")
            if reason == "recovery.required":
                if self._recovery_seen and current_block is None:
                    self.evidence["recovery_sticky"] = True
                    self._complete = True
                self._recovery_seen = True
            elif self._recovery_seen and readiness.get("ready"):
                self._violate("recovery_cleared_without_operator_reset")

    def result(self) -> dict[str, Any]:
        result = dict(self.evidence)
        result["complete"] = self.complete
        result["violations"] = list(self._violations)
        return result


def poll_scenario(
    verifier: FaultScenarioVerifier,
    *,
    status_reader: Callable[[], dict[str, Any]],
    log_reader: Callable[[], str] = lambda: "",
    timeout: float = 120.0,
    poll_interval: float = 0.25,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    deadline = clock() + timeout
    while clock() < deadline:
        verifier.observe(status_reader(), log_segment=log_reader(), at=clock())
        if verifier.violations:
            raise RuntimeError(json.dumps(verifier.result(), ensure_ascii=False))
        if verifier.complete:
            return verifier.result()
        sleeper(poll_interval)
    raise TimeoutError(json.dumps(verifier.result(), ensure_ascii=False))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    parser.add_argument("--robot", required=True)
    parser.add_argument("--status-url", default="http://127.0.0.1:8200/traffic/status")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll-interval", type=float, default=0.25)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    opener = build_opener(ProxyHandler({}))
    log_offset = args.log.stat().st_size if args.log and args.log.exists() else 0

    def read_status() -> dict[str, Any]:
        with opener.open(args.status_url, timeout=5) as response:
            return json.load(response)

    def read_log() -> str:
        if args.log is None or not args.log.exists():
            return ""
        with args.log.open("r", encoding="utf-8", errors="replace") as stream:
            stream.seek(log_offset)
            return stream.read()

    try:
        result = poll_scenario(
            FaultScenarioVerifier(args.scenario, args.robot),
            status_reader=read_status,
            log_reader=read_log,
            timeout=args.timeout,
            poll_interval=args.poll_interval,
        )
    except (RuntimeError, TimeoutError) as error:
        print(str(error))
        return 1
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
