"""Phase-A RMF task admission gate for managed directional corridors.

The gate deliberately sits in front of the RMF REST API.  A managed task is
split into configured safe corridor steps and only the currently admitted
step is forwarded.  The state machine and HTTP/MQTT adapters remain separate
so the admission logic can later move into the Fleet Adapter.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import json
import logging
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable
from urllib import error as urllib_error
from urllib import request as urllib_request
from uuid import uuid4

from .corridor_registry import CorridorRegistry
from .direction_arbiter import DirectionArbiter
from .models import Decision, RouteIntent
from .robot_tracker import MqttStateMonitor, RobotTracker

logger = logging.getLogger(__name__)

Forwarder = Callable[[dict[str, Any]], dict[str, Any]]


class JobStatus(str, Enum):
    WAITING = "WAITING"
    ACTIVE = "ACTIVE"
    RETRY = "RETRY"
    BLOCKED = "BLOCKED"
    COMPLETE = "COMPLETE"
    CANCELLED = "CANCELLED"


@dataclass
class GateJob:
    job_id: str
    robot_id: str
    original_payload: dict[str, Any]
    route: RouteIntent
    step_index: int = 0
    status: JobStatus = JobStatus.WAITING
    upstream_result: dict[str, Any] | None = None
    last_error: str | None = None

    @property
    def current_step(self):
        if self.step_index >= len(self.route.steps):
            return None
        return self.route.steps[self.step_index]


class TaskGate:
    """Configuration-driven admission and safe-segment task staging."""

    def __init__(
        self,
        registry: CorridorRegistry,
        arbiter: DirectionArbiter,
        tracker: RobotTracker,
        forwarder: Forwarder,
    ) -> None:
        self.registry = registry
        self.arbiter = arbiter
        self.tracker = tracker
        self.forwarder = forwarder
        self._jobs: dict[str, GateJob] = {}
        self._lock = threading.RLock()

    def submit(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Submit, queue, or bypass one RMF robot task request."""
        if not isinstance(payload, dict):
            raise TypeError("task payload must be a mapping")

        if not self.registry.enabled:
            upstream = self.forwarder(deepcopy(payload))
            return {"decision": "BYPASS", "upstream": upstream}

        robot_id = _robot_id(payload)
        goal_node = _goal_node(payload)
        start_node = self.tracker.current_safe_node(robot_id)
        if start_node is None:
            return {
                "decision": Decision.BLOCKED.value,
                "reason": "robot_not_at_known_safe_point",
            }

        route = self.registry.resolve_route(start_node, goal_node)
        if route is None:
            policy = str(
                self.registry.settings.get("unmatched_route_policy", "BLOCKED")
            ).upper()
            if policy == "BYPASS":
                upstream = self.forwarder(deepcopy(payload))
                return {"decision": "BYPASS", "upstream": upstream}
            return {
                "decision": Decision.BLOCKED.value,
                "reason": "unmatched_managed_route",
                "start_node": start_node,
                "goal_node": goal_node,
            }

        with self._lock:
            if self._active_job_for_robot(robot_id) is not None:
                return {
                    "decision": Decision.BLOCKED.value,
                    "reason": "robot_already_has_gate_job",
                }
            job = GateJob(
                job_id=uuid4().hex,
                robot_id=robot_id,
                original_payload=deepcopy(payload),
                route=route,
            )
            self._jobs[job.job_id] = job
            logger.info(
                "[TRAFFIC] ARBITER_REQUEST robot=%s route=%s",
                robot_id,
                route.route_id,
            )
            result = self._attempt_current_step(job)
            result["job_id"] = job.job_id
            return result

    def tick(self, *, now: float | None = None) -> None:
        """Advance completed segments, retries, waiting jobs, and timeouts."""
        self.tracker.expire_stale(now=now)
        with self._lock:
            for job in list(self._jobs.values()):
                if job.status in {
                    JobStatus.BLOCKED,
                    JobStatus.COMPLETE,
                    JobStatus.CANCELLED,
                }:
                    continue

                step = job.current_step
                if step is None:
                    job.status = JobStatus.COMPLETE
                    continue

                if job.status is JobStatus.ACTIVE:
                    if not self.tracker.has_arrived(job.robot_id, step.goal_node):
                        continue
                    self.arbiter.mark_arrived(job.robot_id, step.block_id)
                    job.step_index += 1
                    job.upstream_result = None
                    job.last_error = None
                    if job.current_step is None:
                        job.status = JobStatus.COMPLETE
                        logger.info(
                            "[TRAFFIC] TASK_COMPLETED robot=%s job=%s",
                            job.robot_id,
                            job.job_id,
                        )
                        continue
                    job.status = JobStatus.WAITING

                self._attempt_current_step(job)

    def cancel(self, job_id: str) -> bool:
        """Cancel a task that has not yet been submitted upstream."""
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None or job.status in {
                JobStatus.ACTIVE,
                JobStatus.COMPLETE,
                JobStatus.CANCELLED,
            }:
                return False
            if not self.arbiter.cancel(job.robot_id, job.current_step.block_id):
                if job.status is JobStatus.RETRY:
                    return False
            job.status = JobStatus.CANCELLED
            logger.info(
                "[TRAFFIC] TASK_CANCELLED robot=%s job=%s",
                job.robot_id,
                job.job_id,
            )
            return True

    def reset(self, *, force: bool = False) -> bool:
        with self._lock:
            if not self.arbiter.reset(force=force):
                return False
            for job in self._jobs.values():
                if job.status not in {JobStatus.COMPLETE, JobStatus.CANCELLED}:
                    job.status = JobStatus.CANCELLED
            return True

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "enabled": self.registry.enabled,
                "jobs": {
                    job_id: {
                        "robot_id": job.robot_id,
                        "route_id": job.route.route_id,
                        "step_index": job.step_index,
                        "step_count": len(job.route.steps),
                        "status": job.status.value,
                        "last_error": job.last_error,
                    }
                    for job_id, job in sorted(self._jobs.items())
                },
                "arbiter": self.arbiter.snapshot(),
                "robots": self.tracker.snapshot(),
            }

    def _attempt_current_step(self, job: GateJob) -> dict[str, Any]:
        step = job.current_step
        if step is None:
            job.status = JobStatus.COMPLETE
            return {"decision": "COMPLETE"}

        decision = self.arbiter.decision_for(job.robot_id, step.block_id)
        if decision is not Decision.ADMIT:
            decision = self.arbiter.request(
                job.robot_id,
                step.block_id,
                step.direction,
                step.destination_hb,
                source_hb=step.source_hb,
                release_node=step.release_node,
            )
        if decision is Decision.WAIT:
            job.status = JobStatus.WAITING
            logger.info(
                "[TRAFFIC] TASK_HELD robot=%s job=%s block=%s",
                job.robot_id,
                job.job_id,
                step.block_id,
            )
            return {"decision": Decision.WAIT.value}
        if decision is Decision.BLOCKED:
            job.status = JobStatus.BLOCKED
            return {"decision": Decision.BLOCKED.value}
        return self._forward_current_step(job)

    def _forward_current_step(self, job: GateJob) -> dict[str, Any]:
        step = job.current_step
        staged = deepcopy(job.original_payload)
        description = staged.setdefault("request", {}).setdefault("description", {})
        description["places"] = [step.goal_node]
        description["rounds"] = 1
        try:
            result = self.forwarder(staged)
        except urllib_error.HTTPError as error:
            if 400 <= error.code < 500:
                self.arbiter.cancel(job.robot_id, step.block_id)
                job.status = JobStatus.BLOCKED
                job.last_error = str(error)
                logger.error(
                    "[TRAFFIC] TASK_FORWARD_REJECTED robot=%s job=%s status=%s error=%s",
                    job.robot_id,
                    job.job_id,
                    error.code,
                    error,
                )
                return {
                    "decision": Decision.BLOCKED.value,
                    "reason": "upstream_http_rejection",
                    "status_code": error.code,
                    "error": str(error),
                }
            job.status = JobStatus.ACTIVE
            job.last_error = str(error)
            logger.error(
                "[TRAFFIC] TASK_FORWARD_UNKNOWN robot=%s job=%s error=%s",
                job.robot_id,
                job.job_id,
                error,
            )
            return {"decision": "FORWARD_UNKNOWN", "error": str(error)}
        except Exception as error:  # delivery may have succeeded before the response failed
            job.status = JobStatus.ACTIVE
            job.last_error = str(error)
            logger.error(
                "[TRAFFIC] TASK_FORWARD_UNKNOWN robot=%s job=%s error=%s",
                job.robot_id,
                job.job_id,
                error,
            )
            return {"decision": "FORWARD_UNKNOWN", "error": str(error)}

        if isinstance(result, dict) and result.get("success") is False:
            error = str(
                result.get("message") or result.get("error") or "upstream rejected task"
            )
            job.status = JobStatus.RETRY
            job.last_error = error
            logger.error(
                "[TRAFFIC] TASK_FORWARD_RETRY robot=%s job=%s error=%s",
                job.robot_id,
                job.job_id,
                error,
            )
            return {"decision": "RETRY", "error": error}

        job.status = JobStatus.ACTIVE
        job.upstream_result = result
        job.last_error = None
        logger.info(
            "[TRAFFIC] TASK_RELEASED robot=%s job=%s block=%s goal=%s",
            job.robot_id,
            job.job_id,
            step.block_id,
            step.goal_node,
        )
        return {"decision": Decision.ADMIT.value, "upstream": result}

    def _active_job_for_robot(self, robot_id: str) -> GateJob | None:
        terminal = {JobStatus.COMPLETE, JobStatus.CANCELLED, JobStatus.BLOCKED}
        return next(
            (
                job
                for job in self._jobs.values()
                if job.robot_id == robot_id and job.status not in terminal
            ),
            None,
        )


class HttpTaskForwarder:
    """Small no-proxy JSON client for the existing RMF API server."""

    def __init__(self, url: str, *, timeout: float = 10.0) -> None:
        self.url = url
        self.timeout = float(timeout)
        self.bearer_token = os.environ.get("RMF_API_BEARER_TOKEN", "").strip()
        self._opener = urllib_request.build_opener(urllib_request.ProxyHandler({}))

    def __call__(self, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.bearer_token:
            headers["Authorization"] = f"Bearer {self.bearer_token}"
        req = urllib_request.Request(
            self.url,
            data=body,
            headers=headers,
            method="POST",
        )
        with self._opener.open(req, timeout=self.timeout) as response:
            raw = response.read()
        return json.loads(raw.decode("utf-8")) if raw else {"success": True}


class PeriodicGateRunner:
    def __init__(self, gate: TaskGate, *, interval: float = 0.25) -> None:
        self.gate = gate
        self.interval = float(interval)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.interval * 4))

    def _run(self) -> None:
        while not self._stop.wait(self.interval):
            try:
                self.gate.tick()
            except Exception:
                logger.exception("[TRAFFIC] gate tick failed")


def create_app(gate: TaskGate):
    from fastapi import FastAPI, HTTPException, Query

    app = FastAPI(title="RMF Direction Arbiter Task Gate")

    @app.post("/tasks/robot_task")
    def submit_task(payload: dict[str, Any]):
        try:
            return gate.submit(payload)
        except (KeyError, TypeError, ValueError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.get("/traffic/status")
    def traffic_status():
        return gate.status()

    @app.post("/traffic/tick")
    def traffic_tick():
        gate.tick()
        return gate.status()

    @app.post("/traffic/jobs/{job_id}/cancel")
    def cancel_job(job_id: str):
        if not gate.cancel(job_id):
            raise HTTPException(status_code=409, detail="job cannot be cancelled")
        return gate.status()["jobs"][job_id]

    @app.post("/traffic/reset")
    def reset_traffic(force: bool = Query(default=False)):
        if not gate.reset(force=force):
            raise HTTPException(status_code=409, detail="robot is still inside a block")
        return {"reset": True, "force": force}

    return app


def _robot_id(payload: dict[str, Any]) -> str:
    robot_id = payload.get("robot") or payload.get("robot_id")
    if not robot_id:
        raise ValueError("task payload is missing robot")
    return str(robot_id)


def _goal_node(payload: dict[str, Any]) -> str:
    try:
        places = payload["request"]["description"]["places"]
        goal = places[-1]
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError("task payload is missing request.description.places") from error
    if isinstance(goal, dict):
        goal = goal.get("place") or goal.get("waypoint")
    if goal is None:
        raise ValueError("task goal is empty")
    return str(goal)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/corridor_blocks.yaml")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8200)
    parser.add_argument(
        "--upstream", default="http://127.0.0.1:8100/tasks/robot_task"
    )
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument(
        "--mqtt-topic", default="uagv/v2.0.0/inatech/+/state"
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    registry = CorridorRegistry.from_yaml(Path(args.config))
    arbiter = DirectionArbiter(registry)
    tracker = RobotTracker(
        registry,
        arbiter,
        telemetry_timeout=float(registry.settings.get("telemetry_timeout", 5.0)),
    )
    gate = TaskGate(registry, arbiter, tracker, HttpTaskForwarder(args.upstream))
    monitor = (
        MqttStateMonitor(
            tracker,
            host=args.mqtt_host,
            port=args.mqtt_port,
            topic=args.mqtt_topic,
        )
        if registry.enabled
        else None
    )
    runner = PeriodicGateRunner(
        gate,
        interval=float(registry.settings.get("tick_interval", 0.25)),
    )

    import uvicorn

    if monitor is not None:
        monitor.start()
    runner.start()
    try:
        uvicorn.run(create_app(gate), host=args.listen_host, port=args.port)
    finally:
        runner.stop()
        if monitor is not None:
            monitor.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
