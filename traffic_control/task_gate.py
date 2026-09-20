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
from typing import TYPE_CHECKING, Any, Callable
from urllib import error as urllib_error
from urllib import parse as urllib_parse
from urllib import request as urllib_request
from uuid import uuid4

from .corridor_chain import CorridorChainPlanner, PlannedAuthority
from .corridor_registry import CorridorRegistry
from .direction_arbiter import DirectionArbiter
from .models import Decision, RouteIntent, RouteStep
from .robot_tracker import MqttStateMonitor, RobotTracker

if TYPE_CHECKING:
    from .readiness import RuntimeReadiness

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
    wait_for_guarded_route: bool = False

    @property
    def current_step(self):
        if self.step_index >= len(self.route.steps):
            return None
        return self.route.steps[self.step_index]


@dataclass
class ChainGateJob:
    job_id: str
    robot_id: str
    original_payload: dict[str, Any]
    chain_id: str
    final_goal_node: str
    request_time: float = 0.0
    status: JobStatus = JobStatus.WAITING
    active_plan: PlannedAuthority | None = None
    active_authority_id: str | None = None
    leg_count: int = 0
    upstream_result: dict[str, Any] | None = None
    last_error: str | None = None


class TaskGate:
    """Configuration-driven admission and safe-segment task staging."""

    def __init__(
        self,
        registry: CorridorRegistry,
        arbiter: DirectionArbiter,
        tracker: RobotTracker,
        forwarder: Forwarder,
        readiness: "RuntimeReadiness | None" = None,
    ) -> None:
        self.registry = registry
        self.arbiter = arbiter
        self.tracker = tracker
        self.forwarder = forwarder
        self.readiness = readiness
        self._jobs: dict[str, GateJob] = {}
        self._chain_jobs: dict[str, ChainGateJob] = {}
        self._chain_planner = CorridorChainPlanner(registry)
        self._lock = threading.RLock()

    def submit(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Submit, queue, or bypass one RMF robot task request."""
        if not isinstance(payload, dict):
            raise TypeError("task payload must be a mapping")

        if not self.registry.enabled:
            upstream = self.forwarder(deepcopy(payload))
            return {"decision": "BYPASS", "upstream": upstream}

        runtime_rejection = self._runtime_rejection()
        if runtime_rejection is not None:
            return runtime_rejection

        robot_id = _robot_id(payload)
        eligibility = self.tracker.eligibility(robot_id)
        if not eligibility.eligible:
            return {
                "decision": Decision.BLOCKED.value,
                "reason": "robot_not_eligible",
                "eligibility_reasons": list(eligibility.reasons),
            }
        goal_node = _goal_node(payload)
        start_node = self.tracker.current_safe_node(robot_id)
        if start_node is None:
            return {
                "decision": Decision.BLOCKED.value,
                "reason": "robot_not_at_known_safe_point",
            }

        routes = self.registry.resolve_routes(start_node, goal_node)
        route = self._select_route(routes)
        if route is None:
            chain_path = self.registry.resolve_chain_path(start_node, goal_node)
            if chain_path is not None:
                with self._lock:
                    if self._active_job_for_robot(robot_id) is not None:
                        return {
                            "decision": Decision.BLOCKED.value,
                            "reason": "robot_already_has_gate_job",
                        }
                    job = ChainGateJob(
                        job_id=uuid4().hex,
                        robot_id=robot_id,
                        original_payload=deepcopy(payload),
                        chain_id=chain_path.chain_id,
                        final_goal_node=goal_node,
                        request_time=time.monotonic(),
                    )
                    self._chain_jobs[job.job_id] = job
                    logger.info(
                        "[TRAFFIC] ARBITER_REQUEST robot=%s chain=%s goal=%s",
                        robot_id,
                        chain_path.chain_id,
                        goal_node,
                    )
                    result = self._attempt_chain_leg(job)
                    result["job_id"] = job.job_id
                    return result
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

            for job in list(self._chain_jobs.values()):
                if job.status in {
                    JobStatus.BLOCKED,
                    JobStatus.COMPLETE,
                    JobStatus.CANCELLED,
                }:
                    continue
                if job.status is JobStatus.ACTIVE:
                    if (
                        job.active_plan is None
                        or not self.tracker.has_arrived(
                            job.robot_id,
                            job.active_plan.goal_node,
                        )
                    ):
                        continue
                    self.arbiter.mark_authority_arrived(job.robot_id)
                    job.leg_count += 1
                    arrived_goal = job.active_plan.goal_node
                    job.active_plan = None
                    job.active_authority_id = None
                    job.upstream_result = None
                    job.last_error = None
                    if arrived_goal == job.final_goal_node:
                        job.status = JobStatus.COMPLETE
                        logger.info(
                            "[TRAFFIC] TASK_COMPLETED robot=%s job=%s",
                            job.robot_id,
                            job.job_id,
                        )
                        continue
                    job.status = JobStatus.WAITING
                self._attempt_chain_leg(job)

    def cancel(self, job_id: str) -> bool:
        """Cancel a task that has not yet been submitted upstream."""
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None:
                chain_job = self._chain_jobs.get(job_id)
                if chain_job is None or chain_job.status in {
                    JobStatus.ACTIVE,
                    JobStatus.COMPLETE,
                    JobStatus.CANCELLED,
                }:
                    return False
                self.arbiter.cancel_authority(chain_job.robot_id)
                chain_job.status = JobStatus.CANCELLED
                logger.info(
                    "[TRAFFIC] TASK_CANCELLED robot=%s job=%s",
                    chain_job.robot_id,
                    chain_job.job_id,
                )
                return True
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
            for job in self._chain_jobs.values():
                if job.status not in {JobStatus.COMPLETE, JobStatus.CANCELLED}:
                    job.status = JobStatus.CANCELLED
            return True

    def status(self) -> dict[str, Any]:
        with self._lock:
            jobs = {
                job_id: {
                    "robot_id": job.robot_id,
                    "route_id": job.route.route_id,
                    "step_index": job.step_index,
                    "step_count": len(job.route.steps),
                    "status": job.status.value,
                    "last_error": job.last_error,
                }
                for job_id, job in sorted(self._jobs.items())
            }
            jobs.update(
                {
                    job_id: {
                        "robot_id": job.robot_id,
                        "route_id": f"chain:{job.chain_id}",
                        "chain_id": job.chain_id,
                        "final_goal_node": job.final_goal_node,
                        "step_index": job.leg_count,
                        "step_count": None,
                        "status": job.status.value,
                        "last_error": job.last_error,
                        "active_authority_id": job.active_authority_id,
                        "source_slot": (
                            job.active_plan.source_slot if job.active_plan else None
                        ),
                        "destination_slot": (
                            job.active_plan.destination_slot if job.active_plan else None
                        ),
                        "blocks": (
                            list(job.active_plan.block_ids) if job.active_plan else []
                        ),
                    }
                    for job_id, job in sorted(self._chain_jobs.items())
                }
            )
            return {
                "enabled": self.registry.enabled,
                "deployment": (
                    self.readiness.profile.redacted_snapshot()
                    if self.readiness is not None
                    else {"mode": "simulation"}
                ),
                "readiness": (
                    self.readiness.ready()
                    if self.readiness is not None
                    else {"ready": True, "reason": "simulation"}
                ),
                "jobs": jobs,
                "arbiter": self.arbiter.snapshot(),
                "robots": self.tracker.snapshot(),
            }

    def _attempt_chain_leg(self, job: ChainGateJob) -> dict[str, Any]:
        runtime_rejection = self._runtime_rejection()
        if runtime_rejection is not None:
            job.status = JobStatus.WAITING
            return runtime_rejection
        eligibility = self.tracker.eligibility(job.robot_id)
        if not eligibility.eligible:
            job.status = JobStatus.WAITING
            return {
                "decision": Decision.BLOCKED.value,
                "reason": "robot_not_eligible",
                "eligibility_reasons": list(eligibility.reasons),
            }
        if job.active_plan is not None:
            decision = self.arbiter.decision_for_authority(job.robot_id)
            if decision is Decision.ADMIT:
                return self._forward_chain_leg(job)
            if decision is Decision.BLOCKED:
                job.status = JobStatus.BLOCKED
                return {"decision": Decision.BLOCKED.value}

        start_node = self.tracker.current_safe_node(job.robot_id)
        if start_node is None:
            job.status = JobStatus.WAITING
            return {"decision": Decision.WAIT.value}
        path = self.registry.resolve_chain_path(start_node, job.final_goal_node)
        if path is None:
            if start_node == job.final_goal_node:
                job.status = JobStatus.COMPLETE
                return {"decision": "COMPLETE"}
            job.status = JobStatus.BLOCKED
            job.last_error = "chain_path_unavailable"
            return {
                "decision": Decision.BLOCKED.value,
                "reason": job.last_error,
            }
        if self._older_opposite_chain_waiter(job, path):
            job.status = JobStatus.WAITING
            return {"decision": Decision.WAIT.value}
        plan = self._chain_planner.plan(
            path,
            lambda block_ids, direction, destination_slot: (
                self.arbiter.can_reserve_path(
                    block_ids,
                    direction,
                    destination_slot,
                    robot_id=job.robot_id,
                    chain_id=path.chain_id,
                )
            ),
        )
        if plan is None:
            job.status = JobStatus.WAITING
            return {"decision": Decision.WAIT.value}
        decision = self.arbiter.request_authority(plan, robot_id=job.robot_id)
        if decision is Decision.WAIT:
            job.status = JobStatus.WAITING
            return {"decision": Decision.WAIT.value}
        if decision is Decision.BLOCKED:
            job.status = JobStatus.BLOCKED
            return {"decision": Decision.BLOCKED.value}
        job.active_plan = plan
        authority = self.arbiter.authority_for_robot(job.robot_id)
        job.active_authority_id = authority.authority_id if authority else None
        return self._forward_chain_leg(job)

    def _forward_chain_leg(self, job: ChainGateJob) -> dict[str, Any]:
        assert job.active_plan is not None
        staged = deepcopy(job.original_payload)
        description = staged.setdefault("request", {}).setdefault("description", {})
        description["places"] = [job.active_plan.goal_node]
        description["rounds"] = 1
        try:
            result = self.forwarder(staged)
        except urllib_error.HTTPError as error:
            if 400 <= error.code < 500:
                self.arbiter.cancel_authority(job.robot_id)
                job.status = JobStatus.BLOCKED
                job.last_error = str(error)
                return {
                    "decision": Decision.BLOCKED.value,
                    "reason": "upstream_http_rejection",
                    "status_code": error.code,
                    "error": str(error),
                }
            job.status = JobStatus.ACTIVE
            job.last_error = str(error)
            return {"decision": "FORWARD_UNKNOWN", "error": str(error)}
        except Exception as error:
            job.status = JobStatus.ACTIVE
            job.last_error = str(error)
            return {"decision": "FORWARD_UNKNOWN", "error": str(error)}

        if isinstance(result, dict) and result.get("success") is False:
            job.status = JobStatus.RETRY
            job.last_error = str(
                result.get("message") or result.get("error") or "upstream rejected task"
            )
            return {"decision": "RETRY", "error": job.last_error}

        job.status = JobStatus.ACTIVE
        job.upstream_result = result
        job.last_error = None
        logger.info(
            "[TRAFFIC] TASK_RELEASED robot=%s job=%s authority=%s blocks=%s goal=%s",
            job.robot_id,
            job.job_id,
            job.active_authority_id,
            ",".join(job.active_plan.block_ids),
            job.active_plan.goal_node,
        )
        return {"decision": Decision.ADMIT.value, "upstream": result}

    def _older_opposite_chain_waiter(self, job: ChainGateJob, path) -> bool:
        candidate_blocks = set(path.block_ids)
        for other in self._chain_jobs.values():
            if (
                other.job_id == job.job_id
                or other.status is not JobStatus.WAITING
                or other.request_time >= job.request_time
            ):
                continue
            other_start = self.tracker.current_safe_node(other.robot_id)
            if other_start is None:
                continue
            other_path = self.registry.resolve_chain_path(
                other_start,
                other.final_goal_node,
            )
            if (
                other_path is not None
                and other_path.direction is path.direction.opposite
                and candidate_blocks.intersection(other_path.block_ids)
            ):
                destination_slot = self.registry.holding_bay_for_node(
                    other.final_goal_node
                )
                if destination_slot is not None and self._slot_depends_on_direction(
                    destination_slot,
                    path.direction,
                ):
                    continue
                return True
        return False

    def _slot_depends_on_direction(
        self,
        slot_id: str,
        direction,
    ) -> bool:
        occupants = self.registry.holding_bays[slot_id].occupants
        for occupant in occupants:
            occupant_job = next(
                (
                    item
                    for item in self._chain_jobs.values()
                    if item.robot_id == occupant
                    and item.status
                    not in {
                        JobStatus.COMPLETE,
                        JobStatus.CANCELLED,
                        JobStatus.BLOCKED,
                    }
                ),
                None,
            )
            if occupant_job is None:
                continue
            start_node = self.tracker.current_safe_node(occupant)
            if start_node is None:
                continue
            occupant_path = self.registry.resolve_chain_path(
                start_node,
                occupant_job.final_goal_node,
            )
            if occupant_path is not None and occupant_path.direction is direction:
                return True
        return False

    def _attempt_current_step(self, job: GateJob) -> dict[str, Any]:
        runtime_rejection = self._runtime_rejection()
        if runtime_rejection is not None:
            job.status = JobStatus.WAITING
            return runtime_rejection
        eligibility = self.tracker.eligibility(job.robot_id)
        if not eligibility.eligible:
            job.status = JobStatus.WAITING
            return {
                "decision": Decision.BLOCKED.value,
                "reason": "robot_not_eligible",
                "eligibility_reasons": list(eligibility.reasons),
            }
        self._refresh_waiting_route(job)
        step = job.current_step
        if step is None:
            job.status = JobStatus.COMPLETE
            return {"decision": "COMPLETE"}

        if self._guarded_route_is_pending(job):
            self.arbiter.cancel(job.robot_id, step.block_id)
            job.status = JobStatus.WAITING
            logger.info(
                "[TRAFFIC] TASK_HELD robot=%s job=%s block=%s "
                "reason=guarded_route_pending",
                job.robot_id,
                job.job_id,
                step.block_id,
            )
            return {"decision": Decision.WAIT.value}

        if (
            step.requires_opposite_routes_cleared
            and self._opposite_route_work_remains(step, exclude_job_id=job.job_id)
        ):
            self.arbiter.cancel(job.robot_id, step.block_id)
            job.status = JobStatus.WAITING
            logger.info(
                "[TRAFFIC] TASK_HELD robot=%s job=%s block=%s "
                "reason=opposite_routes_pending",
                job.robot_id,
                job.job_id,
                step.block_id,
            )
            return {"decision": Decision.WAIT.value}

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
            if (
                job.step_index == 0
                and not job.route.requires_no_opposite_jobs
                and self._has_guarded_alternative(job)
            ):
                job.wait_for_guarded_route = True
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

    def _opposite_route_work_remains(
        self,
        candidate_step: RouteStep,
        *,
        exclude_job_id: str,
    ) -> bool:
        candidate_domain = self.registry.blocks[
            candidate_step.block_id
        ].direction_domain
        terminal = {JobStatus.COMPLETE, JobStatus.CANCELLED}
        for other in self._jobs.values():
            if other.job_id == exclude_job_id or other.status in terminal:
                continue
            for offset, other_step in enumerate(
                other.route.steps[other.step_index :]
            ):
                other_block = self.registry.blocks[other_step.block_id]
                if (
                    other_block.direction_domain != candidate_domain
                    or other_step.direction is not candidate_step.direction.opposite
                ):
                    continue
                if (
                    offset == 0
                    and other.status is JobStatus.ACTIVE
                    and self.arbiter.has_cleared(
                        other.robot_id,
                        other_step.block_id,
                    )
                ):
                    continue
                return True
        return False

    def _select_route(
        self,
        routes: list[RouteIntent],
        *,
        exclude_job_id: str | None = None,
    ) -> RouteIntent | None:
        eligible = [
            route
            for route in routes
            if self._route_is_eligible(route, exclude_job_id=exclude_job_id)
        ]
        guarded = [route for route in eligible if route.requires_no_opposite_jobs]
        return guarded[0] if guarded else (eligible[0] if eligible else None)

    def _route_is_eligible(
        self,
        route: RouteIntent,
        *,
        exclude_job_id: str | None = None,
    ) -> bool:
        if not route.requires_no_opposite_jobs:
            return True

        terminal = {JobStatus.COMPLETE, JobStatus.CANCELLED}
        for candidate_step in route.steps:
            candidate_block = self.registry.blocks[candidate_step.block_id]
            for other in self._jobs.values():
                if other.job_id == exclude_job_id or other.status in terminal:
                    continue
                for other_step in other.route.steps[other.step_index :]:
                    other_block = self.registry.blocks[other_step.block_id]
                    if (
                        other_block.direction_domain
                        == candidate_block.direction_domain
                        and other_step.direction is candidate_step.direction.opposite
                    ):
                        return False
        return True

    def _has_guarded_alternative(self, job: GateJob) -> bool:
        start_node = self.tracker.current_safe_node(job.robot_id)
        if start_node is None:
            return False
        routes = self.registry.resolve_routes(
            start_node,
            _goal_node(job.original_payload),
        )
        return any(route.requires_no_opposite_jobs for route in routes)

    def _guarded_route_is_pending(self, job: GateJob) -> bool:
        if not job.wait_for_guarded_route or job.step_index != 0:
            return False

        start_node = self.tracker.current_safe_node(job.robot_id)
        if start_node is None:
            return True
        routes = self.registry.resolve_routes(
            start_node,
            _goal_node(job.original_payload),
        )
        guarded = [route for route in routes if route.requires_no_opposite_jobs]
        if not guarded:
            job.wait_for_guarded_route = False
            return False
        if any(
            self._route_is_eligible(route, exclude_job_id=job.job_id)
            for route in guarded
        ):
            job.wait_for_guarded_route = False
            return False
        return True

    def _refresh_waiting_route(self, job: GateJob) -> None:
        if job.status is not JobStatus.WAITING or job.step_index != 0:
            return
        start_node = self.tracker.current_safe_node(job.robot_id)
        if start_node is None or start_node not in job.route.start_nodes:
            return
        routes = self.registry.resolve_routes(
            start_node,
            _goal_node(job.original_payload),
        )
        selected = self._select_route(routes, exclude_job_id=job.job_id)
        if selected is None or selected.route_id == job.route.route_id:
            return

        previous_step = job.current_step
        if previous_step is not None:
            self.arbiter.cancel(job.robot_id, previous_step.block_id)
        previous_route_id = job.route.route_id
        job.route = selected
        if selected.requires_no_opposite_jobs:
            job.wait_for_guarded_route = False
        logger.info(
            "[TRAFFIC] ROUTE_SWITCH robot=%s job=%s from=%s to=%s",
            job.robot_id,
            job.job_id,
            previous_route_id,
            selected.route_id,
        )

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

    def _active_job_for_robot(
        self, robot_id: str
    ) -> GateJob | ChainGateJob | None:
        terminal = {JobStatus.COMPLETE, JobStatus.CANCELLED, JobStatus.BLOCKED}
        legacy = next(
            (
                job
                for job in self._jobs.values()
                if job.robot_id == robot_id and job.status not in terminal
            ),
            None,
        )
        if legacy is not None:
            return legacy
        return next(
            (
                job
                for job in self._chain_jobs.values()
                if job.robot_id == robot_id and job.status not in terminal
            ),
            None,
        )

    def _runtime_rejection(self) -> dict[str, Any] | None:
        if self.readiness is None:
            return None
        status = self.readiness.ready()
        if status["ready"]:
            return None
        return {
            "decision": Decision.BLOCKED.value,
            "reason": "runtime_not_ready",
            "readiness": status,
        }


class HttpTaskForwarder:
    """Small no-proxy JSON client for the existing RMF API server."""

    def __init__(
        self,
        url: str,
        *,
        timeout: float = 10.0,
        bearer_token: str | None = None,
    ) -> None:
        self.url = url
        self.timeout = float(timeout)
        self.bearer_token = (
            os.environ.get("RMF_API_BEARER_TOKEN", "")
            if bearer_token is None
            else bearer_token
        ).strip()
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

    def probe(self) -> bool:
        parsed = urllib_parse.urlsplit(self.url)
        root = urllib_parse.urlunsplit(
            (parsed.scheme, parsed.netloc, "/", "", "")
        )
        request = urllib_request.Request(root, method="GET")
        try:
            with self._opener.open(request, timeout=min(self.timeout, 2.0)):
                return True
        except urllib_error.HTTPError as error:
            return error.code == 404
        except (OSError, urllib_error.URLError):
            return False


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

    @app.get("/health")
    def health():
        if gate.readiness is None:
            return {"healthy": True, "dependencies": {}}
        return gate.readiness.health()

    @app.get("/ready")
    def ready():
        status = (
            gate.readiness.ready()
            if gate.readiness is not None
            else {"ready": True, "reason": "simulation"}
        )
        if not status["ready"]:
            raise HTTPException(status_code=503, detail=status)
        return status

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
    parser.add_argument("--deployment-profile")
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
    profile = None
    if args.deployment_profile:
        from .deployment import DeploymentProfile

        profile = DeploymentProfile.load(Path(args.deployment_profile))
        profile.require_valid()
    config_path = (
        profile.paths.corridor_config
        if profile is not None and profile.paths.corridor_config is not None
        else Path(args.config)
    )
    registry = CorridorRegistry.from_yaml(config_path)
    arbiter = DirectionArbiter(registry)
    eligibility_policy = None
    if profile is not None and profile.telemetry.operational_checks_required:
        from .eligibility import RobotEligibilityPolicy

        eligibility_policy = RobotEligibilityPolicy(
            robots=profile.robots,
            state_timeout=profile.telemetry.state_timeout,
            connection_timeout=profile.telemetry.connection_timeout,
        )
    tracker = RobotTracker(
        registry,
        arbiter,
        telemetry_timeout=float(registry.settings.get("telemetry_timeout", 5.0)),
        eligibility_policy=eligibility_policy,
    )
    upstream = profile.rmf_api.url if profile is not None else args.upstream
    token = (
        profile.rmf_api.bearer_token.resolve(os.environ)
        if profile is not None and profile.rmf_api.bearer_token is not None
        else None
    )
    forwarder = HttpTaskForwarder(upstream, bearer_token=token)
    monitor = (
        MqttStateMonitor(
            tracker,
            host=(profile.mqtt.host if profile is not None else args.mqtt_host),
            port=(profile.mqtt.port if profile is not None else args.mqtt_port),
            topic=(
                profile.mqtt.state_topic if profile is not None else args.mqtt_topic
            ),
            mqtt_config=(profile.mqtt if profile is not None else None),
        )
        if registry.enabled
        else None
    )
    readiness = None
    if profile is not None:
        from .readiness import RuntimeReadiness

        readiness = RuntimeReadiness(
            profile,
            registry,
            tracker,
            mqtt_connected=lambda: bool(monitor and monitor.is_connected),
            rmf_probe=forwarder.probe,
        )
    gate = TaskGate(
        registry, arbiter, tracker, forwarder, readiness=readiness
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
