"""Runtime health and fail-closed production clean-start readiness."""

from __future__ import annotations

import time
from typing import Any, Callable

from .corridor_registry import CorridorRegistry
from .deployment import DeploymentProfile
from .robot_tracker import RobotTracker


class RuntimeReadiness:
    def __init__(
        self,
        profile: DeploymentProfile,
        registry: CorridorRegistry,
        tracker: RobotTracker,
        *,
        mqtt_connected: Callable[[], bool],
        rmf_probe: Callable[[], bool],
        probe_cache_seconds: float = 1.0,
    ) -> None:
        self.profile = profile
        self.registry = registry
        self.tracker = tracker
        self._mqtt_connected = mqtt_connected
        self._rmf_probe = rmf_probe
        self._probe_cache_seconds = float(probe_cache_seconds)
        self._last_probe_at: float | None = None
        self._last_probe_result = False
        self._clean_start_complete = False
        self._recovery_required = False

    def _mqtt_ok(self) -> bool:
        try:
            return bool(self._mqtt_connected())
        except Exception:
            return False

    def _rmf_ok(self, now: float, *, cached: bool = True) -> bool:
        if (
            cached
            and self._last_probe_at is not None
            and now - self._last_probe_at <= self._probe_cache_seconds
        ):
            return self._last_probe_result
        try:
            result = bool(self._rmf_probe())
        except Exception:
            result = False
        self._last_probe_at = now
        self._last_probe_result = result
        return result

    def health(self) -> dict[str, Any]:
        now = time.monotonic()
        return {
            "healthy": True,
            "dependencies": {
                "mqtt": self._mqtt_ok(),
                "rmf_api": self._rmf_ok(now, cached=False),
            },
        }

    def ready(self, *, now: float | None = None) -> dict[str, Any]:
        current = time.monotonic() if now is None else float(now)
        validation_errors = tuple(self.profile.validate())
        if validation_errors:
            return self._result(False, "preflight.invalid", validation_errors)
        if not self._mqtt_ok():
            return self._result(False, "mqtt.disconnected")
        if not self._rmf_ok(current):
            return self._result(False, "rmf.unavailable")
        if self._recovery_required:
            return self._result(False, "recovery.required")

        required = tuple(
            robot_id
            for robot_id, config in self.profile.robots.items()
            if config.required
        )
        telemetry = {
            robot_id: self.tracker.telemetry(robot_id) for robot_id in required
        }
        if any(item is None for item in telemetry.values()):
            return self._result(False, "telemetry.pending")

        reasons = self._eligibility_reasons(required, current)
        if reasons:
            return self._result(False, "robots.ineligible", reasons)

        if not self._clean_start_complete:
            states = tuple(item for item in telemetry.values() if item is not None)
            if any(
                item.current_block is not None
                or item.driving
                or item.faulted
                for item in states
            ):
                self._recovery_required = True
                return self._result(False, "recovery.required")
            unconfirmed = tuple(
                sorted(
                    item.robot_id
                    for item in states
                    if self.tracker.current_safe_node(item.robot_id) is None
                )
            )
            if unconfirmed:
                return self._result(False, "safe_stop.unconfirmed", unconfirmed)
            if any(block.occupants for block in self.registry.blocks.values()):
                self._recovery_required = True
                return self._result(False, "recovery.required")
            self._clean_start_complete = True

        return self._result(True, "ready")

    def _eligibility_reasons(
        self, robot_ids: tuple[str, ...], now: float
    ) -> tuple[str, ...]:
        return tuple(sorted(
            f"{robot_id}:{reason}"
            for robot_id in robot_ids
            for reason in self.tracker.eligibility(robot_id, now=now).reasons
        ))

    def _result(
        self,
        ready: bool,
        reason: str,
        details: tuple[str, ...] = (),
    ) -> dict[str, Any]:
        return {
            "ready": ready,
            "reason": reason,
            "details": list(details),
            "recovery_required": self._recovery_required,
            "clean_start_complete": self._clean_start_complete,
        }

    def recovery_check(self, *, now: float | None = None) -> dict[str, Any]:
        """Check whether an operator recovery acknowledgement is safe."""
        current = time.monotonic() if now is None else float(now)
        validation_errors = tuple(self.profile.validate())
        if validation_errors:
            return self._result(False, "preflight.invalid", validation_errors)
        if not self._mqtt_ok():
            return self._result(False, "mqtt.disconnected")
        if not self._rmf_ok(current):
            return self._result(False, "rmf.unavailable")

        required = tuple(
            robot_id
            for robot_id, config in self.profile.robots.items()
            if config.required
        )
        telemetry = {
            robot_id: self.tracker.telemetry(robot_id) for robot_id in required
        }
        if any(item is None for item in telemetry.values()):
            return self._result(False, "telemetry.pending")

        states = tuple(item for item in telemetry.values() if item is not None)
        if any(
            item.current_block is not None
            or item.current_hb is None
            or item.driving
            for item in states
        ):
            return self._result(False, "recovery.robot_not_safe")

        reasons = self._eligibility_reasons(required, current)
        if reasons:
            return self._result(False, "robots.ineligible", reasons)
        if any(
            block.occupants or block.reservations
            for block in self.registry.blocks.values()
        ):
            return self._result(False, "recovery.arbiter_not_clear")
        return self._result(True, "recovery.safe_to_ack")

    def acknowledge_recovery(self, *, now: float | None = None) -> dict[str, Any]:
        status = self.recovery_check(now=now)
        if not status["ready"]:
            return status
        required = tuple(
            robot_id
            for robot_id, config in self.profile.robots.items()
            if config.required
        )
        if not all(self.tracker.clear_fault_if_safe(robot_id) for robot_id in required):
            return self._result(False, "recovery.robot_not_safe")
        self._recovery_required = False
        self._clean_start_complete = True
        return self._result(True, "ready")

    def can_accept_tasks(self, *, now: float | None = None) -> bool:
        return bool(self.ready(now=now)["ready"])
