"""Pure VDA5050 operational eligibility decisions for corridor admission."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping

from .deployment import RobotDeploymentConfig

if TYPE_CHECKING:
    from .robot_tracker import RobotTelemetry


@dataclass(frozen=True)
class EligibilityResult:
    eligible: bool
    reasons: tuple[str, ...]


class RobotEligibilityPolicy:
    def __init__(
        self,
        *,
        robots: Mapping[str, RobotDeploymentConfig],
        state_timeout: float,
        connection_timeout: float,
        blocking_error_levels: tuple[str, ...] = ("FATAL",),
    ) -> None:
        self.robots = dict(robots)
        self.state_timeout = float(state_timeout)
        self.connection_timeout = float(connection_timeout)
        self.blocking_error_levels = frozenset(
            item.upper() for item in blocking_error_levels
        )

    def evaluate(
        self,
        telemetry: "RobotTelemetry | None",
        now: float,
    ) -> EligibilityResult:
        if telemetry is None:
            return EligibilityResult(False, ("telemetry.missing",))
        reasons: set[str] = set()
        robot = self.robots.get(telemetry.robot_id)
        if robot is None:
            reasons.add("robot.unregistered")
        if telemetry.state_header_id is None:
            reasons.add("state.missing")
        elif now - telemetry.received_at > self.state_timeout:
            reasons.add("state.stale")
        if telemetry.connection_received_at is None:
            reasons.add("connection.missing")
        if telemetry.connection_state != "ONLINE":
            reasons.add("connection.offline")
        if not telemetry.position_initialized:
            reasons.add("position.uninitialized")
        if robot is not None and telemetry.map_id not in robot.allowed_map_ids:
            reasons.add("position.map_mismatch")
        if telemetry.operating_mode != "AUTOMATIC":
            reasons.add("mode.not_automatic")
        if telemetry.e_stop != "NONE":
            reasons.add("safety.estop")
        if telemetry.field_violation:
            reasons.add("safety.field_violation")
        if telemetry.paused:
            reasons.add("state.paused")
        if self.blocking_error_levels.intersection(telemetry.error_levels):
            reasons.add("errors.blocking")
        if telemetry.current_hb is None and telemetry.current_block is None:
            reasons.add("position.outside_managed_area")
        ordered = tuple(sorted(reasons))
        return EligibilityResult(not ordered, ordered)
