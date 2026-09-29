"""Config-driven VDA5050 telemetry fault injection for the simulator."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


_TRIGGERS = {"elapsed_at_least", "at_node", "driving"}
_STATE_ACTIONS = {
    "set_estop",
    "set_operating_mode",
    "set_paused",
    "set_map_id",
    "set_position_initialized",
}
_ACTIONS = _STATE_ACTIONS | {"suppress_state", "publish_connection"}
_CONNECTION_STATES = {"ONLINE", "OFFLINE", "CONNECTIONBROKEN"}


@dataclass(frozen=True)
class FaultEffects:
    triggered: bool = False
    state_overrides: dict[str, Any] = field(default_factory=dict)
    suppress_state: bool = False
    connection_states: tuple[str, ...] = ()


@dataclass(frozen=True)
class _Rule:
    robot: str
    when: Mapping[str, Any]
    action_type: str
    value: Any


class FaultInjectionController:
    """Activate deterministic, one-shot rules and retain their state effects."""

    def __init__(self, rules: tuple[_Rule, ...]) -> None:
        self._rules = rules
        self._fired: set[int] = set()
        self._state_overrides: dict[str, dict[str, Any]] = {}
        self._suppress_state: dict[str, bool] = {}

    @classmethod
    def from_config(
        cls, items: Iterable[Mapping[str, Any]] | None
    ) -> "FaultInjectionController":
        rules: list[_Rule] = []
        for index, item in enumerate(items or ()):
            if not isinstance(item, Mapping):
                raise ValueError(f"fault_injection.rules[{index}].invalid")
            robot = item.get("robot")
            when = item.get("when", {})
            action = item.get("action", {})
            if not isinstance(robot, str) or not robot.strip():
                raise ValueError(f"fault_injection.rules[{index}].robot.invalid")
            if not isinstance(when, Mapping) or not set(when).issubset(_TRIGGERS):
                raise ValueError(f"fault_injection.rules[{index}].when.invalid")
            if not isinstance(action, Mapping):
                raise ValueError(f"fault_injection.rules[{index}].action.invalid")
            action_type = action.get("type")
            if action_type not in _ACTIONS or "value" not in action:
                raise ValueError(f"fault_injection.rules[{index}].action.invalid")
            value = action["value"]
            cls._validate_value(index, action_type, value)
            cls._validate_trigger(index, when)
            rules.append(_Rule(robot.strip(), dict(when), action_type, value))
        return cls(tuple(rules))

    @staticmethod
    def _validate_trigger(index: int, when: Mapping[str, Any]) -> None:
        if "elapsed_at_least" in when:
            value = when["elapsed_at_least"]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                raise ValueError(f"fault_injection.rules[{index}].elapsed.invalid")
        if "at_node" in when and (
            not isinstance(when["at_node"], str) or not when["at_node"]
        ):
            raise ValueError(f"fault_injection.rules[{index}].node.invalid")
        if "driving" in when and not isinstance(when["driving"], bool):
            raise ValueError(f"fault_injection.rules[{index}].driving.invalid")

    @staticmethod
    def _validate_value(index: int, action_type: str, value: Any) -> None:
        if action_type in {"set_paused", "set_position_initialized", "suppress_state"}:
            valid = isinstance(value, bool)
        elif action_type == "publish_connection":
            valid = value in _CONNECTION_STATES
        else:
            valid = isinstance(value, str) and bool(value)
        if not valid:
            raise ValueError(f"fault_injection.rules[{index}].value.invalid")

    def apply(self, robot: Any, elapsed: float) -> FaultEffects:
        robot_id = str(robot.serial_number)
        triggered = False
        connection_states: list[str] = []
        for index, rule in enumerate(self._rules):
            if index in self._fired or rule.robot != robot_id:
                continue
            if not self._matches(rule.when, robot, elapsed):
                continue
            self._fired.add(index)
            triggered = True
            if rule.action_type == "publish_connection":
                connection_states.append(str(rule.value))
            elif rule.action_type == "suppress_state":
                self._suppress_state[robot_id] = bool(rule.value)
            else:
                self._apply_state_action(robot_id, rule.action_type, rule.value)
        return FaultEffects(
            triggered=triggered,
            state_overrides=deepcopy(self._state_overrides.get(robot_id, {})),
            suppress_state=self._suppress_state.get(robot_id, False),
            connection_states=tuple(connection_states),
        )

    @staticmethod
    def _matches(when: Mapping[str, Any], robot: Any, elapsed: float) -> bool:
        return (
            elapsed >= float(when.get("elapsed_at_least", 0.0))
            and ("at_node" not in when or robot.last_node_id == when["at_node"])
            and ("driving" not in when or robot.driving is when["driving"])
        )

    def _apply_state_action(self, robot_id: str, action_type: str, value: Any) -> None:
        overrides = self._state_overrides.setdefault(robot_id, {})
        if action_type == "set_estop":
            overrides.setdefault("safetyState", {})["eStop"] = value
        elif action_type == "set_operating_mode":
            overrides["operatingMode"] = value
        elif action_type == "set_paused":
            overrides["paused"] = value
        elif action_type == "set_map_id":
            overrides.setdefault("agvPosition", {})["mapId"] = value
        elif action_type == "set_position_initialized":
            overrides.setdefault("agvPosition", {})["positionInitialized"] = value


def apply_overrides(target: dict[str, Any], overrides: Mapping[str, Any]) -> None:
    """Recursively overlay state fields without modifying the Robot object."""
    for key, value in overrides.items():
        if isinstance(value, Mapping) and isinstance(target.get(key), dict):
            apply_overrides(target[key], value)
        else:
            target[key] = deepcopy(value)
