"""Command ACK HSM for VDA5050 robot commands.

This module owns the per-robot command queue, transition states, ACK guards,
and command transition logging. RobotAdapter remains responsible for RMF
workflow decisions and command payload construction.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import logging
import threading
import time
from typing import Any, Callable

from vda5050_fleet_adapter.usecase.ports.robot_api import (
    RobotAPI, RobotAPIResult, RobotCommandState,
)

logger = logging.getLogger(__name__)


class CommandTopState(Enum):
    AVAILABLE = 'AVAILABLE'
    BLOCKED = 'BLOCKED'
    FAULTED = 'FAULTED'
    DECOMMISSIONED = 'DECOMMISSIONED'


class CommandState(Enum):
    IDLE = 'IDLE'
    SENDING_CANCEL = 'SENDING_CANCEL'
    WAIT_CANCEL_ACK = 'WAIT_CANCEL_ACK'
    SENDING_ORDER = 'SENDING_ORDER'
    WAIT_ORDER_ACK = 'WAIT_ORDER_ACK'
    NAVIGATING = 'NAVIGATING'
    SENDING_ACTION = 'SENDING_ACTION'
    WAIT_ACTION_ACK = 'WAIT_ACTION_ACK'


class CommandKind(Enum):
    CANCEL_ORDER = 'cancel_order'
    CANCEL_ACTION = 'cancel_action'
    ORDER = 'order'
    ACTION = 'action'
    PAUSE = 'pause'
    GENERIC = 'generic'


class TaskContext(Enum):
    DISPATCHED_TASK = 'DISPATCHED_TASK'
    IDLE_BEHAVIOR = 'IDLE_BEHAVIOR'
    CHARGING_FLOW = 'CHARGING_FLOW'
    NEGOTIATION_PAUSE = 'NEGOTIATION_PAUSE'


@dataclass
class QueuedCommand:
    event_id: str
    kind: CommandKind
    cmd_id: int
    send: Callable[[], RobotAPIResult]
    description: str
    ack_guard: Callable[[RobotCommandState, float], bool] | None = None
    ack_description: str = ''
    on_sent: Callable[[float], None] | None = None
    on_acked: Callable[[], None] | None = None
    order_id: str | None = None
    order_update_id: int | None = None
    action_id: str | None = None
    target_kind: str | None = None
    retry_interval: float = 1.0
    ack_timeout: float = 30.0
    resend_on_timeout: bool = True
    cancel_settle_enabled: bool = False
    cancel_settle_after: float = 2.0
    cancel_settle_fresh_states: int = 2
    cancel_settle_require_stopped: bool = True
    cancel_block_after: float = 10.0
    cancel_fresh_state_times: set[float] = field(default_factory=set)
    sent_at: float | None = None
    last_attempt_at: float = 0.0
    attempt_count: int = 0
    timeout_logged: bool = False


def enum_value(value: Any) -> str:
    if value is None:
        return ''
    return str(getattr(value, 'value', value))


def cancel_order_ack_guard() -> Callable[[RobotCommandState, float], bool]:
    def guard(state: RobotCommandState, sent_at: float) -> bool:
        return (
            state.state_received_at >= sent_at
            and len(state.node_states) == 0
            and len(state.edge_states) == 0
            and not state.driving
        )
    return guard


def cancel_action_ack_guard(
    action_id: str,
) -> Callable[[RobotCommandState, float], bool]:
    def guard(state: RobotCommandState, sent_at: float) -> bool:
        if state.state_received_at < sent_at:
            return False
        return all(
            getattr(action_state, 'action_id', None) != action_id
            for action_state in state.action_states
        )
    return guard


def order_ack_guard(
    order_id: str | None,
    order_update_id: int,
) -> Callable[[RobotCommandState, float], bool]:
    expected_order_id = order_id or ''

    def guard(state: RobotCommandState, sent_at: float) -> bool:
        return (
            state.state_received_at >= sent_at
            and state.order_id == expected_order_id
            and state.order_update_id >= order_update_id
        )
    return guard


def action_ack_guard(
    action_id: str,
) -> Callable[[RobotCommandState, float], bool]:
    def guard(state: RobotCommandState, sent_at: float) -> bool:
        if state.state_received_at < sent_at:
            return False
        for action_state in state.action_states:
            if getattr(action_state, 'action_id', None) != action_id:
                continue
            status = enum_value(getattr(action_state, 'action_status', None))
            return status == 'FINISHED'
        return False
    return guard


class CommandAckHsm:
    """Per-robot command queue and ACK state machine."""

    def __init__(
        self,
        robot_name: str,
        api: RobotAPI,
        task_context: Callable[[], TaskContext],
    ) -> None:
        self._robot_name = robot_name
        self._api = api
        self._task_context = task_context
        self._lock = threading.RLock()
        self._queue: deque[QueuedCommand] = deque()
        self._active_command: QueuedCommand | None = None
        self._top_state = CommandTopState.AVAILABLE
        self._state = CommandState.IDLE
        self._event_seq = 0
        self._last_block_log_at = 0.0
        self._last_block_log_key: tuple[str, str] | None = None

    @property
    def active_command(self) -> QueuedCommand | None:
        return self._active_command

    @property
    def state(self) -> CommandState:
        return self._state

    @property
    def top_state(self) -> CommandTopState:
        return self._top_state

    def next_event_id(self) -> str:
        self._event_seq += 1
        return f'{self._robot_name}-{self._event_seq:06d}'

    def enqueue(self, command: QueuedCommand) -> None:
        with self._lock:
            self._queue.append(command)
            self._log_command_event(
                command, 'Prepared', self._sending_state(command.kind),
                detail=f'queue_depth={len(self._queue)}',
            )
            self.pump()

    def pump(self, *, force_retry: bool = False) -> None:
        with self._lock:
            snapshot, state_supported = self._get_command_state_snapshot()
            self._refresh_top_state(snapshot)
            candidate = self._active_command or (
                self._queue[0] if self._queue else None
            )
            if not self._command_allowed_in_top_state(candidate):
                self._log_command_blocked(candidate, self._top_state.value)
                return

            steps = 0
            while steps < 8:
                steps += 1
                if self._active_command is None:
                    if not self._queue:
                        self._state = CommandState.IDLE
                        return
                    self._active_command = self._queue.popleft()

                command = self._active_command
                now = time.monotonic()
                if command.sent_at is None:
                    if (
                        command.last_attempt_at > 0.0
                        and not force_retry
                        and now - command.last_attempt_at
                        < command.retry_interval
                    ):
                        return
                    self._state = self._sending_state(command.kind)
                    command.last_attempt_at = now
                    command.attempt_count += 1
                    try:
                        result = command.send()
                    except Exception:
                        logger.exception(
                            'Command send raised [%s]: event_id=%s, cmd_id=%d',
                            self._robot_name, command.event_id, command.cmd_id,
                        )
                        result = RobotAPIResult.RETRY
                    sent_at = time.monotonic()
                    if self._is_success(result):
                        command.sent_at = sent_at
                        command.cancel_fresh_state_times.clear()
                        command.timeout_logged = False
                        if command.on_sent is not None:
                            command.on_sent(sent_at)
                        wait_state = self._waiting_state(command.kind)
                        self._log_command_event(
                            command, 'Sent', wait_state,
                            detail=f'attempt={command.attempt_count}',
                        )
                        if command.ack_guard is None:
                            self._ack_active_command(command)
                            continue
                        if not state_supported:
                            self._ack_active_command(
                                command, detail='ack_condition=no_command_state_api'
                            )
                            continue
                        self._state = wait_state
                        if (
                            snapshot is not None
                            and command.ack_guard(snapshot, command.sent_at)
                        ):
                            self._ack_active_command(command)
                            continue
                        return
                    if self._is_impossible(result):
                        self._top_state = CommandTopState.BLOCKED
                        self._log_command_blocked(command, 'send_impossible')
                        return
                    if self._is_retry(result):
                        self._log_command_blocked(command, 'send_retry')
                        return
                    self._log_command_blocked(
                        command, f'unexpected_result={result!r}'
                    )
                    return

                self._state = self._waiting_state(command.kind)
                if not state_supported:
                    self._ack_active_command(command)
                    continue
                if (
                    snapshot is not None
                    and command.ack_guard is not None
                    and command.ack_guard(snapshot, command.sent_at)
                ):
                    self._ack_active_command(command)
                    continue
                if (
                    snapshot is not None
                    and self._cancel_settle_ack(command, snapshot, now)
                ):
                    self._ack_active_command(
                        command,
                        detail=self._cancel_settle_ack_detail(
                            command, snapshot, now
                        ),
                    )
                    continue
                if (
                    snapshot is not None
                    and self._cancel_settle_blocked(command, snapshot, now)
                ):
                    return
                now = time.monotonic()
                if (
                    command.ack_timeout > 0.0
                    and now - command.sent_at >= command.ack_timeout
                ):
                    if not command.timeout_logged:
                        self._log_command_event(
                            command, 'Timed out',
                            self._sending_state(command.kind)
                            if command.resend_on_timeout else self._state,
                            detail=(
                                f'ack_condition={command.ack_description}, '
                                f'elapsed={now - command.sent_at:.1f}s'
                            ),
                            level=logging.WARNING,
                        )
                        command.timeout_logged = True
                    if command.resend_on_timeout:
                        command.sent_at = None
                        command.timeout_logged = False
                    return
                return

    def cancel_pending(self) -> None:
        with self._lock:
            active = self._active_command
            if active is not None:
                self._log_command_event(
                    active, 'Blocked', CommandState.IDLE,
                    detail='reason=command_attempt_cancelled',
                    level=logging.WARNING,
                )
            self._active_command = None
            self._queue.clear()
            self._state = CommandState.IDLE

    @staticmethod
    def _is_cancel_command(command: QueuedCommand) -> bool:
        return command.kind in (
            CommandKind.CANCEL_ORDER, CommandKind.CANCEL_ACTION
        )

    def _record_cancel_fresh_state(
        self, command: QueuedCommand, snapshot: RobotCommandState
    ) -> None:
        if command.sent_at is None or snapshot.state_received_at < command.sent_at:
            return
        command.cancel_fresh_state_times.add(snapshot.state_received_at)

    def _cancel_settle_ack(
        self, command: QueuedCommand, snapshot: RobotCommandState, now: float
    ) -> bool:
        if (
            not command.cancel_settle_enabled
            or not self._is_cancel_command(command)
            or command.sent_at is None
        ):
            return False
        self._record_cancel_fresh_state(command, snapshot)
        if now - command.sent_at < command.cancel_settle_after:
            return False
        if len(command.cancel_fresh_state_times) < command.cancel_settle_fresh_states:
            return False
        if command.cancel_settle_require_stopped and snapshot.driving:
            return False
        return True

    def _cancel_settle_ack_detail(
        self, command: QueuedCommand, snapshot: RobotCommandState, now: float
    ) -> str:
        elapsed = now - command.sent_at if command.sent_at is not None else 0.0
        return (
            'ack_condition=cancel_settle_assumed, strict_ack=False, '
            f'target_kind={command.target_kind}, '
            f'fresh_states={len(command.cancel_fresh_state_times)}, '
            f'elapsed={elapsed:.1f}s, driving={snapshot.driving}, '
            f'node_states={len(snapshot.node_states)}, '
            f'edge_states={len(snapshot.edge_states)}, '
            f'action_states={len(snapshot.action_states)}'
        )

    def _cancel_settle_blocked(
        self, command: QueuedCommand, snapshot: RobotCommandState, now: float
    ) -> bool:
        if (
            not command.cancel_settle_enabled
            or not self._is_cancel_command(command)
            or command.sent_at is None
            or command.cancel_block_after <= 0.0
        ):
            return False
        elapsed = now - command.sent_at
        if elapsed < command.cancel_block_after:
            return False
        self._record_cancel_fresh_state(command, snapshot)
        self._top_state = CommandTopState.BLOCKED
        self._log_command_blocked(
            command,
            'cancel_settle_failed: '
            f'fresh_states={len(command.cancel_fresh_state_times)}, '
            f'required={command.cancel_settle_fresh_states}, '
            f'driving={snapshot.driving}, elapsed={elapsed:.1f}s',
        )
        return True

    @staticmethod
    def _is_success(result: Any) -> bool:
        return result == RobotAPIResult.SUCCESS and result is not False

    @staticmethod
    def _is_retry(result: Any) -> bool:
        return result == RobotAPIResult.RETRY and result is not True

    @staticmethod
    def _is_impossible(result: Any) -> bool:
        return result == RobotAPIResult.IMPOSSIBLE

    def _get_command_state_snapshot(
        self,
    ) -> tuple[RobotCommandState | None, bool]:
        if not callable(getattr(self._api, 'get_command_state', None)):
            return None, False
        try:
            snapshot = self._api.get_command_state(self._robot_name)
        except Exception:
            logger.exception('Command state read failed [%s]', self._robot_name)
            return None, True
        if snapshot is None:
            return None, True
        if not isinstance(snapshot, RobotCommandState):
            return None, False
        return snapshot, True

    @staticmethod
    def _snapshot_has_fatal_error(
        snapshot: RobotCommandState | None,
    ) -> bool:
        if snapshot is None:
            return False
        return any(
            enum_value(getattr(error, 'error_level', None)) == 'FATAL'
            for error in snapshot.errors
        )

    @staticmethod
    def _snapshot_is_manual_mode(
        snapshot: RobotCommandState | None,
    ) -> bool:
        return (
            snapshot is not None
            and enum_value(snapshot.operating_mode)
            in ('MANUAL', 'SERVICE', 'TEACHIN')
        )

    def _task_context_value(self) -> str:
        try:
            return self._task_context().value
        except Exception:
            logger.exception(
                'Command task context callback failed [%s]', self._robot_name
            )
            return '<unknown>'

    @staticmethod
    def _sending_state(kind: CommandKind) -> CommandState:
        if kind in (CommandKind.CANCEL_ORDER, CommandKind.CANCEL_ACTION):
            return CommandState.SENDING_CANCEL
        if kind == CommandKind.ORDER:
            return CommandState.SENDING_ORDER
        return CommandState.SENDING_ACTION

    @staticmethod
    def _waiting_state(kind: CommandKind) -> CommandState:
        if kind in (CommandKind.CANCEL_ORDER, CommandKind.CANCEL_ACTION):
            return CommandState.WAIT_CANCEL_ACK
        if kind == CommandKind.ORDER:
            return CommandState.WAIT_ORDER_ACK
        if kind == CommandKind.ACTION:
            return CommandState.WAIT_ACTION_ACK
        return CommandState.IDLE

    def _log_command_event(
        self,
        command: QueuedCommand | None,
        event: str,
        next_state: CommandState | CommandTopState | str,
        *,
        detail: str = '',
        level: int = logging.INFO,
    ) -> None:
        next_value = (
            next_state.value if isinstance(next_state, Enum) else str(next_state)
        )
        if command is None:
            values = ('<none>',) * 6 + ('',)
        else:
            values = (
                command.event_id, command.cmd_id, command.order_id,
                command.order_update_id, command.action_id,
                command.target_kind, command.description,
            )
        suffix = f', {detail}' if detail else ''
        logger.log(
            level,
            'Command %s [%s]: event_id=%s, cmd_id=%s, order_id=%s, '
            'order_update_id=%s, action_id=%s, target_kind=%s, state=%s, '
            'event=%s, next_state=%s, top_state=%s, task_context=%s, '
            'description=%s%s',
            event, self._robot_name, *values[:6], self._state.value, event,
            next_value, self._top_state.value, self._task_context_value(),
            values[6], suffix,
        )

    def _log_command_blocked(
        self, command: QueuedCommand | None, reason: str
    ) -> None:
        now = time.monotonic()
        key = (command.event_id if command else '<none>', reason)
        if key == self._last_block_log_key and now - self._last_block_log_at < 5.0:
            return
        self._last_block_log_key = key
        self._last_block_log_at = now
        self._log_command_event(
            command, 'Blocked', self._top_state,
            detail=f'reason={reason}', level=logging.WARNING,
        )

    def _refresh_top_state(self, snapshot: RobotCommandState | None) -> None:
        next_top = CommandTopState.AVAILABLE
        reason = 'available'
        if not self._api.is_robot_connected(self._robot_name):
            next_top, reason = CommandTopState.BLOCKED, 'robot_offline'
        elif (
            self._snapshot_is_manual_mode(snapshot)
            or self._api.is_manual_mode(self._robot_name)
        ):
            next_top, reason = CommandTopState.DECOMMISSIONED, 'manual_mode'
        elif self._snapshot_has_fatal_error(snapshot):
            next_top, reason = CommandTopState.FAULTED, 'fatal_error'
        if next_top == self._top_state:
            return
        previous = self._top_state
        self._top_state = next_top
        logger.info(
            'Command top transition [%s]: state=%s, '
            'event=RobotStateUpdated, next_state=%s, reason=%s',
            self._robot_name, previous.value, next_top.value, reason,
        )

    def _command_allowed_in_top_state(
        self, command: QueuedCommand | None
    ) -> bool:
        if self._top_state == CommandTopState.AVAILABLE:
            return True
        if command is None:
            return False
        return (
            command.kind in (CommandKind.CANCEL_ORDER, CommandKind.CANCEL_ACTION)
            and self._top_state
            in (CommandTopState.DECOMMISSIONED, CommandTopState.FAULTED)
        )

    def _ack_active_command(
        self, command: QueuedCommand, *, detail: str | None = None
    ) -> None:
        self._state = CommandState.IDLE
        self._log_command_event(
            command, 'Acked', CommandState.IDLE,
            detail=detail or f'ack_condition={command.ack_description}',
        )
        if command.on_acked is not None:
            command.on_acked()
        self._active_command = None
