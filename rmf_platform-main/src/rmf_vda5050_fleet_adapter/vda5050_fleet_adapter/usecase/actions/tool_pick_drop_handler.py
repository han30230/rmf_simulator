"""TOOL(화물) 로봇의 pick/drop action handler."""

from __future__ import annotations

import logging
from typing import Any, Callable

from vda5050_fleet_adapter.usecase.actions.action_handler import (
    ActionHandler,
    ActionHandleResult,
    PathModifyResult,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import RobotAPI

logger = logging.getLogger(__name__)


class ToolPickDropHandler(ActionHandler):
    """TOOL(화물) pick/drop 처리."""

    def __init__(self) -> None:
        self._destination: str | None = None
        self._station_node: str | None = None
        self._execution: Any = None
        self._cmd_id: int | None = None

    @property
    def destination(self) -> str | None:
        return self._destination

    @property
    def station_node(self) -> str | None:
        return self._station_node

    def modify_navigate_path(
        self,
        path: list[str],
        dest_name: str,
        nav_nodes: dict,
        dest_attrs: dict,
    ) -> PathModifyResult:
        dest_is_pick_drop = dest_attrs.get('pickDrop', False)
        if dest_is_pick_drop and dest_name:
            self._destination = dest_name
            self._station_node = None
            return PathModifyResult(path=path)
        self._destination = None
        self._station_node = None
        return PathModifyResult(path=path)

    def should_skip_navigate(
        self,
        path: list[str],
        current_node: str | None = None,
    ) -> bool:
        return (
            self._destination is not None
            and current_node == self._destination
            and len(path) == 1
            and path[0] == self._destination
        )

    def handle_execute_action(
        self,
        category: str,
        description: dict,
        execution: Any,
        api: RobotAPI,
        robot_name: str,
        cmd_id: int,
        enqueue_activity: (
            Callable[[int, str, dict, str], str] | None
        ) = None,
        enqueue_action_order: (
            Callable[[int, str, dict, str], str] | None
        ) = None,
        current_node: str | None = None,
        final_destination: str | None = None,
    ) -> ActionHandleResult | None:
        if category not in ('pick', 'drop'):
            return None
        if self._destination is None:
            return None

        action_params = dict(description) if description else {}
        station_name = action_params.get('stationName')
        self._station_node = station_name if station_name else None

        if enqueue_activity is not None:
            action_id = enqueue_activity(
                cmd_id, category, action_params,
                f'{category} pickDrop instantAction',
            )
            send_mode = 'queued'
        else:
            api.start_activity(robot_name, cmd_id, category, action_params)
            action_id = '<api-generated>'
            send_mode = 'sent'

        self._execution = execution
        self._cmd_id = cmd_id
        logger.info(
            '%s instantAction %s [%s]: cmd_id=%d, '
            'action_id=%s, pick_drop_dest=%s, station_node=%s, params=%s',
            category, send_mode, robot_name, cmd_id, action_id,
            self._destination, self._station_node, action_params,
        )
        return ActionHandleResult(handled=True)

    def check_action_states(
        self,
        api: RobotAPI,
        robot_name: str,
        is_commissioned: bool = True,
    ) -> None:
        if self._cmd_id is None:
            return
        completed = api.is_command_completed(robot_name, self._cmd_id)
        if completed:
            if self._execution is not None:
                logger.info(
                    'pickDrop action FINISHED for %s, '
                    'calling execution.finished()', robot_name,
                )
                self._execution.finished()
                self._execution = None
            self._cmd_id = None
            self._destination = None
            self._station_node = None

    def reset(self) -> None:
        self._destination = None
        self._station_node = None
        if self._execution is not None:
            self._execution = None
        self._cmd_id = None
