"""RECONSTRUCTED baseline RMF-to-VDA5050 RobotAdapter.

The original company RobotAdapter was only partially supplied. This module
keeps the observed public callback contract and baseline completion semantics;
advanced charging, persistence, corridor and dynamic-mutex behavior is not
claimed to be restored here.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
from typing import Any
import uuid

from vda5050_fleet_adapter.usecase.actions.action_handler import ActionHandler
from vda5050_fleet_adapter.usecase.command_ack_hsm import (
    CommandAckHsm, CommandKind, QueuedCommand, TaskContext, order_ack_guard,
)
from vda5050_fleet_adapter.usecase.graph_utils import (
    DirectedGraph, build_vda5050_nodes_edges, compute_path, find_nearest_node,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import (
    RobotAPI, RobotAPIResult, RobotUpdateData,
)

logger = logging.getLogger(__name__)


@dataclass
class NavigationState:
    is_navigating: bool = False
    target_node: str | None = None
    target_position: list[float] | None = None
    cmd_id: int | None = None
    order_id: str | None = None


class RobotAdapter:
    """Minimum compatible EasyFullControl robot callback implementation."""

    def __init__(
        self,
        name: str,
        configuration: Any = None,
        node: Any = None,
        api: RobotAPI | None = None,
        fleet_handle: Any = None,
        nav_nodes: dict[str, dict] | None = None,
        nav_edges: dict[str, dict] | None = None,
        nav_graph: DirectedGraph | None = None,
        *,
        arrival_threshold: float = 0.5,
        action_handler: ActionHandler | None = None,
    ) -> None:
        if api is None:
            raise ValueError('RobotAdapter requires a RobotAPI')
        self.name = name
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.nav_nodes = nav_nodes or {}
        self.nav_edges = nav_edges or {}
        self.nav_graph = nav_graph or DirectedGraph()
        self.arrival_threshold = float(arrival_threshold)
        self._action_handler = action_handler
        self._execution_lock = threading.RLock()
        self.execution: Any | None = None
        self.update_handle: Any | None = None
        self.position: list[float] | None = None
        self.last_node_id = ''
        self.cmd_id = 0
        self._nav = NavigationState()
        self._action_cmd_id: int | None = None
        self._command_hsm = CommandAckHsm(
            name, api, self._task_context
        )

    def _task_context(self) -> TaskContext:
        if self._action_cmd_id is not None:
            return TaskContext.CHARGING_FLOW
        return TaskContext.DISPATCHED_TASK

    def update(self, state: Any, data: RobotUpdateData) -> None:
        self.position = list(data.position)
        self.last_node_id = data.last_node_id
        self._command_hsm.pump()

        with self._execution_lock:
            execution = self.execution
            if execution is not None and self._nav.is_navigating:
                target = self._nav.target_position
                distance = math.inf
                if target is not None:
                    distance = math.hypot(
                        data.position[0] - target[0],
                        data.position[1] - target[1],
                    )
                if (
                    not data.driving
                    and data.last_node_id == self._nav.target_node
                    and distance <= self.arrival_threshold
                ):
                    execution.finished()
                    self.execution = None
                    self._nav = NavigationState()
            elif (
                execution is not None
                and self._action_cmd_id is not None
                and self.api.is_command_completed(
                    self.name, self._action_cmd_id
                )
            ):
                execution.finished()
                self.execution = None
                self._action_cmd_id = None

        if self._action_handler is not None:
            self._action_handler.check_action_states(self.api, self.name, True)

        if self.update_handle is not None and state is not None:
            activity = self.execution.identifier if self.execution else None
            self.update_handle.update(state, activity)

    def navigate(self, destination: Any, execution: Any) -> None:
        goal_name = getattr(destination, 'name', '')
        if not goal_name or goal_name not in self.nav_nodes:
            goal_name = find_nearest_node(
                self.nav_nodes,
                float(destination.position[0]),
                float(destination.position[1]),
            )
        if goal_name is None:
            raise ValueError('cannot resolve RMF destination to graph node')

        start_name = self.last_node_id
        if start_name not in self.nav_nodes and self.position is not None:
            start_name = find_nearest_node(
                self.nav_nodes, self.position[0], self.position[1]
            ) or ''
        if start_name not in self.nav_nodes:
            start_name = goal_name

        waypoint_names = getattr(destination, 'waypoint_names', None)
        if (
            isinstance(waypoint_names, (list, tuple))
            and waypoint_names
            and all(name in self.nav_nodes for name in waypoint_names)
        ):
            path = list(waypoint_names)
            if start_name in path:
                path = path[path.index(start_name):]
            elif path[0] != start_name:
                bridge = compute_path(self.nav_graph, start_name, path[0])
                path = (bridge[:-1] + path) if bridge else [start_name] + path
        else:
            path = compute_path(self.nav_graph, start_name, goal_name)
        if not path:
            raise ValueError(f'no graph path from {start_name} to {goal_name}')

        self.cmd_id += 1
        cmd_id = self.cmd_id
        order_id = f'order_{cmd_id}_{uuid.uuid4().hex[:8]}'
        map_name = str(getattr(destination, 'map', '') or 'L1')
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            path, self.nav_nodes, map_name,
            base_end_index=len(path) - 1,
            edges=self.nav_edges,
        )
        self.execution = execution
        self._nav = NavigationState(
            is_navigating=True,
            target_node=path[-1],
            target_position=[
                self.nav_nodes[path[-1]]['x'], self.nav_nodes[path[-1]]['y']
            ],
            cmd_id=cmd_id,
            order_id=order_id,
        )
        command = QueuedCommand(
            event_id=self._command_hsm.next_event_id(),
            kind=CommandKind.ORDER,
            cmd_id=cmd_id,
            send=lambda: self.api.navigate(
                self.name, cmd_id, vda_nodes, vda_edges, map_name,
                order_id=order_id, order_update_id=0,
            ),
            description='navigate order',
            order_id=order_id,
            order_update_id=0,
            ack_guard=order_ack_guard(order_id, 0),
            ack_description='state reports order id/update id',
        )
        self._command_hsm.enqueue(command)

    def stop(self, activity: Any) -> None:
        with self._execution_lock:
            if self.execution is None:
                return
            if not self.execution.identifier.is_same(activity):
                return
            self.execution = None
            self._nav = NavigationState()
        self._command_hsm.cancel_pending()
        self.cmd_id += 1
        self.api.pause(self.name, self.cmd_id)

    def execute_action(
        self, category: str, description: dict, execution: Any
    ) -> None:
        params = description if isinstance(description, dict) else {}
        next_cmd_id = self.cmd_id + 1
        if self._action_handler is not None:
            result = self._action_handler.handle_execute_action(
                category, params, execution, self.api, self.name, next_cmd_id,
                lambda cmd_id, activity, action_params, _description: (
                    self._enqueue_activity(cmd_id, activity, action_params)
                ),
                None,
                self.last_node_id or None,
                self._nav.target_node,
            )
            if result is not None:
                self.cmd_id = next_cmd_id
                return
        self.cmd_id = next_cmd_id
        self.execution = execution
        self._action_cmd_id = self.cmd_id
        self.api.start_activity(
            self.name, self.cmd_id, category, params,
            action_id=f'{category}_{self.cmd_id}_{uuid.uuid4().hex[:8]}',
        )

    def _enqueue_activity(
        self, cmd_id: int, activity: str, params: dict
    ) -> str:
        action_id = f'{activity}_{cmd_id}_{uuid.uuid4().hex[:8]}'
        self.api.start_activity(
            self.name, cmd_id, activity, params, action_id=action_id
        )
        return action_id

    def make_callbacks(self) -> Any:
        import rmf_adapter.easy_full_control as rmf_easy
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )

    def add_to_fleet(self, state: Any) -> Any:
        if self.fleet_handle is None:
            raise RuntimeError('fleet_handle is not configured')
        return self.fleet_handle.add_robot(
            self.name, state, self.configuration, self.make_callbacks()
        )
