"""VDA5050 MQTT 기반 RobotAPI 구현.

VDA5050 프로토콜을 통해 AGV와 통신하는 RobotAPI 어댑터.
MQTT로 Order/InstantActions를 발행하고, State를 구독하여
로봇 상태를 캐시한다.
"""

from __future__ import annotations

from datetime import UTC, datetime
import logging
import threading
import time
from typing import Any, Callable
import uuid

from vda5050_fleet_adapter.domain.entities.action import Action, ActionParameter
from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import (
    ActionStatus, BlockingType, ConnectionState, OperatingMode,
)
from vda5050_fleet_adapter.infra.mqtt.message_serializer import (
    deserialize_connection, deserialize_state, serialize_instant_actions,
    serialize_order,
)
from vda5050_fleet_adapter.infra.mqtt.mqtt_client import MqttClient
from vda5050_fleet_adapter.usecase.ports.robot_api import (
    CommissionState, RobotAPI, RobotAPIResult, RobotCommandState,
    RobotUpdateData,
)

logger = logging.getLogger(__name__)
_VDA5050_VERSION = '2.0.0'


class Vda5050RobotAPI(RobotAPI):
    """VDA5050 MQTT protocol implementation of the RobotAPI port."""

    def __init__(
        self,
        mqtt_client: MqttClient,
        prefix: str,
        manufacturer: str = '',
        download_map_config: dict | None = None,
    ) -> None:
        self._mqtt = mqtt_client
        self._prefix = prefix.rstrip('/')
        self._manufacturer = manufacturer
        self._lock = threading.Lock()
        self._state_cache: dict[str, Any] = {}
        self._state_received_at_cache: dict[str, float] = {}
        self._cmd_order_map: dict[str, dict[int, str]] = {}
        self._completed_cmds: dict[str, set[int]] = {}
        self._header_ids: dict[str, int] = {}
        self._connection_cache: dict[str, ConnectionState] = {}
        self._download_map_config = download_map_config
        self._download_map_pending: dict[str, str | None] = {}
        self._on_robot_offline_cb: Callable[[str], None] | None = None

    def subscribe_robot(self, robot_name: str) -> None:
        state_topic = self._build_topic(robot_name, 'state')
        self._mqtt.subscribe(
            state_topic,
            lambda _topic, payload: self._on_state_message(robot_name, payload),
            qos=1,
        )
        logger.info('Subscribed to %s', state_topic)
        connection_topic = self._build_topic(robot_name, 'connection')
        self._mqtt.subscribe(
            connection_topic,
            lambda _topic, payload: self._on_connection_message(
                robot_name, payload
            ),
            qos=1,
        )
        logger.info('Subscribed to %s', connection_topic)

    def set_on_robot_offline(self, callback: Callable[[str], None]) -> None:
        self._on_robot_offline_cb = callback

    def is_robot_connected(self, robot_name: str) -> bool:
        with self._lock:
            state = self._connection_cache.get(robot_name)
        return state is None or state == ConnectionState.ONLINE

    def is_download_map_ready(self, robot_name: str) -> bool:
        if self._download_map_config is None:
            return True
        with self._lock:
            action_id = self._download_map_pending.get(robot_name)
            state = self._state_cache.get(robot_name)
        if action_id is None:
            return False
        if action_id == '':
            return True
        if state is None:
            return False
        for action_state in state.action_states:
            if (
                action_state.action_id == action_id
                and action_state.action_status
                in (ActionStatus.FINISHED, ActionStatus.FAILED)
            ):
                with self._lock:
                    self._download_map_pending[robot_name] = ''
                logger.info(
                    'downloadMap completed: robot=%s, action_id=%s, status=%s',
                    robot_name, action_id, action_state.action_status,
                )
                return True
        return False

    def _send_download_map(self, robot_name: str) -> None:
        if self._download_map_config is None:
            return
        action_id = f'downloadMap_{uuid.uuid4().hex[:8]}'
        params = [
            ActionParameter('mapId', self._download_map_config.get('map_id', '')),
            ActionParameter(
                'mapDownloadUrl',
                self._download_map_config.get('map_download_url', ''),
            ),
            ActionParameter(
                'mapVersion', self._download_map_config.get('map_version', '')
            ),
        ]
        action = Action(
            'downloadMap', action_id, BlockingType.NONE,
            action_parameters=params,
        )
        header = self._make_header(robot_name, 'instantActions')
        self._mqtt.publish(
            self._build_topic(robot_name, 'instantActions'),
            serialize_instant_actions(header, [action]),
            qos=2,
        )
        with self._lock:
            self._download_map_pending[robot_name] = action_id
        logger.info(
            'downloadMap sent: robot=%s, action_id=%s', robot_name, action_id
        )

    def connect(self) -> None:
        self._mqtt.connect()

    def disconnect(self) -> None:
        self._mqtt.disconnect()

    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        nodes: list[Node],
        edges: list[Edge],
        map_name: str,
        order_id: str = '',
        order_update_id: int = 0,
        *,
        track_action_id: str | None = None,
    ) -> RobotAPIResult:
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry navigate')
            return RobotAPIResult.RETRY
        if not self.is_robot_connected(robot_name):
            logger.warning(
                'Robot %s not connected, will retry navigate', robot_name
            )
            return RobotAPIResult.RETRY
        if not self.is_download_map_ready(robot_name):
            logger.warning(
                'Robot %s downloadMap not ready, will retry navigate',
                robot_name,
            )
            return RobotAPIResult.RETRY
        if not order_id:
            order_id = f'order_{cmd_id}_{uuid.uuid4().hex[:8]}'
        order = Order(
            header=self._make_header(robot_name, 'order'),
            order_id=order_id,
            order_update_id=order_update_id,
            nodes=nodes,
            edges=edges,
        )
        self._mqtt.publish(
            self._build_topic(robot_name, 'order'),
            serialize_order(order),
            qos=2,
        )
        track_id = track_action_id or order_id
        with self._lock:
            self._cmd_order_map.setdefault(robot_name, {})[cmd_id] = track_id
            self._completed_cmds.setdefault(robot_name, set())
        logger.info(
            'Navigate order sent: robot=%s, cmd_id=%d, order_id=%s, '
            'track_id=%s, nodes=%d, edges=%d',
            robot_name, cmd_id, order_id, track_id, len(nodes), len(edges),
        )
        return RobotAPIResult.SUCCESS

    def stop(self, robot_name: str, cmd_id: int) -> RobotAPIResult:
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry stop')
            return RobotAPIResult.RETRY
        if not self.is_robot_connected(robot_name):
            logger.warning('Robot %s not connected, will retry stop', robot_name)
            return RobotAPIResult.RETRY
        action = Action(
            'cancelOrder', f'cancel_{cmd_id}_{uuid.uuid4().hex[:8]}',
            BlockingType.HARD,
        )
        self._mqtt.publish(
            self._build_topic(robot_name, 'instantActions'),
            serialize_instant_actions(
                self._make_header(robot_name, 'instantActions'), [action]
            ),
            qos=2,
        )
        logger.info('Stop (cancelOrder) sent: robot=%s', robot_name)
        return RobotAPIResult.SUCCESS

    def pause(self, robot_name: str, cmd_id: int) -> RobotAPIResult:
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry pause')
            return RobotAPIResult.RETRY
        action = Action(
            'startPause', f'pause_{cmd_id}_{uuid.uuid4().hex[:8]}',
            BlockingType.HARD,
        )
        self._mqtt.publish(
            self._build_topic(robot_name, 'instantActions'),
            serialize_instant_actions(
                self._make_header(robot_name, 'instantActions'), [action]
            ),
            qos=2,
        )
        logger.info('Pause (startPause) sent: robot=%s', robot_name)
        return RobotAPIResult.SUCCESS

    def start_activity(
        self,
        robot_name: str,
        cmd_id: int,
        activity: str,
        action_params: dict,
        *,
        action_id: str | None = None,
    ) -> RobotAPIResult:
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry activity')
            return RobotAPIResult.RETRY
        if not self.is_robot_connected(robot_name):
            logger.warning(
                'Robot %s not connected, will retry activity', robot_name
            )
            return RobotAPIResult.RETRY
        resolved_action_id = (
            action_id or f'{activity}_{cmd_id}_{uuid.uuid4().hex[:8]}'
        )
        action = Action(
            activity, resolved_action_id, BlockingType.HARD,
            action_parameters=[
                ActionParameter(key, value)
                for key, value in action_params.items()
            ],
        )
        self._mqtt.publish(
            self._build_topic(robot_name, 'instantActions'),
            serialize_instant_actions(
                self._make_header(robot_name, 'instantActions'), [action]
            ),
            qos=2,
        )
        with self._lock:
            self._cmd_order_map.setdefault(robot_name, {})[cmd_id] = action.action_id
        logger.info(
            'Activity sent: robot=%s, activity=%s, cmd_id=%d',
            robot_name, activity, cmd_id,
        )
        return RobotAPIResult.SUCCESS

    def get_data(self, robot_name: str) -> RobotUpdateData | None:
        with self._lock:
            state = self._state_cache.get(robot_name)
        if state is None:
            return None
        position = [0.0, 0.0, 0.0]
        map_name = ''
        if state.agv_position is not None:
            position = [
                state.agv_position.x,
                state.agv_position.y,
                state.agv_position.theta,
            ]
            map_name = state.agv_position.map_id
        battery_soc = 0.0
        charging = False
        if state.battery_state is not None:
            battery_soc = state.battery_state.battery_charge / 100.0
            charging = state.battery_state.charging
        return RobotUpdateData(
            robot_name=robot_name, map_name=map_name, position=position,
            battery_soc=battery_soc, last_node_id=state.last_node_id,
            driving=state.driving, charging=charging,
        )

    def get_command_state(self, robot_name: str) -> RobotCommandState | None:
        with self._lock:
            state = self._state_cache.get(robot_name)
            received_at = self._state_received_at_cache.get(robot_name)
            connection = self._connection_cache.get(robot_name)
        if state is None or received_at is None:
            return None
        return RobotCommandState(
            state_received_at=received_at, order_id=state.order_id,
            order_update_id=state.order_update_id,
            last_node_id=state.last_node_id,
            last_node_sequence_id=state.last_node_sequence_id,
            driving=state.driving, paused=state.paused,
            new_base_request=state.new_base_request,
            node_states=list(state.node_states), edge_states=list(state.edge_states),
            action_states=list(state.action_states), errors=list(state.errors),
            information=list(state.information),
            operating_mode=state.operating_mode,
            connection_state=connection,
        )

    def get_battery_soc(self, robot_name: str) -> float | None:
        with self._lock:
            state = self._state_cache.get(robot_name)
        if state is None or state.battery_state is None:
            return None
        return state.battery_state.battery_charge / 100.0

    def get_commission_state(
        self, robot_name: str
    ) -> CommissionState | None:
        with self._lock:
            state = self._state_cache.get(robot_name)
            connection = self._connection_cache.get(robot_name)
        if state is None:
            return None
        if connection in (
            ConnectionState.OFFLINE, ConnectionState.CONNECTIONBROKEN
        ):
            return CommissionState(False, False, False)
        if state.has_fatal_error or state.is_emergency_stopped:
            return CommissionState(False, False, False)
        if state.operating_mode in (
            OperatingMode.MANUAL, OperatingMode.SERVICE, OperatingMode.TEACHIN
        ):
            return CommissionState(False, False, False)
        if state.operating_mode == OperatingMode.SEMIAUTOMATIC:
            return CommissionState(False, True, True)
        return CommissionState(True, True, True)

    def is_manual_mode(self, robot_name: str) -> bool:
        with self._lock:
            state = self._state_cache.get(robot_name)
        if state is None:
            return False
        return state.operating_mode in (
            OperatingMode.MANUAL, OperatingMode.SERVICE, OperatingMode.TEACHIN
        )

    def get_robot_state_summary(self, robot_name: str) -> dict | None:
        with self._lock:
            state = self._state_cache.get(robot_name)
            connection = self._connection_cache.get(robot_name)
        if state is None:
            return None
        return {
            'robot': robot_name,
            'operating_mode': state.operating_mode.value,
            'has_fatal_error': state.has_fatal_error,
            'is_emergency_stopped': state.is_emergency_stopped,
            'connection_state': (
                connection.value if connection else 'OFFLINE'
            ),
        }

    def is_command_completed(self, robot_name: str, cmd_id: int) -> bool:
        with self._lock:
            if cmd_id in self._completed_cmds.get(robot_name, set()):
                return True
            state = self._state_cache.get(robot_name)
            tracked_id = self._cmd_order_map.get(robot_name, {}).get(cmd_id)
        if state is None or tracked_id is None or state.has_fatal_error:
            return False
        if tracked_id.startswith('order_') or tracked_id == state.order_id:
            if state.order_id == tracked_id:
                has_released = any(item.released for item in state.node_states)
                if not has_released and not state.driving:
                    with self._lock:
                        self._completed_cmds.setdefault(robot_name, set()).add(
                            cmd_id
                        )
                    return True
            elif not state.order_id:
                with self._lock:
                    self._completed_cmds.setdefault(robot_name, set()).add(cmd_id)
                return True
        else:
            for action_state in state.action_states:
                if (
                    action_state.action_id == tracked_id
                    and action_state.action_status == ActionStatus.FINISHED
                ):
                    with self._lock:
                        self._completed_cmds.setdefault(robot_name, set()).add(
                            cmd_id
                        )
                    return True
        return False

    def _on_state_message(self, robot_name: str, payload: bytes) -> None:
        try:
            state = deserialize_state(payload.decode('utf-8'))
            with self._lock:
                self._state_cache[robot_name] = state
                self._state_received_at_cache[robot_name] = time.monotonic()
        except Exception:
            logger.exception('Failed to deserialize state: robot=%s', robot_name)

    def _on_connection_message(self, robot_name: str, payload: bytes) -> None:
        try:
            connection = deserialize_connection(payload.decode('utf-8'))
            new_state = connection.connection_state
            with self._lock:
                previous = self._connection_cache.get(robot_name)
                self._connection_cache[robot_name] = new_state
            logger.info(
                'Connection update: robot=%s, state=%s', robot_name, new_state
            )
            if (
                new_state in (
                    ConnectionState.OFFLINE, ConnectionState.CONNECTIONBROKEN
                )
                and previous == ConnectionState.ONLINE
                and self._on_robot_offline_cb is not None
            ):
                self._on_robot_offline_cb(robot_name)
            if new_state == ConnectionState.ONLINE and previous != new_state:
                with self._lock:
                    self._state_cache.pop(robot_name, None)
                    self._state_received_at_cache.pop(robot_name, None)
                logger.info(
                    'Cleared stale state cache on reconnect: robot=%s', robot_name
                )
                self._send_download_map(robot_name)
        except Exception:
            logger.exception(
                'Failed to deserialize connection: robot=%s', robot_name
            )

    def _build_topic(self, robot_name: str, topic_name: str) -> str:
        return f'{self._prefix}/{robot_name}/{topic_name}'

    def _make_header(self, robot_name: str, topic: str) -> Header:
        key = f'{robot_name}/{topic}'
        with self._lock:
            header_id = self._header_ids.get(key, 0)
            self._header_ids[key] = header_id + 1
        return Header(
            version=_VDA5050_VERSION, manufacturer=self._manufacturer,
            serial_number=robot_name, header_id=header_id,
            timestamp=datetime.now(UTC),
        )
