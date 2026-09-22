"""State/Visualization 메시지 생성 및 주기적 발행."""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import asdict
from typing import TYPE_CHECKING

from .models import _to_dict, _timestamp
from .fault_injection import FaultInjectionController, apply_overrides

if TYPE_CHECKING:
    from .robot import Robot
    from .mqtt_client import MqttClient

logger = logging.getLogger(__name__)


def _clean_none(d):
    """dict에서 None 값을 재귀적으로 제거."""
    if isinstance(d, dict):
        return {k: _clean_none(v) for k, v in d.items() if v is not None}
    if isinstance(d, list):
        return [_clean_none(item) for item in d]
    return d


class StatePublisher:
    def __init__(self, robot: Robot, mqtt_client: MqttClient, config: dict):
        self._robot = robot
        self._mqtt = mqtt_client
        pub_cfg = config["publishing"]
        robot_cfg = config["robot"]

        self._state_interval = pub_cfg["state_interval"]
        self._vis_interval = pub_cfg["visualization_interval"]
        self._manufacturer = robot_cfg["manufacturer"]
        self._serial_number = robot_cfg["serial_number"]
        self._faults = FaultInjectionController.from_config(
            config.get("fault_injection", [])
        )
        self._started_at = time.monotonic()

        self._state_changed = asyncio.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        robot.set_state_change_callback(self._on_state_change)

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def _on_state_change(self):
        """상태 변경 통지. asyncio 스레드 또는 다른 스레드에서 호출 가능."""
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._state_changed.set)
        else:
            self._state_changed.set()

    def _build_state_dict(self) -> dict:
        """로봇 상태를 State 메시지 dict로 조립."""
        r = self._robot
        state = {
            "headerId": 0,  # mqtt_client에서 덮어씀
            "timestamp": "",  # mqtt_client에서 덮어씀
            "version": "2.0.0",
            "manufacturer": self._manufacturer,
            "serialNumber": self._serial_number,
            "orderId": r.order_id,
            "orderUpdateId": r.order_update_id,
            "lastNodeId": r.last_node_id,
            "lastNodeSequenceId": r.last_node_sequence_id,
            "driving": r.driving,
            "newBaseRequest": r.new_base_request,
            "distanceSinceLastNode": round(r.distance_since_last_node, 3),
            "operatingMode": r.operating_mode,
            "paused": r.paused,
            "nodeStates": [_to_dict(ns) for ns in r.node_states],
            "edgeStates": [_to_dict(es) for es in r.edge_states],
            "actionStates": [_to_dict(a) for a in r.action_states],
            "agvPosition": _to_dict(r.get_agv_position()),
            "velocity": _to_dict(r.get_velocity()),
            "batteryState": _to_dict(r.get_battery_state()),
            "safetyState": _to_dict(r.safety_state),
            "loads": [_to_dict(l) for l in r.loads],
            "errors": [_to_dict(e) for e in r.errors],
            "information": [_to_dict(i) for i in r.information],
            "maps": [_to_dict(m) for m in r.maps],
        }
        return _clean_none(state)

    def _build_visualization_dict(self) -> dict:
        """Visualization 메시지 dict (위치 데이터 중심)."""
        r = self._robot
        vis = {
            "headerId": 0,
            "timestamp": "",
            "version": "2.0.0",
            "manufacturer": self._manufacturer,
            "serialNumber": self._serial_number,
            "orderId": r.order_id,
            "orderUpdateId": r.order_update_id,
            "lastNodeId": r.last_node_id,
            "lastNodeSequenceId": r.last_node_sequence_id,
            "driving": r.driving,
            "paused": r.paused,
            "agvPosition": _to_dict(r.get_agv_position()),
            "velocity": _to_dict(r.get_velocity()),
        }
        return _clean_none(vis)

    def publish_state_now(self):
        """즉시 State 발행."""
        effects = self._faults.apply(
            self._robot, elapsed=time.monotonic() - self._started_at
        )
        for connection_state in effects.connection_states:
            self._mqtt.publish_connection(connection_state)
            logger.warning(
                "[%s] fault injection connection=%s",
                self._serial_number,
                connection_state,
            )
        if effects.suppress_state:
            if effects.triggered:
                logger.warning(
                    "[%s] fault injection suppressing state",
                    self._serial_number,
                )
            return
        state_dict = self._build_state_dict()
        apply_overrides(state_dict, effects.state_overrides)
        self._mqtt.publish_state(state_dict)

    async def state_publish_loop(self):
        """State 발행 루프: 고정 주기로 발행."""
        while True:
            await asyncio.sleep(self._state_interval)
            self._state_changed.clear()
            self.publish_state_now()

    async def visualization_publish_loop(self):
        """Visualization 발행 루프."""
        while True:
            await asyncio.sleep(self._vis_interval)
            vis_dict = self._build_visualization_dict()
            self._mqtt.publish_visualization(vis_dict)
