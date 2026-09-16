"""MQTT 클라이언트 - VDA5050 토픽 구독/발행 관리."""

from __future__ import annotations

import json
import logging
from typing import Callable

import paho.mqtt.client as mqtt

from .models import _to_dict, _timestamp, ConnectionMessage

logger = logging.getLogger(__name__)


class MqttClient:
    def __init__(self, config: dict):
        mqtt_cfg = config["mqtt"]
        robot_cfg = config["robot"]

        self._broker_host = mqtt_cfg["broker_host"]
        self._broker_port = mqtt_cfg["broker_port"]
        self._manufacturer = robot_cfg["manufacturer"]
        self._serial_number = robot_cfg["serial_number"]
        self._interface = robot_cfg["interface_name"]
        self._version = robot_cfg["protocol_version"]

        self._topic_prefix = (
            f"{self._interface}/{self._version}/"
            f"{self._manufacturer}/{self._serial_number}"
        )

        self._header_ids: dict[str, int] = {}
        self._on_order: Callable | None = None
        self._on_instant_actions: Callable | None = None
        self._on_connection_online: Callable | None = None

        if hasattr(mqtt, "CallbackAPIVersion"):
            # paho-mqtt >= 2.0
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=f"vda5050_sim_{self._serial_number}",
            )
        else:
            # paho-mqtt < 2.0
            self._client = mqtt.Client(
                client_id=f"vda5050_sim_{self._serial_number}",
            )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect

        # Last Will: CONNECTIONBROKEN
        last_will = ConnectionMessage(
            headerId=0,
            timestamp=_timestamp(),
            version="2.0.0",
            manufacturer=self._manufacturer,
            serialNumber=self._serial_number,
            connectionState="CONNECTIONBROKEN",
        )
        self._client.will_set(
            topic=f"{self._topic_prefix}/connection",
            payload=json.dumps(_to_dict(last_will)),
            qos=1,
            retain=True,
        )

    def set_callbacks(
        self,
        on_order: Callable | None = None,
        on_instant_actions: Callable | None = None,
        on_connection_online: Callable | None = None,
    ):
        self._on_order = on_order
        self._on_instant_actions = on_instant_actions
        self._on_connection_online = on_connection_online

    def connect(self):
        self._log_prefix = f"[{self._serial_number}]"
        logger.info(
            "%s MQTT 연결 시도: %s:%d", self._log_prefix, self._broker_host, self._broker_port
        )
        self._client.connect(self._broker_host, self._broker_port)
        self._client.loop_start()

    def disconnect(self):
        self.publish_connection("OFFLINE")
        self._client.loop_stop()
        self._client.disconnect()
        logger.info("%s MQTT 연결 종료", self._log_prefix)

    def _on_connect(self, client, userdata, flags, rc, *args):
        if rc == 0 or (hasattr(rc, "value") and rc.value == 0):
            logger.info("%s MQTT 연결 성공", self._log_prefix)
            # 토픽 구독
            order_topic = f"{self._topic_prefix}/order"
            ia_topic = f"{self._topic_prefix}/instantActions"
            client.subscribe(order_topic, qos=0)
            client.subscribe(ia_topic, qos=0)
            logger.info("%s 구독: %s, %s", self._log_prefix, order_topic, ia_topic)
            # ONLINE 발행
            self.publish_connection("ONLINE")
            if self._on_connection_online:
                self._on_connection_online()
        else:
            logger.error("%s MQTT 연결 실패: rc=%s", self._log_prefix, rc)

    def _on_disconnect(self, client, userdata, flags_or_rc, rc=None, properties=None):
        # paho v1: (client, userdata, rc) — rc lands in flags_or_rc
        # paho v2: (client, userdata, flags, rc, properties)
        if rc is None:
            rc = flags_or_rc
        if rc != 0 and not (hasattr(rc, "value") and rc.value == 0):
            logger.warning("%s MQTT 비정상 연결 해제: rc=%s", self._log_prefix, rc)

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            logger.error("%s 메시지 파싱 실패 [%s]: %s", self._log_prefix, topic, e)
            return

        if topic.endswith("/order"):
            logger.info("%s Order 수신: orderId=%s", self._log_prefix, payload.get("orderId", "?"))
            if self._on_order:
                self._on_order(payload)
        elif topic.endswith("/instantActions"):
            logger.info("%s InstantActions 수신", self._log_prefix)
            if self._on_instant_actions:
                self._on_instant_actions(payload)
        else:
            logger.debug("알 수 없는 토픽: %s", topic)

    def _next_header_id(self, topic: str) -> int:
        self._header_ids[topic] = self._header_ids.get(topic, 0) + 1
        return self._header_ids[topic]

    def publish_connection(self, state: str):
        topic = f"{self._topic_prefix}/connection"
        msg = ConnectionMessage(
            headerId=self._next_header_id("connection"),
            timestamp=_timestamp(),
            version="2.0.0",
            manufacturer=self._manufacturer,
            serialNumber=self._serial_number,
            connectionState=state,
        )
        self._client.publish(
            topic, json.dumps(_to_dict(msg)), qos=1, retain=True
        )
        logger.info("%s Connection 발행: %s", self._log_prefix, state)

    def publish_state(self, state_dict: dict):
        topic = f"{self._topic_prefix}/state"
        state_dict["headerId"] = self._next_header_id("state")
        state_dict["timestamp"] = _timestamp()
        self._client.publish(topic, json.dumps(state_dict, ensure_ascii=False))

    def publish_visualization(self, vis_dict: dict):
        topic = f"{self._topic_prefix}/visualization"
        vis_dict["headerId"] = self._next_header_id("visualization")
        vis_dict["timestamp"] = _timestamp()
        self._client.publish(topic, json.dumps(vis_dict, ensure_ascii=False))
