"""VDA5050 state tracking and geometry-based corridor occupancy detection."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import json
import logging
import os
import threading
import time
from typing import Any, Mapping

from .corridor_registry import CorridorRegistry
from .deployment import MqttDeploymentConfig
from .direction_arbiter import DirectionArbiter
from .eligibility import EligibilityResult, RobotEligibilityPolicy

logger = logging.getLogger(__name__)


@dataclass
class RobotTelemetry:
    robot_id: str
    received_at: float
    x: float | None = None
    y: float | None = None
    last_node_id: str = ""
    driving: bool = False
    node_states: list[dict[str, Any]] = field(default_factory=list)
    edge_states: list[dict[str, Any]] = field(default_factory=list)
    current_block: str | None = None
    current_hb: str | None = None
    faulted: bool = False
    state_header_id: int | None = None
    state_timestamp: str = ""
    connection_received_at: float | None = None
    connection_header_id: int | None = None
    connection_timestamp: str = ""
    connection_state: str = ""
    manufacturer: str = ""
    serial_number: str = ""
    position_initialized: bool = False
    map_id: str = ""
    operating_mode: str = ""
    e_stop: str = ""
    field_violation: bool = False
    paused: bool = False
    error_levels: tuple[str, ...] = ()


class RobotTracker:
    def __init__(
        self,
        registry: CorridorRegistry,
        arbiter: DirectionArbiter,
        *,
        telemetry_timeout: float = 5.0,
        eligibility_policy: RobotEligibilityPolicy | None = None,
    ) -> None:
        self.registry = registry
        self.arbiter = arbiter
        self.telemetry_timeout = float(telemetry_timeout)
        self.eligibility_policy = eligibility_policy
        self._robots: dict[str, RobotTelemetry] = {}
        self._lock = threading.RLock()

    def ingest_state(
        self,
        robot_id: str,
        payload: dict[str, Any],
        *,
        received_at: float | None = None,
    ) -> None:
        now = time.monotonic() if received_at is None else float(received_at)
        with self._lock:
            previous = self._robots.get(robot_id)
            if not self._identity_matches(robot_id, payload):
                logger.error("[TRAFFIC] rejected state identity robot=%s", robot_id)
                return
            header_id = self._header_id(payload)
            timestamp = str(payload.get("timestamp") or "")
            if previous is not None and not self._is_newer(
                header_id,
                timestamp,
                previous.state_header_id,
                previous.state_timestamp,
            ):
                logger.warning("[TRAFFIC] ignored out-of-order state robot=%s", robot_id)
                return
            old_block = previous.current_block if previous else None
            last_node_id = str(payload.get("lastNodeId") or "")
            release = self.arbiter.release_node_for_robot(robot_id)
            release_matches = release is not None and last_node_id == release[1]
            position = payload.get("agvPosition")
            x: float | None = None
            y: float | None = None
            block_id = old_block
            hb_id = previous.current_hb if previous else None
            edge_states = list(payload.get("edgeStates") or [])

            if isinstance(position, dict) and position.get("x") is not None and position.get("y") is not None:
                x = float(position["x"])
                y = float(position["y"])
                matching_blocks = self.registry.blocks_for_position(x, y)
                granted_blocks = self.arbiter.granted_blocks_for_robot(robot_id)
                granted_block = next(
                    (item for item in granted_blocks if item in matching_blocks),
                    None,
                )
                authority = self.arbiter.authority_for_robot(robot_id)
                destination_hb = self.arbiter.destination_holding_bay_for_robot(
                    robot_id
                )
                source_hb = self.arbiter.source_holding_bay_for_robot(robot_id)
                hb_id = self.registry.holding_bay_for_position(x, y)
                if release_matches:
                    observed_block = None
                    hb_id = None
                elif hb_id is not None and (
                    not granted_blocks
                    or hb_id == destination_hb
                    or hb_id == source_hb
                ):
                    # A grant can precede departure from its source bay.
                    # Existing occupancy is still retained below unless this
                    # is the destination; returning to source is not an exit.
                    observed_block = None
                elif old_block in matching_blocks:
                    observed_block = old_block
                elif granted_block in matching_blocks:
                    observed_block = granted_block
                else:
                    observed_block = matching_blocks[0] if matching_blocks else None

                if observed_block is not None and observed_block != old_block:
                    if (
                        old_block is not None
                        and authority is not None
                        and old_block in authority.unreleased_blocks
                    ):
                        self.arbiter.mark_cleared(robot_id, old_block)
                    try:
                        self.arbiter.mark_entered(robot_id, observed_block)
                    except ValueError:
                        self.arbiter.report_unexpected_occupancy(
                            robot_id,
                            observed_block,
                            reason="unreserved_robot_detected_inside",
                        )
                elif (
                    old_block is not None
                    and observed_block is None
                    and hb_id == destination_hb
                ):
                    if authority is not None:
                        self.arbiter.mark_authority_arrived(robot_id)
                    else:
                        self.arbiter.mark_exited(robot_id, old_block)

                if observed_block is not None:
                    block_id = observed_block
                elif old_block is not None and hb_id != destination_hb:
                    # No safe exit evidence: retain unresolved occupancy fail-closed.
                    block_id = old_block
                else:
                    block_id = None

                if hb_id is not None and block_id is None:
                    self.arbiter.occupy_holding_bay(hb_id, robot_id)
            elif bool(payload.get("driving", False)) and not release_matches:
                observed_block = self.registry.block_for_edges(edge_states)
                if observed_block is not None and observed_block != old_block:
                    try:
                        self.arbiter.mark_entered(robot_id, observed_block)
                    except ValueError:
                        self.arbiter.report_unexpected_occupancy(
                            robot_id,
                            observed_block,
                            reason="unreserved_edge_state_inside",
                        )
                    block_id = observed_block
                    hb_id = None

            if release_matches:
                release_block, _ = release
                self.arbiter.mark_cleared(robot_id, release_block)
                block_id = None
                hb_id = None

            telemetry = RobotTelemetry(
                robot_id=robot_id,
                received_at=now,
                x=x if x is not None else (previous.x if previous else None),
                y=y if y is not None else (previous.y if previous else None),
                last_node_id=last_node_id,
                driving=bool(payload.get("driving", False)),
                node_states=list(payload.get("nodeStates") or []),
                edge_states=edge_states,
                current_block=block_id,
                current_hb=hb_id if block_id is None else None,
                faulted=False,
                state_header_id=header_id,
                state_timestamp=timestamp,
                connection_received_at=(
                    previous.connection_received_at if previous else None
                ),
                connection_header_id=(
                    previous.connection_header_id if previous else None
                ),
                connection_timestamp=(
                    previous.connection_timestamp if previous else ""
                ),
                connection_state=(previous.connection_state if previous else ""),
                manufacturer=str(payload.get("manufacturer") or ""),
                serial_number=str(payload.get("serialNumber") or ""),
                position_initialized=bool(
                    isinstance(position, dict)
                    and position.get("positionInitialized", False)
                ),
                map_id=(
                    str(position.get("mapId") or "")
                    if isinstance(position, dict)
                    else ""
                ),
                operating_mode=str(payload.get("operatingMode") or ""),
                e_stop=str((payload.get("safetyState") or {}).get("eStop") or ""),
                field_violation=bool(
                    (payload.get("safetyState") or {}).get("fieldViolation", False)
                ),
                paused=bool(payload.get("paused", False)),
                error_levels=tuple(
                    str(item.get("errorLevel") or "").upper()
                    for item in (payload.get("errors") or [])
                    if isinstance(item, dict) and item.get("errorLevel")
                ),
            )
            self._robots[robot_id] = telemetry
            self._fault_if_unsafe_inside(robot_id, now)

    def ingest_connection(
        self,
        robot_id: str,
        payload: dict[str, Any],
        *,
        received_at: float | None = None,
    ) -> None:
        now = time.monotonic() if received_at is None else float(received_at)
        with self._lock:
            if not self._identity_matches(robot_id, payload):
                logger.error("[TRAFFIC] rejected connection identity robot=%s", robot_id)
                return
            previous = self._robots.get(robot_id)
            header_id = self._header_id(payload)
            timestamp = str(payload.get("timestamp") or "")
            if previous is not None and not self._is_newer(
                header_id,
                timestamp,
                previous.connection_header_id,
                previous.connection_timestamp,
            ):
                logger.warning(
                    "[TRAFFIC] ignored out-of-order connection robot=%s", robot_id
                )
                return
            if previous is None:
                previous = RobotTelemetry(robot_id=robot_id, received_at=now)
            self._robots[robot_id] = replace(
                previous,
                connection_received_at=now,
                connection_header_id=header_id,
                connection_timestamp=timestamp,
                connection_state=str(payload.get("connectionState") or ""),
                manufacturer=str(payload.get("manufacturer") or previous.manufacturer),
                serial_number=str(payload.get("serialNumber") or previous.serial_number),
            )
            self._fault_if_unsafe_inside(robot_id, now)

    @staticmethod
    def _header_id(payload: dict[str, Any]) -> int | None:
        value = payload.get("headerId")
        try:
            return int(value) if value is not None else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _is_newer(
        header_id: int | None,
        timestamp: str,
        previous_header_id: int | None,
        previous_timestamp: str,
    ) -> bool:
        if previous_timestamp and timestamp:
            return timestamp > previous_timestamp
        if previous_header_id is not None and header_id is not None:
            return header_id > previous_header_id
        return True

    def _identity_matches(self, robot_id: str, payload: dict[str, Any]) -> bool:
        if self.eligibility_policy is None:
            return True
        expected = self.eligibility_policy.robots.get(robot_id)
        if expected is None:
            return False
        manufacturer = str(payload.get("manufacturer") or "")
        serial_number = str(payload.get("serialNumber") or "")
        return (
            manufacturer == expected.manufacturer
            and serial_number == expected.serial_number
        )

    def eligibility(
        self, robot_id: str, *, now: float | None = None
    ) -> EligibilityResult:
        if self.eligibility_policy is None:
            return EligibilityResult(True, ())
        current = time.monotonic() if now is None else float(now)
        with self._lock:
            return self.eligibility_policy.evaluate(
                self._robots.get(robot_id), current
            )

    def telemetry(self, robot_id: str) -> RobotTelemetry | None:
        with self._lock:
            item = self._robots.get(robot_id)
            return replace(item) if item is not None else None

    def _fault_if_unsafe_inside(self, robot_id: str, now: float) -> None:
        telemetry = self._robots.get(robot_id)
        if telemetry is None or self.eligibility_policy is None:
            return
        authority = self.arbiter.authority_for_robot(robot_id)
        if telemetry.current_block is None and authority is None:
            return
        result = self.eligibility_policy.evaluate(telemetry, now)
        if result.eligible or telemetry.faulted:
            return
        reason = result.reasons[0]
        telemetry.faulted = True
        self.arbiter.fault(robot_id, reason=reason)

    def current_safe_node(self, robot_id: str) -> str | None:
        with self._lock:
            telemetry = self._robots.get(robot_id)
            if telemetry is None or telemetry.current_block is not None:
                return None
            if telemetry.current_hb is not None:
                return self.registry.holding_bays[telemetry.current_hb].node_id
            hb_id = self.registry.holding_bay_for_node(telemetry.last_node_id)
            return telemetry.last_node_id if hb_id is not None else None

    def has_arrived(self, robot_id: str, node_id: str) -> bool:
        with self._lock:
            telemetry = self._robots.get(robot_id)
            if telemetry is None or telemetry.current_block is not None:
                return False
            safe_node = self.current_safe_node(robot_id)
            return safe_node == node_id and not telemetry.driving

    def expire_stale(self, *, now: float | None = None) -> list[str]:
        current = time.monotonic() if now is None else float(now)
        expired: list[str] = []
        with self._lock:
            for robot_id, telemetry in self._robots.items():
                if (
                    (
                        telemetry.current_block is not None
                        or self.arbiter.authority_for_robot(robot_id) is not None
                    )
                    and not telemetry.faulted
                    and current - telemetry.received_at > self.telemetry_timeout
                ):
                    telemetry.faulted = True
                    self.arbiter.fault(robot_id, reason="telemetry_timeout")
                    expired.append(robot_id)
        return sorted(expired)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                robot_id: {
                    "received_at": state.received_at,
                    "position": (
                        {"x": state.x, "y": state.y}
                        if state.x is not None and state.y is not None
                        else None
                    ),
                    "last_node_id": state.last_node_id,
                    "driving": state.driving,
                    "current_block": state.current_block,
                    "current_hb": state.current_hb,
                    "faulted": state.faulted,
                    "state_header_id": state.state_header_id,
                    "connection_state": state.connection_state,
                    "connection_received_at": state.connection_received_at,
                    "position_initialized": state.position_initialized,
                    "map_id": state.map_id,
                    "operating_mode": state.operating_mode,
                    "e_stop": state.e_stop,
                    "field_violation": state.field_violation,
                    "paused": state.paused,
                    "error_levels": list(state.error_levels),
                    "eligibility": {
                        "eligible": self.eligibility(robot_id).eligible,
                        "reasons": list(self.eligibility(robot_id).reasons),
                    },
                }
                for robot_id, state in sorted(self._robots.items())
            }


class MqttStateMonitor:
    """Thin paho-mqtt adapter; core tracker stays independent and testable."""

    def __init__(
        self,
        tracker: RobotTracker,
        *,
        host: str,
        port: int,
        topic: str,
        client_id: str = "direction_arbiter_state_monitor",
        mqtt_config: MqttDeploymentConfig | None = None,
        environ: Mapping[str, str] | None = None,
    ) -> None:
        import paho.mqtt.client as mqtt

        self.tracker = tracker
        values = os.environ if environ is None else environ
        self.host = mqtt_config.host if mqtt_config is not None else host
        self.port = int(mqtt_config.port if mqtt_config is not None else port)
        self.keepalive = int(
            mqtt_config.keepalive_sec if mqtt_config is not None else 60
        )
        self.topic = topic
        self.connection_topic = (
            f"{topic[:-len('state')]}connection"
            if topic.endswith("state")
            else f"{topic}/connection"
        )
        self._connected = False
        if hasattr(mqtt, "CallbackAPIVersion"):
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
            )
        else:
            self._client = mqtt.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        if mqtt_config is not None:
            self._client.reconnect_delay_set(
                min_delay=1,
                max_delay=mqtt_config.reconnect_max_delay_sec,
            )
            username = (
                mqtt_config.username.resolve(values)
                if mqtt_config.username is not None
                else ""
            )
            password = (
                mqtt_config.password.resolve(values)
                if mqtt_config.password is not None
                else ""
            )
            if username:
                self._client.username_pw_set(username, password)
            if bool(mqtt_config.cert_file) != bool(mqtt_config.key_file):
                raise ValueError(
                    "MQTT client certificate and key must be configured together"
                )
            if mqtt_config.tls_required:
                if mqtt_config.ca_file is None:
                    raise ValueError("MQTT TLS requires a CA file")
                self._client.tls_set(
                    ca_certs=str(mqtt_config.ca_file),
                    certfile=(
                        str(mqtt_config.cert_file)
                        if mqtt_config.cert_file is not None
                        else None
                    ),
                    keyfile=(
                        str(mqtt_config.key_file)
                        if mqtt_config.key_file is not None
                        else None
                    ),
                )

    @property
    def is_connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        self._client.connect(self.host, self.port, keepalive=self.keepalive)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    def _on_connect(self, client, userdata, flags, rc, *args) -> None:
        code = rc.value if hasattr(rc, "value") else rc
        if code == 0:
            self._connected = True
            client.subscribe(self.topic, qos=1)
            client.subscribe(self.connection_topic, qos=1)
            logger.info(
                "[TRAFFIC] MQTT subscriptions: %s, %s",
                self.topic,
                self.connection_topic,
            )
        else:
            self._connected = False
            logger.error("[TRAFFIC] MQTT connection failed rc=%s", rc)

    def _on_disconnect(self, client, userdata, flags_or_rc, rc=None, *args) -> None:
        self._connected = False
        code = flags_or_rc if rc is None else rc
        if hasattr(code, "value"):
            code = code.value
        if code:
            logger.warning("[TRAFFIC] MQTT disconnected rc=%s", code)

    def _on_message(self, client, userdata, message) -> None:
        try:
            payload = json.loads(message.payload.decode("utf-8"))
            parts = message.topic.split("/")
            robot_id = parts[-2]
            if parts[-1] == "state":
                self.tracker.ingest_state(robot_id, payload)
            elif parts[-1] == "connection":
                self.tracker.ingest_connection(robot_id, payload)
        except (IndexError, UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as error:
            logger.error("[TRAFFIC] invalid state message topic=%s error=%s", message.topic, error)
