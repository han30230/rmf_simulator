"""VDA5050 state tracking and geometry-based corridor occupancy detection."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import logging
import threading
import time
from typing import Any

from .corridor_registry import CorridorRegistry
from .direction_arbiter import DirectionArbiter

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


class RobotTracker:
    def __init__(
        self,
        registry: CorridorRegistry,
        arbiter: DirectionArbiter,
        *,
        telemetry_timeout: float = 5.0,
    ) -> None:
        self.registry = registry
        self.arbiter = arbiter
        self.telemetry_timeout = float(telemetry_timeout)
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
                granted_block = self.arbiter.active_block_for_robot(robot_id)
                destination_hb = self.arbiter.destination_holding_bay_for_robot(
                    robot_id
                )
                source_hb = self.arbiter.source_holding_bay_for_robot(robot_id)
                hb_id = self.registry.holding_bay_for_position(x, y)
                if release_matches:
                    observed_block = None
                    hb_id = None
                elif hb_id is not None and (
                    granted_block is None
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
            )
            self._robots[robot_id] = telemetry

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
                    telemetry.current_block is not None
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
    ) -> None:
        import paho.mqtt.client as mqtt

        self.tracker = tracker
        self.host = host
        self.port = int(port)
        self.topic = topic
        if hasattr(mqtt, "CallbackAPIVersion"):
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
            )
        else:
            self._client = mqtt.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def start(self) -> None:
        self._client.connect(self.host, self.port)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    def _on_connect(self, client, userdata, flags, rc, *args) -> None:
        code = rc.value if hasattr(rc, "value") else rc
        if code == 0:
            client.subscribe(self.topic, qos=1)
            logger.info("[TRAFFIC] MQTT state subscription: %s", self.topic)
        else:
            logger.error("[TRAFFIC] MQTT connection failed rc=%s", rc)

    def _on_message(self, client, userdata, message) -> None:
        try:
            payload = json.loads(message.payload.decode("utf-8"))
            parts = message.topic.split("/")
            robot_id = parts[-2]
            self.tracker.ingest_state(robot_id, payload)
        except (IndexError, UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as error:
            logger.error("[TRAFFIC] invalid state message topic=%s error=%s", message.topic, error)
