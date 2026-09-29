"""로봇 상태 관리 및 이동 시뮬레이션."""

from __future__ import annotations

import asyncio
import logging
import math
from typing import Optional, Callable

from .models import (
    Node,
    Edge,
    Action,
    ActionState,
    AgvPosition,
    Velocity,
    BatteryState,
    SafetyState,
    Load,
    NodeState,
    EdgeState,
    ErrorEntry,
    InfoEntry,
    MapState,
)

logger = logging.getLogger(__name__)


OBSTACLE_WARNING_ERROR_TYPE = "11504"
OBSTACLE_WARNING_LEVEL = "WARNING"


class Robot:
    def __init__(self, config: dict):
        robot_cfg = config["robot"]
        pos_cfg = robot_cfg["initial_position"]
        bat_cfg = robot_cfg["battery"]
        pub_cfg = config["publishing"]

        self._manufacturer = robot_cfg["manufacturer"]
        self._serial_number = robot_cfg["serial_number"]
        self._max_speed = robot_cfg["max_speed"]
        self._rotation_speed = robot_cfg["rotation_speed"]
        self._acceleration = robot_cfg.get("acceleration", 0.75)
        self._deceleration = robot_cfg.get("deceleration", 0.75)
        self._stitching_tolerance_mult = robot_cfg.get("stitching_tolerance_multiplier", 3)
        self._stitching_skip_next_closer_ratio = min(
            1.0,
            max(
                0.0,
                float(robot_cfg.get("stitching_skip_next_closer_ratio", 1.0)),
            ),
        )
        self._last_node_update_distance = max(
            0.0, float(robot_cfg.get("last_node_update_distance", 0.5))
        )
        self._tick = pub_cfg["simulation_tick"]
        self._current_speed: float = 0.0  # 현재 선속도 (가감속용)

        obstacle_cfg = robot_cfg.get("obstacle_detection", {})
        self._obstacle_detection_enabled: bool = bool(
            obstacle_cfg.get("enabled", False)
        )
        self._obstacle_front_distance: float = max(
            0.0, float(obstacle_cfg.get("front_distance", 0.0))
        )
        self._obstacle_rear_distance: float = max(
            0.0, float(obstacle_cfg.get("rear_distance", 0.0))
        )
        self._obstacle_left_distance: float = max(
            0.0, float(obstacle_cfg.get("left_distance", 0.0))
        )
        self._obstacle_right_distance: float = max(
            0.0, float(obstacle_cfg.get("right_distance", 0.0))
        )
        self._manual_obstacle_stop: bool = False

        # 위치
        self.position_x: float = pos_cfg["x"]
        self.position_y: float = pos_cfg["y"]
        self.position_theta: float = pos_cfg["theta"]
        self.map_id: str = pos_cfg["map_id"]
        self.position_initialized: bool = True

        # 속도
        self.velocity_vx: float = 0.0
        self.velocity_vy: float = 0.0
        self.velocity_omega: float = 0.0

        # 배터리
        self._battery_charge: float = bat_cfg["initial_charge"]
        self._discharge_rate: float = bat_cfg["discharge_rate"]
        self._idle_discharge_rate: float = bat_cfg.get("idle_discharge_rate", 0.01)
        self._charge_rate: float = bat_cfg.get("charge_rate", 0.1)
        self.battery_state_charging: bool = False

        # 주문 상태
        self.order_id: str = ""
        self.order_update_id: int = 0
        self.last_node_id: str = ""
        self.last_node_sequence_id: int = 0
        self.driving: bool = False
        self.new_base_request: bool = False
        self.distance_since_last_node: float = 0.0

        # 운영 상태
        self.operating_mode: str = "AUTOMATIC"
        self.paused: bool = False

        # 상태 배열
        self.node_states: list[NodeState] = []
        self.edge_states: list[EdgeState] = []
        self.action_states: list[ActionState] = []
        self.loads: list[Load] = []
        self.errors: list[ErrorEntry] = []
        self.information: list[InfoEntry] = []
        self.maps: list[MapState] = [
            MapState(mapId=pos_cfg["map_id"], mapVersion="1.0", mapStatus="ENABLED")
        ]
        self.safety_state = SafetyState(eStop="NONE", fieldViolation=False)

        # 로그 접두사 (멀티 로봇 구분용)
        self._log_prefix = f"[{self._serial_number}]"

        # 내부 주행 상태
        self._nodes: list[Node] = []
        self._edges: list[Edge] = []
        self._current_node_index: int = 0  # 다음 목표 노드 인덱스
        self._current_edge_index: int = 0  # 현재 주행 중인 엣지 인덱스
        self._navigation_active: bool = False
        self._order_cancelled: bool = False
        self._download_map_pending_action_id: str | None = None
        self._logged_moving_to: str = ""  # 이동 시작 로그 중복 방지
        self._peer_robots: list[Robot] = []
        self._obstacle_blocker_serial: str | None = None

        # 상태 변경 콜백
        self._state_change_callback: Callable | None = None
        # action_handler는 나중에 설정
        self._action_handler = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    @property
    def serial_number(self) -> str:
        return self._serial_number

    def set_action_handler(self, handler):
        self._action_handler = handler

    def set_peer_robots(self, robots: list[Robot]):
        """장애물 감지를 위해 같은 프로세스의 로봇 목록을 설정한다."""
        self._peer_robots = robots

    def set_state_change_callback(self, callback: Callable):
        self._state_change_callback = callback

    def notify_state_change(self):
        if self._state_change_callback:
            self._state_change_callback()

    def set_download_map_pending(self, action_id: str):
        """downloadMap 액션 pending 설정."""
        self._download_map_pending_action_id = action_id
        logger.info("%s downloadMap pending 설정: %s", self._log_prefix, action_id)

    def is_download_map_ready(self) -> bool:
        """downloadMap 완료 여부 확인."""
        if self._download_map_pending_action_id is None:
            return True
        for astate in self.action_states:
            if astate.actionId == self._download_map_pending_action_id:
                if astate.actionStatus in ("FINISHED", "FAILED"):
                    return True
                return False
        return True

    def _consume_idle_battery(self):
        """정지/대기 상태의 배터리 소모를 적용한다."""
        if not self.battery_state_charging:
            self._battery_charge -= self._idle_discharge_rate * self._tick
            if self._battery_charge < 0:
                self._battery_charge = 0

    def _set_obstacle_warning(
        self,
        active: bool,
        description: str = "",
        hint: str = "",
    ) -> bool:
        """11504 obstacle warning을 active errors에 반영한다."""
        existing = [
            error for error in self.errors
            if error.errorType == OBSTACLE_WARNING_ERROR_TYPE
        ]
        if active:
            description = description or "Robot stopped by obstacle"
            if existing:
                changed = False
                for error in existing:
                    if error.errorLevel != OBSTACLE_WARNING_LEVEL:
                        error.errorLevel = OBSTACLE_WARNING_LEVEL
                        changed = True
                    if error.errorDescription != description:
                        error.errorDescription = description
                        changed = True
                    normalized_hint = hint or None
                    if error.errorHint != normalized_hint:
                        error.errorHint = normalized_hint
                        changed = True
                return changed

            self.errors.append(
                ErrorEntry(
                    errorType=OBSTACLE_WARNING_ERROR_TYPE,
                    errorLevel=OBSTACLE_WARNING_LEVEL,
                    errorDescription=description,
                    errorHint=hint or None,
                )
            )
            return True

        if not existing:
            return False
        self.errors = [
            error for error in self.errors
            if error.errorType != OBSTACLE_WARNING_ERROR_TYPE
        ]
        return True

    def _find_obstacle_blocker(self) -> Robot | None:
        """감지 영역 안에 있는 가장 가까운 다른 로봇을 찾는다."""
        if not self._obstacle_detection_enabled:
            return None

        nearest_robot = None
        nearest_distance = math.inf
        cos_theta = math.cos(self.position_theta)
        sin_theta = math.sin(self.position_theta)

        for robot in self._peer_robots:
            if robot is self:
                continue
            if robot.map_id != self.map_id:
                continue

            dx = robot.position_x - self.position_x
            dy = robot.position_y - self.position_y
            local_x = dx * cos_theta + dy * sin_theta
            local_y = -dx * sin_theta + dy * cos_theta

            in_detection_area = (
                -self._obstacle_rear_distance <= local_x <= self._obstacle_front_distance
                and -self._obstacle_right_distance <= local_y <= self._obstacle_left_distance
            )
            if not in_detection_area:
                continue

            distance = math.sqrt(dx * dx + dy * dy)
            if distance < nearest_distance:
                nearest_robot = robot
                nearest_distance = distance

        return nearest_robot

    def _stop_for_obstacle_if_needed(self) -> bool:
        """장애물이 있으면 주행을 멈추고 True를 반환한다."""
        if self._manual_obstacle_stop:
            warning_changed = self._set_obstacle_warning(
                True,
                "Robot stopped by obstacle command",
                "Manual obstacle stop command is active",
            )
            motion_changed = (
                self.driving
                or self.velocity_vx != 0.0
                or self.velocity_vy != 0.0
                or self.velocity_omega != 0.0
                or self._current_speed != 0.0
            )
            self.driving = False
            self.velocity_vx = 0.0
            self.velocity_vy = 0.0
            self.velocity_omega = 0.0
            self._current_speed = 0.0
            self.safety_state.fieldViolation = True
            self._consume_idle_battery()
            if warning_changed or motion_changed:
                self.notify_state_change()
            return True

        blocker = self._find_obstacle_blocker()
        blocker_serial = blocker._serial_number if blocker else None
        blocker_changed = blocker_serial != self._obstacle_blocker_serial
        warning_changed = False

        if blocker_changed:
            if blocker is None:
                logger.info("%s 장애물 해제 — 주행 재개 가능", self._log_prefix)
                warning_changed = self._set_obstacle_warning(False)
                self.safety_state.fieldViolation = False
            else:
                logger.info(
                    "%s 장애물 감지 — 정지: blocker=%s, 위치=(%.2f, %.2f)",
                    self._log_prefix,
                    blocker._serial_number,
                    blocker.position_x,
                    blocker.position_y,
                )
                warning_changed = self._set_obstacle_warning(
                    True,
                    "Robot stopped by detected obstacle",
                    f"Blocked by robot {blocker._serial_number}",
                )
                self.safety_state.fieldViolation = True
            self._obstacle_blocker_serial = blocker_serial

        if blocker is None:
            if blocker_changed or warning_changed:
                self.notify_state_change()
            return False

        motion_changed = (
            self.driving
            or self.velocity_vx != 0.0
            or self.velocity_vy != 0.0
            or self.velocity_omega != 0.0
            or self._current_speed != 0.0
        )
        self.driving = False
        self.velocity_vx = 0.0
        self.velocity_vy = 0.0
        self.velocity_omega = 0.0
        self._current_speed = 0.0
        self.safety_state.fieldViolation = True
        self._consume_idle_battery()

        if blocker_changed or warning_changed or motion_changed:
            self.notify_state_change()
        return True

    def _distance_to_node(self, node: Node) -> float:
        """현재 위치에서 노드까지의 거리를 계산한다."""
        if node.nodePosition is None:
            return 0.0
        dx = node.nodePosition.x - self.position_x
        dy = node.nodePosition.y - self.position_y
        return math.sqrt(dx * dx + dy * dy)

    def _nearest_positioned_node(self, nodes: list[Node]):
        """현재 위치에서 가장 가까운 위치 보유 노드를 찾는다."""
        nearest_node = None
        nearest_distance = math.inf
        for node in nodes:
            if node.nodePosition is None:
                continue
            if node.nodePosition.mapId and node.nodePosition.mapId != self.map_id:
                continue
            distance = self._distance_to_node(node)
            if distance < nearest_distance:
                nearest_node = node
                nearest_distance = distance
        return nearest_node, nearest_distance

    def _is_nearest_order_node(
        self,
        node: Node,
        nodes: list[Node],
        distance: float | None = None,
    ) -> bool:
        """node가 현재 위치에서 order 내 가장 가까운 노드인지 확인한다."""
        nearest_node, nearest_distance = self._nearest_positioned_node(nodes)
        if nearest_node is None:
            return True
        if nearest_node.sequenceId == node.sequenceId:
            return True
        if distance is None:
            distance = self._distance_to_node(node)
        return distance <= nearest_distance + 1e-9

    def _has_passed_node_towards_next(self, node: Node, next_node: Node) -> bool:
        """현재 위치가 node를 지나 next_node 방향에 있는지 판별한다."""
        if node.nodePosition is None or next_node.nodePosition is None:
            return False
        if not next_node.released:
            return False
        if node.nodePosition.mapId and node.nodePosition.mapId != self.map_id:
            return False

        node_x = node.nodePosition.x
        node_y = node.nodePosition.y
        next_x = next_node.nodePosition.x
        next_y = next_node.nodePosition.y
        edge_dx = next_x - node_x
        edge_dy = next_y - node_y
        edge_len = math.hypot(edge_dx, edge_dy)
        if edge_len <= 1e-9:
            return False

        pos_dx = self.position_x - node_x
        pos_dy = self.position_y - node_y
        projected = (pos_dx * edge_dx + pos_dy * edge_dy) / edge_len
        if projected <= 1e-6:
            return False

        lateral = abs(pos_dx * edge_dy - pos_dy * edge_dx) / edge_len
        allowed_xy = node.nodePosition.allowedDeviationXY or 0.0
        tolerance = max(0.05, float(allowed_xy), self._last_node_update_distance)
        lateral_tolerance = tolerance * max(1.0, float(self._stitching_tolerance_mult))
        return lateral <= lateral_tolerance

    def _has_passed_node_from_previous(self, previous_node: Node, node: Node) -> bool:
        """현재 위치가 previous_node→node 진행 방향으로 node를 지났는지 판별한다."""
        if previous_node.nodePosition is None or node.nodePosition is None:
            return False
        if node.nodePosition.mapId and node.nodePosition.mapId != self.map_id:
            return False

        prev_x = previous_node.nodePosition.x
        prev_y = previous_node.nodePosition.y
        node_x = node.nodePosition.x
        node_y = node.nodePosition.y
        edge_dx = node_x - prev_x
        edge_dy = node_y - prev_y
        edge_len = math.hypot(edge_dx, edge_dy)
        if edge_len <= 1e-9:
            return False

        past_dx = self.position_x - node_x
        past_dy = self.position_y - node_y
        projected_past = (past_dx * edge_dx + past_dy * edge_dy) / edge_len
        if projected_past <= 1e-6:
            return False

        pos_dx = self.position_x - prev_x
        pos_dy = self.position_y - prev_y
        lateral = abs(pos_dx * edge_dy - pos_dy * edge_dx) / edge_len
        allowed_xy = node.nodePosition.allowedDeviationXY or 0.0
        tolerance = max(0.05, float(allowed_xy), self._last_node_update_distance)
        lateral_tolerance = tolerance * max(1.0, float(self._stitching_tolerance_mult))
        return lateral <= lateral_tolerance

    def _is_at_last_node_for_stitching(self, node: Node) -> bool:
        """lastNodeId가 가리키는 노드에 실제로 붙어 있는지 판별한다."""
        if node.nodePosition is None:
            return True
        if node.nodePosition.mapId and node.nodePosition.mapId != self.map_id:
            return False
        tolerance = max(0.05, self._last_node_update_distance)
        return self._distance_to_node(node) <= tolerance

    def _is_next_node_closer(self, node: Node, next_node: Node) -> bool:
        """다음 노드가 현재 후보 노드보다 가까운지 판별한다."""
        if node.nodePosition is None or next_node.nodePosition is None:
            return False
        if not next_node.released:
            return False
        if next_node.nodeId == self.last_node_id:
            return False
        if node.nodePosition.mapId and node.nodePosition.mapId != self.map_id:
            return False
        if next_node.nodePosition.mapId and next_node.nodePosition.mapId != self.map_id:
            return False

        node_distance = self._distance_to_node(node)
        next_distance = self._distance_to_node(next_node)
        if node_distance <= self._last_node_update_distance:
            return False

        return next_distance < (
            node_distance * self._stitching_skip_next_closer_ratio
        )

    def _leading_passed_node_count(self, nodes: list[Node]) -> int:
        """새 order 앞쪽에 이미 지나온 released 노드가 몇 개인지 계산한다."""
        count = 0
        while count < len(nodes) - 1:
            previous_node = nodes[count - 1] if count > 0 else None
            node = nodes[count]
            next_node = nodes[count + 1]
            if not node.released:
                break
            if (
                self.last_node_id == node.nodeId
                and self._is_at_last_node_for_stitching(node)
            ):
                count += 1
                continue
            if previous_node is not None:
                if self._has_passed_node_from_previous(previous_node, node):
                    count += 1
                    continue
                break
            if self._has_passed_node_towards_next(node, next_node):
                count += 1
                continue
            break
        return count

    def _is_passthrough_node(self, node: Node, node_index: int) -> bool:
        """통과 가능 노드인지 판별. released + theta 없음 + HARD 액션 없음 + 다음 노드도 released."""
        if not node.released:
            return False
        if node.nodePosition is not None and node.nodePosition.theta is not None:
            return False
        for action in node.actions:
            if action.blockingType == "HARD":
                return False
        # 다음 노드가 존재하고 released여야 통과 가능
        next_idx = node_index + 1
        if next_idx >= len(self._nodes):
            return False
        next_node = self._nodes[next_idx]
        if not next_node.released:
            return False
        return True

    def _distance_to_stop_node(self) -> float:
        """현재 위치에서 실제 정지해야 하는 노드까지의 누적 거리를 계산한다."""
        total_dist = 0.0
        prev_x = self.position_x
        prev_y = self.position_y
        for i in range(self._current_node_index, len(self._nodes)):
            node = self._nodes[i]
            if node.nodePosition is None:
                continue
            dx = node.nodePosition.x - prev_x
            dy = node.nodePosition.y - prev_y
            total_dist += math.sqrt(dx * dx + dy * dy)
            if not self._is_passthrough_node(node, i):
                break
            prev_x = node.nodePosition.x
            prev_y = node.nodePosition.y
        return total_dist

    def _target_speed_for_current_edge(self) -> float:
        """현재 주행 edge의 order maxSpeed를 목표 속도로 사용한다."""
        if not 0 <= self._current_edge_index < len(self._edges):
            return float(self._max_speed)

        edge = self._edges[self._current_edge_index]
        if edge.maxSpeed is None:
            return float(self._max_speed)

        try:
            target_speed = float(edge.maxSpeed)
        except (TypeError, ValueError):
            logger.warning(
                "%s edge maxSpeed 무시: edgeId=%s, maxSpeed=%r",
                self._log_prefix,
                edge.edgeId,
                edge.maxSpeed,
            )
            return float(self._max_speed)

        if not math.isfinite(target_speed) or target_speed < 0.0:
            logger.warning(
                "%s edge maxSpeed 무시: edgeId=%s, maxSpeed=%r",
                self._log_prefix,
                edge.edgeId,
                edge.maxSpeed,
            )
            return float(self._max_speed)

        return target_speed

    def _rotate_towards_node_theta(self, node: Node) -> bool:
        """Rotate one simulation tick toward node theta.

        Returns True when theta is already satisfied. XY arrival and node
        completion stay in the main navigation loop, so order updates cannot
        race a nested rotation loop.
        """
        if node.nodePosition is None or node.nodePosition.theta is None:
            return True

        self.position_x = node.nodePosition.x
        self.position_y = node.nodePosition.y
        target_theta = node.nodePosition.theta
        tolerance = node.nodePosition.allowedDeviationTheta or 0.01
        angle_diff = math.atan2(
            math.sin(target_theta - self.position_theta),
            math.cos(target_theta - self.position_theta),
        )
        if abs(angle_diff) <= tolerance:
            self.position_theta = target_theta
            self.velocity_vx = 0.0
            self.velocity_vy = 0.0
            self.velocity_omega = 0.0
            self._current_speed = 0.0
            return True

        if self._stop_for_obstacle_if_needed():
            return False

        rot_step = self._rotation_speed * self._tick
        if rot_step >= abs(angle_diff):
            self.position_theta = target_theta
        else:
            direction = 1.0 if angle_diff > 0 else -1.0
            self.position_theta += direction * rot_step
            self.position_theta = math.atan2(
                math.sin(self.position_theta),
                math.cos(self.position_theta),
            )

        self.velocity_vx = 0.0
        self.velocity_vy = 0.0
        self.velocity_omega = (
            self._rotation_speed if angle_diff > 0
            else -self._rotation_speed
        )
        self._current_speed = 0.0
        self.driving = True
        self._battery_charge -= self._discharge_rate * self._tick
        if self._battery_charge < 0:
            self._battery_charge = 0
        return False

    def apply_order(
        self,
        order_id: str,
        order_update_id: int,
        nodes: list[Node],
        edges: list[Edge],
        node_states: list[NodeState],
        edge_states: list[EdgeState],
    ):
        """검증된 주문을 로봇에 적용하고 주행을 시작한다."""
        self.order_id = order_id
        self.order_update_id = order_update_id
        self._nodes = nodes
        self._edges = edges
        self.node_states = node_states
        self.edge_states = edge_states
        self._order_cancelled = False
        self.paused = False
        self.new_base_request = False

        # 액션 상태 생성
        self.action_states = []
        for node in nodes:
            for action in node.actions:
                astate = self._action_handler.create_action_state(action, "WAITING")
                self.action_states.append(astate)
        for edge in edges:
            for action in edge.actions:
                astate = self._action_handler.create_action_state(action, "WAITING")
                self.action_states.append(astate)

        # 새 order가 상태 갱신보다 빨리 들어오면 앞쪽에 이미 지난 노드가
        # 포함될 수 있다. 그런 노드는 완료된 것으로 보고 다음 목표로 직행한다.
        first_node = nodes[0]
        dist_to_first = self._distance_to_node(first_node)
        skipped_node_count = self._leading_passed_node_count(nodes)

        if skipped_node_count > 0:
            skipped_nodes = nodes[:skipped_node_count]
            skipped_sequences = {node.sequenceId for node in skipped_nodes}
            self.node_states = [
                ns for ns in self.node_states
                if ns.sequenceId not in skipped_sequences
            ]
            completed_edge_sequences = {
                edge.sequenceId
                for edge in edges[:max(0, skipped_node_count - 1)]
            }
            if completed_edge_sequences:
                self.edge_states = [
                    es for es in self.edge_states
                    if es.sequenceId not in completed_edge_sequences
                ]

            last_skipped = skipped_nodes[-1]
            self.last_node_id = last_skipped.nodeId
            self.last_node_sequence_id = last_skipped.sequenceId
            self.distance_since_last_node = self._distance_to_node(last_skipped)
            self._current_node_index = skipped_node_count
            self._current_edge_index = skipped_node_count - 1
            self._navigation_active = True
            target_node = nodes[self._current_node_index]
            self.driving = target_node.released
            self.new_base_request = not target_node.released
            if not self.driving:
                self.velocity_vx = 0.0
                self.velocity_vy = 0.0
                self.velocity_omega = 0.0
                self._current_speed = 0.0
            if (
                self.driving
                and 0 <= self._current_edge_index < len(edges)
                and edges[self._current_edge_index].released
            ):
                asyncio.ensure_future(
                    self._trigger_edge_actions(edges[self._current_edge_index])
                )
            logger.info(
                "%s order 앞쪽 통과 노드 스킵: %s → 목표 노드(%s) "
                "(현재위치=(%.2f, %.2f), 첫 노드까지 거리=%.2fm)",
                self._log_prefix,
                [node.nodeId for node in skipped_nodes],
                target_node.nodeId,
                self.position_x,
                self.position_y,
                dist_to_first,
            )
        else:
            # stitching 미도달 / 단일 노드 / 역방향 replan → 첫 노드를 향해 주행
            self._current_node_index = 0
            self._current_edge_index = -1
            self._navigation_active = True
            self.driving = first_node.released
            self.new_base_request = not first_node.released

        self.notify_state_change()
        node_ids = [n.nodeId for n in nodes]
        logger.info(
            "%s 주문 적용: orderId=%s, updateId=%d, 노드 %d개 %s, 엣지 %d개",
            self._log_prefix, order_id, order_update_id,
            len(nodes), node_ids, len(edges),
        )

    def cancel_current_order(self):
        """현재 주문을 취소한다."""
        self._order_cancelled = True
        self._navigation_active = False
        self.paused = False
        self.driving = False
        self._current_speed = 0.0
        self.velocity_vx = 0.0
        self.velocity_vy = 0.0
        self.velocity_omega = 0.0
        self.node_states = []
        self.edge_states = []
        self.order_id = ""
        self.order_update_id = 0
        self.new_base_request = False
        logger.info("%s 주문 취소됨, 로봇 정지", self._log_prefix)

    async def navigation_loop(self):
        """메인 네비게이션 루프. asyncio 태스크로 실행."""
        while True:
            await asyncio.sleep(self._tick)

            if not self._navigation_active or self.paused or self._order_cancelled:
                self.velocity_vx = 0.0
                self.velocity_vy = 0.0
                self.velocity_omega = 0.0
                if self.paused and self._navigation_active:
                    self.driving = False
                # 대기 중에도 배터리 소모 (충전 중이 아닐 때)
                if not self.battery_state_charging:
                    self._battery_charge -= self._idle_discharge_rate * self._tick
                    if self._battery_charge < 0:
                        self._battery_charge = 0
                continue

            # downloadMap 완료 대기
            if not self.is_download_map_ready():
                self.velocity_vx = 0.0
                self.velocity_vy = 0.0
                self.velocity_omega = 0.0
                self.driving = False
                if not self.battery_state_charging:
                    self._battery_charge -= self._idle_discharge_rate * self._tick
                    if self._battery_charge < 0:
                        self._battery_charge = 0
                continue

            if self._current_node_index >= len(self._nodes):
                # 모든 노드 통과 완료
                self._navigation_active = False
                self.driving = False
                self.velocity_vx = 0.0
                self.velocity_vy = 0.0
                self.velocity_omega = 0.0
                self.notify_state_change()
                logger.info(
                    "%s 모든 노드 주행 완료 (orderId=%s, 최종위치=(%.2f, %.2f))",
                    self._log_prefix, self.order_id, self.position_x, self.position_y,
                )
                continue

            target_node = self._nodes[self._current_node_index]

            # released 확인 - Horizon 노드는 주행하지 않음.
            # 단일 노드 order도 released=true일 때만 아래 주행 로직으로 이동한다.
            if not target_node.released:
                self.new_base_request = True
                self.driving = False
                self.velocity_vx = 0.0
                self.velocity_vy = 0.0
                self.velocity_omega = 0.0
                if not self.battery_state_charging:
                    self._battery_charge -= self._idle_discharge_rate * self._tick
                    if self._battery_charge < 0:
                        self._battery_charge = 0
                continue
            else:
                if not self.driving:
                    logger.info(
                        "%s 주행 재개 → 목표노드=%s (seq=%d)",
                        self._log_prefix, target_node.nodeId, target_node.sequenceId,
                    )
                self.new_base_request = False
                self.driving = True

            if self._stop_for_obstacle_if_needed():
                continue

            if target_node.nodePosition is None:
                # 위치 없는 노드는 즉시 통과
                logger.info(
                    "%s 위치 없는 노드 즉시 통과: %s (seq=%d)",
                    self._log_prefix, target_node.nodeId, target_node.sequenceId,
                )
                await self._arrive_at_node(target_node)
                continue

            target_x = target_node.nodePosition.x
            target_y = target_node.nodePosition.y
            deviation = self._last_node_update_distance

            # 현재 위치에서 목표까지 거리
            dx = target_x - self.position_x
            dy = target_y - self.position_y
            distance = math.sqrt(dx * dx + dy * dy)

            # 현재 엣지의 maxSpeed 또는 기본 속도
            target_speed = self._target_speed_for_current_edge()

            # 노드 도달 확인
            if (
                distance <= deviation
                and self._is_nearest_order_node(
                    target_node, self._nodes, distance,
                )
            ):
                if not self._rotate_towards_node_theta(target_node):
                    continue

                is_passthrough = self._is_passthrough_node(
                    target_node, self._current_node_index,
                )
                if is_passthrough:
                    # 통과 노드: 속도 유지하며 통과
                    logger.info(
                        "%s ▶ 노드 통과(감속 없음): %s (seq=%d)",
                        self._log_prefix, target_node.nodeId,
                        target_node.sequenceId,
                    )
                    await self._arrive_at_node(target_node, keep_speed=True)
                else:
                    self._current_speed = 0.0
                    await self._arrive_at_node(target_node)
                continue

            # ── 이동 시작 로그 (노드별 1회) ──
            if self._logged_moving_to != target_node.nodeId:
                self._logged_moving_to = target_node.nodeId
                logger.info(
                    "%s 이동 시작 → 목표=%s (%.2f, %.2f), 현재위치=(%.2f, %.2f), "
                    "거리=%.2fm, 목표속도=%.2fm/s",
                    self._log_prefix, target_node.nodeId, target_x, target_y,
                    self.position_x, self.position_y, distance, target_speed,
                )

            # ── 엣지 orientation 회전 (이동 전) ──
            # 엣지에 orientation이 있으면 해당 방향으로 먼저 회전한다.
            # 이동 중에는 heading을 변경하지 않고 x,y 좌표만 변경한다.
            _reverse_traverse = False
            current_edge = (
                self._edges[self._current_edge_index]
                if 0 <= self._current_edge_index < len(self._edges)
                else None
            )
            if current_edge is not None and current_edge.orientation is not None:
                edge_theta = current_edge.orientation
                angle_diff = math.atan2(
                    math.sin(edge_theta - self.position_theta),
                    math.cos(edge_theta - self.position_theta),
                )
                rotation_threshold = 0.05  # ~3도
                if abs(angle_diff) > rotation_threshold:
                    # orientation으로 회전 중 — 이동하지 않음
                    rot_step = self._rotation_speed * self._tick
                    if rot_step >= abs(angle_diff):
                        self.position_theta = edge_theta
                    else:
                        rot_dir = 1.0 if angle_diff > 0 else -1.0
                        self.position_theta += rot_dir * rot_step
                    self.position_theta = math.atan2(
                        math.sin(self.position_theta),
                        math.cos(self.position_theta),
                    )
                    self.velocity_vx = 0.0
                    self.velocity_vy = 0.0
                    self.velocity_omega = (
                        self._rotation_speed
                        if angle_diff > 0 else -self._rotation_speed
                    )
                    self._current_speed = 0.0
                    self._battery_charge -= self._discharge_rate * self._tick
                    if self._battery_charge < 0:
                        self._battery_charge = 0
                    continue

            # 전진/후진 판단 (현재 heading과 목표 방향의 내적)
            heading_x = math.cos(self.position_theta)
            heading_y = math.sin(self.position_theta)
            if distance > 0:
                dot = heading_x * (dx / distance) + heading_y * (dy / distance)
                _reverse_traverse = dot < 0

            # ── 가감속 계산 ──
            # 감속에 필요한 거리: v² / (2 * decel)
            decel_dist = (
                self._current_speed * self._current_speed
                / (2 * self._deceleration)
            ) if self._deceleration > 0 else 0

            # 실제 정지 노드까지의 거리로 감속 판단
            stop_distance = self._distance_to_stop_node()

            if stop_distance <= decel_dist:
                # 감속 구간
                self._current_speed = max(
                    0.05,
                    self._current_speed
                    - self._deceleration * self._tick,
                )
            elif self._current_speed < target_speed:
                # 가속 구간
                self._current_speed = min(
                    target_speed,
                    self._current_speed
                    + self._acceleration * self._tick,
                )
            elif self._current_speed > target_speed:
                # order에서 낮아진 maxSpeed도 감속으로 따라간다.
                self._current_speed = max(
                    target_speed,
                    self._current_speed
                    - self._deceleration * self._tick,
                )
            else:
                # 정속 구간
                self._current_speed = target_speed

            speed = self._current_speed

            # 이동
            move_dist = speed * self._tick
            if move_dist >= distance:
                move_dist = distance

            if distance > 0:
                ratio = move_dist / distance
                self.position_x += dx * ratio
                self.position_y += dy * ratio

            # 방향은 회전 단계에서 이미 설정됨 (이동 중 덮어쓰지 않음)

            # 속도 업데이트 (후진 시 음수 방향)
            vel_sign = -1.0 if _reverse_traverse else 1.0
            self.velocity_vx = (
                vel_sign * (dx / distance) * speed if distance > 0 else 0.0
            )
            self.velocity_vy = (
                vel_sign * (dy / distance) * speed if distance > 0 else 0.0
            )
            self.velocity_omega = 0.0

            # 이동 거리 누적
            self.distance_since_last_node += move_dist

            # 배터리 소모
            self._battery_charge -= self._discharge_rate * self._tick
            if self._battery_charge < 0:
                self._battery_charge = 0

    async def _arrive_at_node(self, node: Node, *, keep_speed: bool = False):
        """노드 도착 처리. keep_speed=True이면 속도를 유지하며 통과."""
        prev_last_node = self.last_node_id
        pos_str = ""
        if node.nodePosition:
            pos_str = f", 위치=({node.nodePosition.x:.2f}, {node.nodePosition.y:.2f})"
        if not keep_speed:
            logger.info(
                "%s ★ 노드 도착: %s (seq=%d)%s, lastNodeId: %s → %s",
                self._log_prefix, node.nodeId, node.sequenceId, pos_str,
                prev_last_node, node.nodeId,
            )

        # 정확한 위치로 맞춤
        if node.nodePosition:
            self.position_x = node.nodePosition.x
            self.position_y = node.nodePosition.y

        # nodeStates에서 제거
        self.node_states = [
            ns for ns in self.node_states if ns.sequenceId != node.sequenceId
        ]

        # 이전 엣지 완료 (edgeStates에서 제거)
        if 0 <= self._current_edge_index < len(self._edges):
            completed_edge = self._edges[self._current_edge_index]
            self.edge_states = [
                es for es in self.edge_states if es.sequenceId != completed_edge.sequenceId
            ]
            # 엣지 액션이 아직 WAITING이면 FINISHED로 처리
            for action in completed_edge.actions:
                for astate in self.action_states:
                    if astate.actionId == action.actionId and astate.actionStatus == "WAITING":
                        astate.actionStatus = "FINISHED"

        # lastNodeId 업데이트 — 실제 도착 시에만 갱신
        self.last_node_id = node.nodeId
        self.last_node_sequence_id = node.sequenceId
        self.distance_since_last_node = 0.0

        # 노드 액션 트리거
        await self._trigger_node_actions(node)

        # 다음 노드/엣지로 이동
        self._current_node_index += 1
        self._current_edge_index += 1

        # 다음 엣지 액션 트리거
        if self._current_edge_index < len(self._edges):
            next_edge = self._edges[self._current_edge_index]
            if next_edge.released:
                await self._trigger_edge_actions(next_edge)

        # 모든 Base 노드 완료 확인
        remaining_base = [ns for ns in self.node_states if ns.released]
        if not remaining_base and self._current_node_index >= len(self._nodes):
            self.driving = False
            self._navigation_active = False
            logger.info(
                "%s ★ 모든 주행 완료 (orderId=%s, 최종위치=(%.2f, %.2f), 배터리=%.1f%%)",
                self._log_prefix, self.order_id,
                self.position_x, self.position_y, self._battery_charge,
            )

        self.notify_state_change()

    async def _trigger_node_actions(self, node: Node):
        """노드의 액션을 트리거."""
        for action in node.actions:
            for astate in self.action_states:
                if astate.actionId == action.actionId and astate.actionStatus == "WAITING":
                    if action.blockingType == "HARD":
                        # HARD 액션: 완료될 때까지 대기
                        await self._action_handler.trigger_action(action, astate)
                        # HARD 블로킹이면 완료 대기
                        while astate.actionStatus in ("INITIALIZING", "RUNNING"):
                            await asyncio.sleep(0.1)
                    else:
                        # NONE/SOFT: 비동기 실행
                        await self._action_handler.trigger_action(action, astate)

    async def _trigger_edge_actions(self, edge: Edge):
        """엣지의 액션을 트리거."""
        for action in edge.actions:
            for astate in self.action_states:
                if astate.actionId == action.actionId and astate.actionStatus == "WAITING":
                    await self._action_handler.trigger_action(action, astate)

    async def charging_loop(self):
        """충전 시뮬레이션 루프."""
        while True:
            await asyncio.sleep(1.0)
            if self.battery_state_charging and self._battery_charge < 100.0:
                old_charge = self._battery_charge
                self._battery_charge = min(
                    100.0, self._battery_charge + self._charge_rate
                )
                old_tens = int(old_charge) // 10
                new_tens = int(self._battery_charge) // 10
                if new_tens > old_tens:
                    logger.info(
                        "%s 배터리 충전 중: %.1f%% (rate=%.1f%%/s)",
                        self._log_prefix, self._battery_charge, self._charge_rate,
                    )

    async def move_linear(self, distance_m: float, forward: bool = True):
        """현재 heading 방향으로 직선 이동 시뮬레이션. forward=False이면 후진."""
        previous_driving = self.driving
        self.driving = True
        self.notify_state_change()

        direction = 1.0 if forward else -1.0
        cos_theta = math.cos(self.position_theta)
        sin_theta = math.sin(self.position_theta)
        target_x = self.position_x + direction * distance_m * cos_theta
        target_y = self.position_y + direction * distance_m * sin_theta

        remaining = distance_m
        speed = self._max_speed

        while remaining > 0.001:
            await asyncio.sleep(self._tick)
            if self._stop_for_obstacle_if_needed():
                continue
            if not self.driving:
                self.driving = True
                self.notify_state_change()
            move_dist = min(speed * self._tick, remaining)
            self.position_x += direction * move_dist * cos_theta
            self.position_y += direction * move_dist * sin_theta
            remaining -= move_dist
            self.velocity_vx = direction * speed * cos_theta
            self.velocity_vy = direction * speed * sin_theta

        self.position_x = target_x
        self.position_y = target_y
        self.velocity_vx = 0.0
        self.velocity_vy = 0.0
        self.driving = previous_driving
        self.notify_state_change()

    def set_manual_obstacle_stop(self, enabled: bool):
        """GUI/테스트용 수동 장애물 정지 상태를 설정한다."""
        enabled = bool(enabled)
        if self._manual_obstacle_stop == enabled:
            return

        self._manual_obstacle_stop = enabled
        self.safety_state.fieldViolation = enabled
        if enabled:
            self.driving = False
            self.velocity_vx = 0.0
            self.velocity_vy = 0.0
            self.velocity_omega = 0.0
            self._current_speed = 0.0
            self._set_obstacle_warning(
                True,
                "Robot stopped by obstacle command",
                "Manual obstacle stop command is active",
            )
            logger.warning("%s 수동 장애물 정지 활성화", self._log_prefix)
        else:
            blocker = self._find_obstacle_blocker()
            if blocker is None:
                self._obstacle_blocker_serial = None
                self._set_obstacle_warning(False)
                self.safety_state.fieldViolation = False
            else:
                self._obstacle_blocker_serial = blocker._serial_number
                self._set_obstacle_warning(
                    True,
                    "Robot stopped by detected obstacle",
                    f"Blocked by robot {blocker._serial_number}",
                )
                self.safety_state.fieldViolation = True
            logger.info("%s 수동 장애물 정지 해제", self._log_prefix)
        self.notify_state_change()

    def set_battery_charge(self, charge: float):
        """GUI/테스트용 배터리 잔량 설정."""
        old_charge = self._battery_charge
        self._battery_charge = min(100.0, max(0.0, float(charge)))
        logger.info(
            "%s 배터리 잔량 설정: %.1f%% -> %.1f%%",
            self._log_prefix,
            old_charge,
            self._battery_charge,
        )
        self.notify_state_change()

    def raise_error(
        self,
        error_type: str,
        description: str = "",
        level: str = "WARNING",
        hint: str = "",
    ):
        """GUI/테스트용 VDA5050 error entry를 추가한다."""
        normalized_level = level.upper()
        if normalized_level not in ("WARNING", "FATAL"):
            normalized_level = "WARNING"
        error = ErrorEntry(
            errorType=error_type or "simulatedError",
            errorLevel=normalized_level,
            errorDescription=description or None,
            errorHint=hint or None,
        )
        self.errors.append(error)
        logger.warning(
            "%s 에러 발생: [%s] %s - %s",
            self._log_prefix,
            normalized_level,
            error.errorType,
            description,
        )
        self.notify_state_change()

    def clear_errors(self):
        """GUI/테스트용 error entry 초기화."""
        if not self.errors:
            return
        self.errors.clear()
        logger.info("%s 에러 목록 초기화", self._log_prefix)
        self.notify_state_change()

    def get_agv_position(self) -> AgvPosition:
        return AgvPosition(
            x=round(self.position_x, 4),
            y=round(self.position_y, 4),
            theta=round(self.position_theta, 4),
            mapId=self.map_id,
            positionInitialized=self.position_initialized,
            localizationScore=1.0,
        )

    def get_velocity(self) -> Velocity:
        return Velocity(
            vx=round(self.velocity_vx, 4),
            vy=round(self.velocity_vy, 4),
            omega=round(self.velocity_omega, 4),
        )

    def get_battery_state(self) -> BatteryState:
        return BatteryState(
            batteryCharge=round(self._battery_charge, 1),
            charging=self.battery_state_charging,
            batteryVoltage=48.0,
            reach=int(self._battery_charge * 50),
        )
