"""Order 수신/검증/업데이트 관리."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from .models import (
    Order,
    Node,
    Edge,
    ErrorEntry,
    ErrorReference,
    NodeState,
    EdgeState,
    parse_order,
)

if TYPE_CHECKING:
    from .robot import Robot

logger = logging.getLogger(__name__)


class OrderManager:
    def __init__(self, robot: Robot):
        self._robot = robot
        self._current_order: Order | None = None
        self._log_prefix = f"[{robot._serial_number}]"

    @property
    def current_order(self) -> Order | None:
        return self._current_order

    def process_order(self, payload: dict) -> bool:
        """Order 메시지를 처리한다. 성공 시 True 반환."""
        # 1. 파싱
        try:
            order = parse_order(payload)
        except (KeyError, TypeError, ValueError) as e:
            self._add_error(
                "orderError", "FATAL",
                f"Order 파싱 실패: {e}",
                "JSON 포맷을 확인하세요",
            )
            return False

        # 2. 기본 검증
        if not order.nodes:
            self._add_error(
                "orderError", "FATAL",
                "노드가 비어 있습니다",
                "최소 하나의 노드가 필요합니다",
            )
            return False

        # 3. 신규 주문 vs 업데이트 판별
        if self._current_order is None or order.orderId != self._current_order.orderId:
            return self._handle_new_order(order)
        else:
            return self._handle_order_update(order)

    def _handle_new_order(self, order: Order) -> bool:
        """신규 주문 처리."""
        logger.info("%s 신규 주문 수신: %s (updateId=%d)", self._log_prefix, order.orderId, order.orderUpdateId)

        # 첫 노드에 nodePosition이 있어야 함
        first_node = order.nodes[0]
        if first_node.nodePosition is None:
            self._add_error(
                "orderError", "FATAL",
                "첫 노드에 위치 정보가 없습니다",
                "nodePosition을 포함하세요",
                [ErrorReference("nodeId", first_node.nodeId)],
            )
            return False

        # sequenceId 검증 (노드: 짝수, 엣지: 홀수)
        if not self._validate_sequence_ids(order):
            return False

        # 시작 위치 도달 가능성 확인 (현재는 간단히 통과)
        self._current_order = order
        self._apply_order(order)
        return True

    def _handle_order_update(self, order: Order) -> bool:
        """기존 주문 업데이트 처리."""
        logger.info(
            "%s 주문 업데이트 수신: %s (updateId=%d → %d)",
            self._log_prefix, order.orderId,
            self._current_order.orderUpdateId,
            order.orderUpdateId,
        )

        # orderUpdateId 증가 검증
        if order.orderUpdateId <= self._current_order.orderUpdateId:
            self._add_error(
                "orderUpdateError", "WARNING",
                f"orderUpdateId가 현재({self._current_order.orderUpdateId})보다 "
                f"크지 않음: {order.orderUpdateId}",
                "orderUpdateId를 증가시켜 전송하세요",
            )
            return False

        # Stitching 검증: 새 주문의 첫 노드 = 이전 Base의 마지막 노드
        if not self._validate_stitching(order):
            return False

        # sequenceId 검증
        if not self._validate_sequence_ids(order):
            return False

        self._current_order = order
        self._apply_order(order)
        return True

    def _validate_sequence_ids(self, order: Order) -> bool:
        """노드/엣지 sequenceId 검증 (order update 시 0이 아닌 시작값 허용)."""
        if not order.nodes:
            return True

        start_seq = order.nodes[0].sequenceId
        for i, node in enumerate(order.nodes):
            expected = start_seq + i * 2
            if node.sequenceId != expected:
                self._add_error(
                    "orderError", "WARNING",
                    f"노드 sequenceId 불일치: nodeId={node.nodeId}, "
                    f"expected={expected}, got={node.sequenceId}",
                    "노드 sequenceId는 짝수이고 2씩 증가해야 합니다",
                    [ErrorReference("nodeId", node.nodeId)],
                )
                return False

        for i, edge in enumerate(order.edges):
            expected = start_seq + i * 2 + 1
            if edge.sequenceId != expected:
                self._add_error(
                    "orderError", "WARNING",
                    f"엣지 sequenceId 불일치: edgeId={edge.edgeId}, "
                    f"expected={expected}, got={edge.sequenceId}",
                    "엣지 sequenceId는 홀수이고 2씩 증가해야 합니다",
                    [ErrorReference("edgeId", edge.edgeId)],
                )
                return False

        return True

    def _validate_stitching(self, order: Order) -> bool:
        """Stitching 검증: 새 주문의 첫 노드가 현재 주행의 마지막 Base 노드와 일치하는지."""
        if not order.nodes:
            return False

        new_first_node = order.nodes[0]
        last_node_id = self._robot.last_node_id

        if last_node_id and new_first_node.nodeId != last_node_id:
            # 현재 남은 Base 노드의 마지막 노드와 비교
            base_nodes = [ns for ns in self._robot.node_states if ns.released]
            if base_nodes:
                last_base = base_nodes[-1]
                if new_first_node.nodeId != last_base.nodeId:
                    self._add_error(
                        "orderUpdateError", "WARNING",
                        f"Stitching 실패: 새 주문 첫 노드({new_first_node.nodeId}) != "
                        f"현재 Base 마지막 노드({last_base.nodeId})",
                        "이전 Base의 마지막 노드에서 연결하세요",
                    )
                    return False

        return True

    def _apply_order(self, order: Order):
        """검증 통과한 주문을 로봇에 적용."""
        # nodeStates 세팅
        node_states = []
        for node in order.nodes:
            node_states.append(NodeState(
                nodeId=node.nodeId,
                sequenceId=node.sequenceId,
                released=node.released,
                nodePosition=node.nodePosition,
            ))

        # edgeStates 세팅
        edge_states = []
        for edge in order.edges:
            edge_states.append(EdgeState(
                edgeId=edge.edgeId,
                sequenceId=edge.sequenceId,
                released=edge.released,
            ))

        self._robot.apply_order(
            order_id=order.orderId,
            order_update_id=order.orderUpdateId,
            nodes=order.nodes,
            edges=order.edges,
            node_states=node_states,
            edge_states=edge_states,
        )

    def clear_order(self):
        """주문 클리어 (취소 후)."""
        self._current_order = None

    def _add_error(
        self,
        error_type: str,
        level: str,
        description: str,
        hint: str = "",
        refs: list[ErrorReference] | None = None,
    ):
        error = ErrorEntry(
            errorType=error_type,
            errorLevel=level,
            errorDescription=description,
            errorHint=hint,
            errorReferences=refs or [],
        )
        self._robot.errors.append(error)
        self._robot.notify_state_change()
        logger.error("%s 에러 추가: [%s] %s - %s", self._log_prefix, level, error_type, description)
