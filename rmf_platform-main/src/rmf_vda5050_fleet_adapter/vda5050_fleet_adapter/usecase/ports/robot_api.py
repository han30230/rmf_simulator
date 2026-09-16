"""RobotAPI 포트 인터페이스."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any


class RobotAPIResult(IntEnum):
    """RobotAPI 호출 결과."""

    SUCCESS = 0
    RETRY = 1
    IMPOSSIBLE = 2


@dataclass(frozen=True)
class CommissionState:
    """RMF Commission 상태."""

    accept_dispatched_tasks: bool = True
    accept_direct_tasks: bool = True
    perform_idle_behavior: bool = True


@dataclass
class RobotUpdateData:
    """주기적 상태 업데이트 데이터."""

    robot_name: str
    map_name: str
    position: list[float]
    battery_soc: float
    last_completed_cmd_id: int = 0
    last_node_id: str = ''
    driving: bool = False
    charging: bool = False


@dataclass
class RobotCommandState:
    """Command ACK 판단에 사용하는 VDA5050 state snapshot."""

    state_received_at: float
    order_id: str = ''
    order_update_id: int = 0
    last_node_id: str = ''
    last_node_sequence_id: int = 0
    driving: bool = False
    paused: bool = False
    new_base_request: bool = False
    node_states: list[Any] = field(default_factory=list)
    edge_states: list[Any] = field(default_factory=list)
    action_states: list[Any] = field(default_factory=list)
    errors: list[Any] = field(default_factory=list)
    information: list[Any] = field(default_factory=list)
    operating_mode: Any = None
    connection_state: Any = None


class RobotAPI(ABC):
    """AGV 통신 인터페이스."""

    @abstractmethod
    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        nodes: list,
        edges: list,
        map_name: str,
        order_id: str = '',
        order_update_id: int = 0,
        *,
        track_action_id: str | None = None,
    ) -> RobotAPIResult:
        """VDA5050 Order를 전송하여 내비게이션을 시작한다."""

    @abstractmethod
    def stop(self, robot_name: str, cmd_id: int) -> RobotAPIResult:
        """Cancel order instant action을 전송한다."""

    @abstractmethod
    def pause(self, robot_name: str, cmd_id: int) -> RobotAPIResult:
        """Start-pause instant action을 전송한다."""

    @abstractmethod
    def start_activity(
        self,
        robot_name: str,
        cmd_id: int,
        activity: str,
        action_params: dict,
        *,
        action_id: str | None = None,
    ) -> RobotAPIResult:
        """VDA5050 instant action을 전송한다."""

    @abstractmethod
    def get_data(self, robot_name: str) -> RobotUpdateData | None:
        """로봇의 현재 상태 데이터를 반환한다."""

    @abstractmethod
    def get_command_state(
        self, robot_name: str
    ) -> RobotCommandState | None:
        """ACK 판단용 최신 command state snapshot을 반환한다."""

    @abstractmethod
    def get_battery_soc(self, robot_name: str) -> float | None:
        """로봇의 현재 배터리 SOC를 반환한다 (0.0~1.0)."""

    @abstractmethod
    def get_commission_state(
        self, robot_name: str
    ) -> CommissionState | None:
        """VDA5050 상태 기반 commission 상태를 반환한다."""

    @abstractmethod
    def is_manual_mode(self, robot_name: str) -> bool:
        """로봇이 수동 모드인지 확인한다."""

    @abstractmethod
    def is_robot_connected(self, robot_name: str) -> bool:
        """로봇의 연결 상태가 ONLINE인지 확인한다."""

    @abstractmethod
    def is_command_completed(
        self, robot_name: str, cmd_id: int
    ) -> bool:
        """명령 완료 여부를 확인한다."""
