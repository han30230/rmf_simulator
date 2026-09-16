"""ActionHandler 추상 인터페이스."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Callable

from vda5050_fleet_adapter.usecase.ports.robot_api import RobotAPI


@dataclass(frozen=True)
class PathModifyResult:
    """navigate 경로 수정 결과."""

    path: list[str]
    final_destination_override: str | None = None


@dataclass(frozen=True)
class ActionHandleResult:
    """execute_action 처리 결과."""

    handled: bool = True


class ActionHandler(ABC):
    """어플리케이션별 action 처리 전략."""

    @abstractmethod
    def modify_navigate_path(
        self,
        path: list[str],
        dest_name: str,
        nav_nodes: dict,
        dest_attrs: dict,
    ) -> PathModifyResult:
        """Navigate 시 경로를 수정한다."""

    @abstractmethod
    def should_skip_navigate(
        self,
        path: list[str],
        current_node: str | None = None,
    ) -> bool:
        """로봇이 이미 목적지에 있어 navigate를 스킵할지 판단."""

    @abstractmethod
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
        """execute_action 콜백을 처리한다."""

    @abstractmethod
    def check_action_states(
        self,
        api: RobotAPI,
        robot_name: str,
        is_commissioned: bool = True,
    ) -> None:
        """Update 루프에서 action 완료를 감시한다."""

    @abstractmethod
    def reset(self) -> None:
        """모든 내부 상태를 초기화한다."""
