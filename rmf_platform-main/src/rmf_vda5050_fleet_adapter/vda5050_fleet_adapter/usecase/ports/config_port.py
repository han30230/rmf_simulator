"""설정 포트 인터페이스.

애플리케이션 설정의 로딩을 추상화한다.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class MqttConfig:
    """MQTT 브로커 접속 설정."""

    broker_host: str = 'localhost'
    broker_port: int = 1883
    keepalive_sec: int = 60
    reconnect_max_delay_sec: int = 60


@dataclass(frozen=True)
class FleetManagerConfig:
    """VDA5050 Fleet Manager (MQTT) 설정."""

    ip: str = '127.0.0.1'
    prefix: str = 'uagv/v2/manufacturer'
    port: int = 1883
    user: str = ''
    password: str = ''
    robot_state_update_frequency: float = 10.0


@dataclass(frozen=True)
class ReferenceCoordinates:
    """좌표 변환 기준점 설정."""

    rmf: list[list[float]] = field(default_factory=list)
    robot: list[list[float]] = field(default_factory=list)


class ConfigPort(ABC):
    """설정 로더 인터페이스."""

    @abstractmethod
    def load(
        self, config_path: str, nav_graph_path: str
    ) -> dict[str, Any]:
        """설정 파일을 로드하여 raw dict로 반환한다."""
