"""RECONSTRUCTED VDA5050 position value objects."""

from dataclasses import dataclass


@dataclass
class AgvPosition:
    x: float
    y: float
    theta: float
    map_id: str
    position_initialized: bool = True
    localization_score: float | None = None


@dataclass
class NodePosition:
    x: float
    y: float
    map_id: str
    theta: float | None = None
    allowed_deviation_xy: float | None = None
    allowed_deviation_theta: float | None = None


@dataclass
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
