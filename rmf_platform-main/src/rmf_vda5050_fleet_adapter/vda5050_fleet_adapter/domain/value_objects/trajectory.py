"""RECONSTRUCTED VDA5050 trajectory value objects."""

from dataclasses import dataclass, field


@dataclass
class ControlPoint:
    x: float
    y: float
    weight: float = 1.0


@dataclass
class Trajectory:
    degree: int
    knot_vector: tuple[float, ...] = field(default_factory=tuple)
    control_points: tuple[ControlPoint, ...] = field(default_factory=tuple)
