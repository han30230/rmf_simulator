"""RECONSTRUCTED VDA5050 physical value objects."""

from dataclasses import dataclass

from vda5050_fleet_adapter.domain.enums import CorridorRefPoint


@dataclass
class BoundingBoxReference:
    x: float
    y: float
    z: float
    theta: float = 0.0


@dataclass
class Corridor:
    left_width: float
    right_width: float
    corridor_ref_point: CorridorRefPoint = CorridorRefPoint.KINEMATICCENTER


@dataclass
class LoadDimensions:
    length: float
    width: float
    height: float = 0.0
