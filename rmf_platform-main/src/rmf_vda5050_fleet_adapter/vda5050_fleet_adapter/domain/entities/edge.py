"""RECONSTRUCTED VDA5050 edge entities."""

from dataclasses import dataclass, field

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.value_objects.physical import Corridor
from vda5050_fleet_adapter.domain.value_objects.trajectory import Trajectory


@dataclass
class Edge:
    edge_id: str
    sequence_id: int
    released: bool
    start_node_id: str
    end_node_id: str
    edge_description: str = ''
    max_speed: float | None = None
    orientation: float | None = None
    rotation_allowed: bool | None = None
    trajectory: Trajectory | None = None
    corridor: Corridor | None = None
    actions: list[Action] = field(default_factory=list)


@dataclass
class EdgeState:
    edge_id: str
    sequence_id: int
    released: bool
    edge_description: str = ''
