"""RECONSTRUCTED VDA5050 node entities."""

from dataclasses import dataclass, field

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.value_objects.position import NodePosition


@dataclass
class Node:
    node_id: str
    sequence_id: int
    released: bool
    node_description: str = ''
    node_position: NodePosition | None = None
    actions: list[Action] = field(default_factory=list)


@dataclass
class NodeState:
    node_id: str
    sequence_id: int
    released: bool
    node_description: str = ''
    node_position: NodePosition | None = None
