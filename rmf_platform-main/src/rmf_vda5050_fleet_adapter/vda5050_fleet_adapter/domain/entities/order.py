"""RECONSTRUCTED VDA5050 order entity."""

from dataclasses import dataclass, field

from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.node import Node


@dataclass
class Order:
    header: Header
    order_id: str
    order_update_id: int
    zone_set_id: str = ''
    nodes: list[Node] = field(default_factory=list)
    edges: list[Edge] = field(default_factory=list)
