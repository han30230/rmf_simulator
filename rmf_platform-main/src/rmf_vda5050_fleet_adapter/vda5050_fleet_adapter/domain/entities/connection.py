"""RECONSTRUCTED VDA5050 connection entity."""

from dataclasses import dataclass

from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.enums import ConnectionState


@dataclass
class Connection:
    header: Header
    connection_state: ConnectionState
