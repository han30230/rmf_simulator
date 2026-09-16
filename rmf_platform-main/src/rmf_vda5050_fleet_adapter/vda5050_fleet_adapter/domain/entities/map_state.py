"""RECONSTRUCTED VDA5050 map entity."""

from dataclasses import dataclass

from vda5050_fleet_adapter.domain.enums import MapStatus


@dataclass
class AgvMap:
    map_id: str
    map_version: str
    map_status: MapStatus = MapStatus.DISABLED
