"""RECONSTRUCTED VDA5050 load entity."""

from dataclasses import dataclass

from vda5050_fleet_adapter.domain.value_objects.physical import (
    BoundingBoxReference,
    LoadDimensions,
)


@dataclass
class Load:
    load_id: str = ''
    load_type: str = ''
    load_position: str = ''
    bounding_box_reference: BoundingBoxReference | None = None
    load_dimensions: LoadDimensions | None = None
    weight: float | None = None
