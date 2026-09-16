"""RECONSTRUCTED VDA5050 safety entity."""

from dataclasses import dataclass

from vda5050_fleet_adapter.domain.enums import EStopType


@dataclass
class SafetyState:
    e_stop: EStopType = EStopType.NONE
    field_violation: bool = False
