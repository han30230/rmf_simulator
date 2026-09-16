"""RECONSTRUCTED VDA5050 battery entity."""

from dataclasses import dataclass


@dataclass
class BatteryState:
    battery_charge: float
    charging: bool = False
    battery_voltage: float | None = None
    battery_health: float | None = None
    reach: float | None = None
