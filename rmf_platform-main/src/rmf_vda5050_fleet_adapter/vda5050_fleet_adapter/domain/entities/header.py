"""RECONSTRUCTED VDA5050 header entity."""

from dataclasses import dataclass, field
from datetime import UTC, datetime


@dataclass
class Header:
    header_id: int = 0
    timestamp: datetime = field(default_factory=lambda: datetime.now(UTC))
    version: str = '2.0.0'
    manufacturer: str = ''
    serial_number: str = ''
