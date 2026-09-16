"""RECONSTRUCTED VDA5050 error and information entities."""

from dataclasses import dataclass, field

from vda5050_fleet_adapter.domain.enums import ErrorLevel, InfoLevel


@dataclass
class ErrorReference:
    reference_key: str
    reference_value: str


@dataclass
class AgvError:
    error_type: str
    error_level: ErrorLevel
    error_description: str = ''
    error_hint: str = ''
    error_references: list[ErrorReference] = field(default_factory=list)


@dataclass
class AgvInformation:
    info_type: str
    info_level: InfoLevel
    info_description: str = ''
    info_references: list[ErrorReference] = field(default_factory=list)
