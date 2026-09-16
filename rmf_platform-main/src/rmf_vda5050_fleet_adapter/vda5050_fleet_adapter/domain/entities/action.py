"""RECONSTRUCTED VDA5050 action entities."""

from dataclasses import dataclass, field
from typing import Any

from vda5050_fleet_adapter.domain.enums import ActionStatus, BlockingType


@dataclass
class ActionParameter:
    key: str
    value: Any


@dataclass
class Action:
    action_type: str
    action_id: str
    blocking_type: BlockingType
    action_description: str = ''
    action_parameters: list[ActionParameter] = field(default_factory=list)


@dataclass
class ActionState:
    action_id: str
    action_type: str = ''
    action_status: ActionStatus = ActionStatus.WAITING
    action_description: str = ''
    result_description: str = ''
