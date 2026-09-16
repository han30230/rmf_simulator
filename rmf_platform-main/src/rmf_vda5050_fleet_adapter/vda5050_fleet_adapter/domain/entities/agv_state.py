"""RECONSTRUCTED aggregate VDA5050 State entity."""

from dataclasses import dataclass, field

from vda5050_fleet_adapter.domain.entities.action import ActionState
from vda5050_fleet_adapter.domain.entities.battery import BatteryState
from vda5050_fleet_adapter.domain.entities.edge import EdgeState
from vda5050_fleet_adapter.domain.entities.error import AgvError, AgvInformation
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.load import Load
from vda5050_fleet_adapter.domain.entities.map_state import AgvMap
from vda5050_fleet_adapter.domain.entities.node import NodeState
from vda5050_fleet_adapter.domain.entities.safety import SafetyState
from vda5050_fleet_adapter.domain.enums import ErrorLevel, EStopType, OperatingMode
from vda5050_fleet_adapter.domain.value_objects.position import AgvPosition, Velocity


@dataclass
class AgvState:
    header: Header
    order_id: str = ''
    order_update_id: int = 0
    last_node_id: str = ''
    last_node_sequence_id: int = 0
    driving: bool = False
    new_base_request: bool = False
    distance_since_last_node: float | None = None
    operating_mode: OperatingMode = OperatingMode.AUTOMATIC
    paused: bool = False
    node_states: list[NodeState] = field(default_factory=list)
    edge_states: list[EdgeState] = field(default_factory=list)
    action_states: list[ActionState] = field(default_factory=list)
    agv_position: AgvPosition | None = None
    velocity: Velocity | None = None
    loads: list[Load] = field(default_factory=list)
    battery_state: BatteryState | None = None
    errors: list[AgvError] = field(default_factory=list)
    information: list[AgvInformation] = field(default_factory=list)
    safety_state: SafetyState = field(default_factory=SafetyState)
    maps: list[AgvMap] = field(default_factory=list)

    @property
    def has_fatal_error(self) -> bool:
        return any(error.error_level == ErrorLevel.FATAL for error in self.errors)

    @property
    def is_emergency_stopped(self) -> bool:
        return self.safety_state.e_stop != EStopType.NONE
