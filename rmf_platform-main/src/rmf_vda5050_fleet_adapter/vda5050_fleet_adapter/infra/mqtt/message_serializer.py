"""VDA5050 메시지 JSON 직렬화/역직렬화.

도메인 엔티티 ↔ VDA5050 JSON (camelCase) 변환을 담당한다.
snake_case(도메인) ↔ camelCase(VDA5050 프로토콜) 변환은
이 모듈에서만 처리한다.
"""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from datetime import datetime
from enum import Enum
import json
import re
from typing import Any

from vda5050_fleet_adapter.domain.entities.action import (
    Action, ActionParameter, ActionState,
)
from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.battery import BatteryState
from vda5050_fleet_adapter.domain.entities.connection import Connection
from vda5050_fleet_adapter.domain.entities.edge import Edge, EdgeState
from vda5050_fleet_adapter.domain.entities.error import (
    AgvError, AgvInformation, ErrorReference,
)
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.load import Load
from vda5050_fleet_adapter.domain.entities.map_state import AgvMap
from vda5050_fleet_adapter.domain.entities.node import Node, NodeState
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.entities.safety import SafetyState
from vda5050_fleet_adapter.domain.enums import (
    ActionStatus, BlockingType, ConnectionState, CorridorRefPoint,
    ErrorLevel, EStopType, InfoLevel, MapStatus, OperatingMode,
)
from vda5050_fleet_adapter.domain.value_objects.physical import (
    BoundingBoxReference, Corridor, LoadDimensions,
)
from vda5050_fleet_adapter.domain.value_objects.position import (
    AgvPosition, NodePosition, Velocity,
)
from vda5050_fleet_adapter.domain.value_objects.trajectory import (
    ControlPoint, Trajectory,
)


_SNAKE_RE = re.compile(r'_([a-z])')
_CAMEL_RE = re.compile(r'([A-Z])')
_SPECIAL_SNAKE_TO_CAMEL = {
    'e_stop': 'eStop',
    'vx': 'vx',
    'vy': 'vy',
    'allowed_deviation_xy': 'allowedDeviationXY',
}
_SPECIAL_CAMEL_TO_SNAKE = {
    value: key for key, value in _SPECIAL_SNAKE_TO_CAMEL.items()
}


def _snake_to_camel(name: str) -> str:
    if name in _SPECIAL_SNAKE_TO_CAMEL:
        return _SPECIAL_SNAKE_TO_CAMEL[name]
    return _SNAKE_RE.sub(lambda match: match.group(1).upper(), name)


def _camel_to_snake(name: str) -> str:
    if name in _SPECIAL_CAMEL_TO_SNAKE:
        return _SPECIAL_CAMEL_TO_SNAKE[name]
    return _CAMEL_RE.sub(r'_\1', name).lower()


def _serialize_value(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, datetime):
        return value.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
    if isinstance(value, (list, tuple)):
        return [_serialize_value(item) for item in value]
    if is_dataclass(value) and not isinstance(value, type):
        return _dataclass_to_dict(value)
    return value


def _dataclass_to_dict(obj: Any) -> dict[str, Any]:
    result = {}
    for field_info in fields(obj):
        value = getattr(obj, field_info.name)
        if value is None:
            continue
        result[_snake_to_camel(field_info.name)] = _serialize_value(value)
    return result


def serialize_order(order: Order) -> str:
    data = _dataclass_to_dict(order.header)
    data['orderId'] = order.order_id
    data['orderUpdateId'] = order.order_update_id
    if order.zone_set_id:
        data['zoneSetId'] = order.zone_set_id
    data['nodes'] = [_dataclass_to_dict(node) for node in order.nodes]
    data['edges'] = [_dataclass_to_dict(edge) for edge in order.edges]
    return json.dumps(data, ensure_ascii=False)


def serialize_instant_actions(header: Header, actions: list[Action]) -> str:
    data = _dataclass_to_dict(header)
    data['actions'] = [_dataclass_to_dict(action) for action in actions]
    return json.dumps(data, ensure_ascii=False)


def serialize_connection(connection: Connection) -> str:
    data = _dataclass_to_dict(connection.header)
    data['connectionState'] = connection.connection_state.value
    return json.dumps(data, ensure_ascii=False)


def _map_keys_to_snake(data: dict[str, Any]) -> dict[str, Any]:
    return {_camel_to_snake(key): value for key, value in data.items()}


def _parse_header(data: dict[str, Any]) -> Header:
    return Header(
        header_id=data.get('headerId', 0),
        timestamp=(
            datetime.fromisoformat(data['timestamp'].replace('Z', '+00:00'))
            if 'timestamp' in data else datetime.now()
        ),
        version=data.get('version', '2.0.0'),
        manufacturer=data.get('manufacturer', ''),
        serial_number=data.get('serialNumber', ''),
    )


def _parse_action_parameter(data: dict[str, Any]) -> ActionParameter:
    return ActionParameter(key=data['key'], value=data['value'])


def _parse_action(data: dict[str, Any]) -> Action:
    return Action(
        action_type=data['actionType'],
        action_id=data['actionId'],
        blocking_type=BlockingType(data['blockingType']),
        action_description=data.get('actionDescription', ''),
        action_parameters=[
            _parse_action_parameter(item)
            for item in data.get('actionParameters', [])
        ],
    )


def _parse_node_position(data: dict[str, Any]) -> NodePosition:
    return NodePosition(
        x=data['x'], y=data['y'], map_id=data['mapId'],
        theta=data.get('theta'),
        allowed_deviation_xy=data.get('allowedDeviationXY'),
        allowed_deviation_theta=data.get('allowedDeviationTheta'),
    )


def _parse_node(data: dict[str, Any]) -> Node:
    position = None
    if data.get('nodePosition') is not None:
        position = _parse_node_position(data['nodePosition'])
    return Node(
        node_id=data['nodeId'], sequence_id=data['sequenceId'],
        released=data['released'], node_position=position,
        actions=[_parse_action(item) for item in data.get('actions', [])],
    )


def _parse_corridor(data: dict[str, Any]) -> Corridor:
    return Corridor(
        left_width=data['leftWidth'], right_width=data['rightWidth'],
        corridor_ref_point=CorridorRefPoint(
            data.get('corridorRefPoint', 'KINEMATICCENTER')
        ),
    )


def _parse_control_point(data: dict[str, Any]) -> ControlPoint:
    return ControlPoint(data['x'], data['y'], data.get('weight', 1.0))


def _parse_trajectory(data: dict[str, Any]) -> Trajectory:
    return Trajectory(
        degree=data['degree'],
        knot_vector=tuple(data.get('knotVector', [])),
        control_points=tuple(
            _parse_control_point(item)
            for item in data.get('controlPoints', [])
        ),
    )


def _parse_edge(data: dict[str, Any]) -> Edge:
    trajectory = (
        _parse_trajectory(data['trajectory'])
        if data.get('trajectory') is not None else None
    )
    corridor = (
        _parse_corridor(data['corridor'])
        if data.get('corridor') is not None else None
    )
    return Edge(
        edge_id=data['edgeId'], sequence_id=data['sequenceId'],
        released=data['released'], start_node_id=data['startNodeId'],
        end_node_id=data['endNodeId'], max_speed=data.get('maxSpeed'),
        orientation=data.get('orientation'),
        rotation_allowed=data.get('rotationAllowed'),
        trajectory=trajectory, corridor=corridor,
        actions=[_parse_action(item) for item in data.get('actions', [])],
    )


def deserialize_order(payload: str) -> Order:
    data = json.loads(payload)
    return Order(
        header=_parse_header(data), order_id=data['orderId'],
        order_update_id=data['orderUpdateId'],
        zone_set_id=data.get('zoneSetId', ''),
        nodes=[_parse_node(item) for item in data.get('nodes', [])],
        edges=[_parse_edge(item) for item in data.get('edges', [])],
    )


def _parse_agv_position(data: dict[str, Any]) -> AgvPosition:
    return AgvPosition(
        x=data['x'], y=data['y'], theta=data['theta'], map_id=data['mapId'],
        position_initialized=data.get('positionInitialized', True),
        localization_score=data.get('localizationScore'),
    )


def _parse_velocity(data: dict[str, Any]) -> Velocity:
    return Velocity(
        vx=data.get('vx', 0.0), vy=data.get('vy', 0.0),
        omega=data.get('omega', 0.0),
    )


def _parse_node_state(data: dict[str, Any]) -> NodeState:
    position = (
        _parse_node_position(data['nodePosition'])
        if data.get('nodePosition') is not None else None
    )
    return NodeState(
        node_id=data['nodeId'], sequence_id=data['sequenceId'],
        released=data['released'],
        node_description=data.get('nodeDescription', ''),
        node_position=position,
    )


def _parse_edge_state(data: dict[str, Any]) -> EdgeState:
    return EdgeState(
        edge_id=data['edgeId'], sequence_id=data['sequenceId'],
        released=data['released'],
        edge_description=data.get('edgeDescription', ''),
    )


def _parse_action_state(data: dict[str, Any]) -> ActionState:
    return ActionState(
        action_id=data['actionId'], action_type=data.get('actionType', ''),
        action_status=ActionStatus(data['actionStatus']),
        action_description=data.get('actionDescription', ''),
        result_description=data.get('resultDescription', ''),
    )


def _parse_error_reference(data: dict[str, Any]) -> ErrorReference:
    return ErrorReference(data['referenceKey'], data['referenceValue'])


def _parse_agv_error(data: dict[str, Any]) -> AgvError:
    return AgvError(
        error_type=data['errorType'], error_level=ErrorLevel(data['errorLevel']),
        error_description=data.get('errorDescription', ''),
        error_hint=data.get('errorHint', ''),
        error_references=[
            _parse_error_reference(item)
            for item in data.get('errorReferences', [])
        ],
    )


def _parse_agv_information(data: dict[str, Any]) -> AgvInformation:
    return AgvInformation(
        info_type=data['infoType'], info_level=InfoLevel(data['infoLevel']),
        info_description=data.get('infoDescription', ''),
        info_references=[
            _parse_error_reference(item)
            for item in data.get('infoReferences', [])
        ],
    )


def _parse_bounding_box(data: dict[str, Any]) -> BoundingBoxReference:
    return BoundingBoxReference(
        data['x'], data['y'], data['z'], data.get('theta', 0.0)
    )


def _parse_load_dimensions(data: dict[str, Any]) -> LoadDimensions:
    return LoadDimensions(
        data['length'], data['width'], data.get('height', 0.0)
    )


def _parse_load(data: dict[str, Any]) -> Load:
    return Load(
        load_id=data.get('loadId', ''), load_type=data.get('loadType', ''),
        load_position=data.get('loadPosition', ''),
        bounding_box_reference=(
            _parse_bounding_box(data['boundingBoxReference'])
            if data.get('boundingBoxReference') else None
        ),
        load_dimensions=(
            _parse_load_dimensions(data['loadDimensions'])
            if data.get('loadDimensions') else None
        ),
        weight=data.get('weight'),
    )


def _parse_battery_state(data: dict[str, Any]) -> BatteryState:
    return BatteryState(
        battery_charge=data['batteryCharge'],
        charging=data.get('charging', False),
        battery_voltage=data.get('batteryVoltage'),
        battery_health=data.get('batteryHealth'), reach=data.get('reach'),
    )


def _parse_safety_state(data: dict[str, Any]) -> SafetyState:
    return SafetyState(
        e_stop=EStopType(data.get('eStop', 'NONE')),
        field_violation=data.get('fieldViolation', False),
    )


def _parse_agv_map(data: dict[str, Any]) -> AgvMap:
    return AgvMap(
        map_id=data['mapId'], map_version=data['mapVersion'],
        map_status=MapStatus(data.get('mapStatus', 'DISABLED')),
    )


def deserialize_state(payload: str) -> AgvState:
    data = json.loads(payload)
    agv_position = (
        _parse_agv_position(data['agvPosition'])
        if data.get('agvPosition') else None
    )
    velocity = _parse_velocity(data['velocity']) if data.get('velocity') else None
    battery = (
        _parse_battery_state(data['batteryState'])
        if data.get('batteryState') else None
    )
    safety = (
        _parse_safety_state(data['safetyState'])
        if data.get('safetyState') else SafetyState()
    )
    return AgvState(
        header=_parse_header(data), order_id=data.get('orderId', ''),
        order_update_id=data.get('orderUpdateId', 0),
        last_node_id=data.get('lastNodeId', ''),
        last_node_sequence_id=data.get('lastNodeSequenceId', 0),
        driving=data.get('driving', False),
        new_base_request=data.get('newBaseRequest', False),
        distance_since_last_node=data.get('distanceSinceLastNode'),
        operating_mode=OperatingMode(data.get('operatingMode', 'AUTOMATIC')),
        paused=data.get('paused', False),
        node_states=[
            _parse_node_state(item) for item in data.get('nodeStates', [])
        ],
        edge_states=[
            _parse_edge_state(item) for item in data.get('edgeStates', [])
        ],
        action_states=[
            _parse_action_state(item) for item in data.get('actionStates', [])
        ],
        agv_position=agv_position, velocity=velocity,
        loads=[_parse_load(item) for item in data.get('loads', [])],
        battery_state=battery,
        errors=[_parse_agv_error(item) for item in data.get('errors', [])],
        information=[
            _parse_agv_information(item)
            for item in data.get('information', [])
        ],
        safety_state=safety,
        maps=[_parse_agv_map(item) for item in data.get('maps', [])],
    )


def deserialize_connection(payload: str) -> Connection:
    data = json.loads(payload)
    return Connection(
        header=_parse_header(data),
        connection_state=ConnectionState(data['connectionState']),
    )
