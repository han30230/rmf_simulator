"""VDA5050 v2.0 데이터 모델 정의."""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from typing import Optional


def _to_dict(obj) -> dict:
    """dataclass를 JSON 직렬화 가능한 dict로 변환. None 값은 제외."""
    if hasattr(obj, "__dataclass_fields__"):
        result = {}
        for k, v in asdict(obj).items():
            if v is not None:
                result[k] = v
        return result
    return obj


def _to_json(obj) -> str:
    return json.dumps(_to_dict(obj), ensure_ascii=False)


def _timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


# --- Action 관련 ---

@dataclass
class ActionParameter:
    key: str
    value: str


@dataclass
class Action:
    actionType: str
    actionId: str
    blockingType: str  # NONE, SOFT, HARD
    actionDescription: Optional[str] = None
    actionParameters: list[ActionParameter] = field(default_factory=list)


# --- Node / Edge ---

@dataclass
class NodePosition:
    x: float
    y: float
    mapId: str
    theta: Optional[float] = None
    allowedDeviationXY: Optional[float] = None
    allowedDeviationTheta: Optional[float] = None


@dataclass
class Node:
    nodeId: str
    sequenceId: int
    released: bool
    nodePosition: Optional[NodePosition] = None
    actions: list[Action] = field(default_factory=list)


@dataclass
class Corridor:
    leftWidth: float
    rightWidth: float
    corridorRefPoint: Optional[str] = None  # KINEMATICCENTER, CONTOUR


@dataclass
class Edge:
    edgeId: str
    sequenceId: int
    released: bool
    startNodeId: str
    endNodeId: str
    maxSpeed: Optional[float] = None
    orientation: Optional[float] = None
    rotationAllowed: Optional[bool] = None
    trajectory: Optional[dict] = None
    corridor: Optional[Corridor] = None
    actions: list[Action] = field(default_factory=list)


# --- Order (Master → AGV) ---

@dataclass
class Order:
    headerId: int
    timestamp: str
    version: str
    manufacturer: str
    serialNumber: str
    orderId: str
    orderUpdateId: int
    nodes: list[Node]
    edges: list[Edge]
    zoneSetId: Optional[str] = None


# --- Instant Actions (Master → AGV) ---

@dataclass
class InstantActions:
    headerId: int
    timestamp: str
    version: str
    manufacturer: str
    serialNumber: str
    actions: list[Action]


# --- State 관련 (AGV → Master) ---

@dataclass
class AgvPosition:
    x: float
    y: float
    theta: float
    mapId: str
    positionInitialized: bool
    localizationScore: Optional[float] = None


@dataclass
class Velocity:
    vx: Optional[float] = None
    vy: Optional[float] = None
    omega: Optional[float] = None


@dataclass
class BatteryState:
    batteryCharge: float
    charging: bool
    batteryVoltage: Optional[float] = None
    batteryHealth: Optional[float] = None
    reach: Optional[int] = None


@dataclass
class SafetyState:
    eStop: str  # AUTOACK, MANUAL, REMOTE, NONE
    fieldViolation: bool


@dataclass
class Load:
    loadId: Optional[str] = None
    loadType: Optional[str] = None
    loadPosition: Optional[str] = None
    weight: Optional[float] = None


@dataclass
class NodeState:
    nodeId: str
    sequenceId: int
    released: bool
    nodeDescription: Optional[str] = None
    nodePosition: Optional[NodePosition] = None


@dataclass
class EdgeState:
    edgeId: str
    sequenceId: int
    released: bool
    edgeDescription: Optional[str] = None


@dataclass
class ActionState:
    actionId: str
    actionType: str
    actionStatus: str  # WAITING, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED
    actionDescription: Optional[str] = None
    resultDescription: Optional[str] = None


@dataclass
class ErrorReference:
    referenceKey: str
    referenceValue: str


@dataclass
class ErrorEntry:
    errorType: str
    errorLevel: str  # WARNING, FATAL
    errorDescription: Optional[str] = None
    errorHint: Optional[str] = None
    errorReferences: list[ErrorReference] = field(default_factory=list)


@dataclass
class InfoEntry:
    infoType: str
    infoLevel: str  # INFO, DEBUG
    infoDescription: Optional[str] = None


@dataclass
class MapState:
    mapId: str
    mapVersion: str
    mapStatus: str  # ENABLED, DISABLED


@dataclass
class State:
    headerId: int
    timestamp: str
    version: str
    manufacturer: str
    serialNumber: str
    orderId: str
    orderUpdateId: int
    lastNodeId: str
    lastNodeSequenceId: int
    driving: bool
    newBaseRequest: bool
    operatingMode: str
    paused: bool
    nodeStates: list[NodeState]
    edgeStates: list[EdgeState]
    actionStates: list[ActionState]
    agvPosition: AgvPosition
    velocity: Velocity
    batteryState: BatteryState
    safetyState: SafetyState
    distanceSinceLastNode: Optional[float] = None
    loads: list[Load] = field(default_factory=list)
    errors: list[ErrorEntry] = field(default_factory=list)
    information: list[InfoEntry] = field(default_factory=list)
    maps: list[MapState] = field(default_factory=list)


@dataclass
class ConnectionMessage:
    headerId: int
    timestamp: str
    version: str
    manufacturer: str
    serialNumber: str
    connectionState: str  # ONLINE, OFFLINE, CONNECTIONBROKEN


# --- JSON 파싱 헬퍼 ---

def parse_action_parameter(d: dict) -> ActionParameter:
    return ActionParameter(key=d["key"], value=str(d["value"]))


def parse_action(d: dict) -> Action:
    params = [parse_action_parameter(p) for p in d.get("actionParameters", [])]
    return Action(
        actionType=d["actionType"],
        actionId=d["actionId"],
        blockingType=d["blockingType"],
        actionDescription=d.get("actionDescription"),
        actionParameters=params,
    )


def parse_node_position(d: dict | None) -> NodePosition | None:
    if d is None:
        return None
    return NodePosition(
        x=d["x"],
        y=d["y"],
        mapId=d["mapId"],
        theta=d.get("theta"),
        allowedDeviationXY=d.get("allowedDeviationXY"),
        allowedDeviationTheta=d.get("allowedDeviationTheta"),
    )


def parse_node(d: dict) -> Node:
    return Node(
        nodeId=d["nodeId"],
        sequenceId=d["sequenceId"],
        released=d["released"],
        nodePosition=parse_node_position(d.get("nodePosition")),
        actions=[parse_action(a) for a in d.get("actions", [])],
    )


def parse_corridor(d: dict | None) -> Corridor | None:
    if d is None:
        return None
    return Corridor(
        leftWidth=d["leftWidth"],
        rightWidth=d["rightWidth"],
        corridorRefPoint=d.get("corridorRefPoint"),
    )


def parse_edge(d: dict) -> Edge:
    return Edge(
        edgeId=d["edgeId"],
        sequenceId=d["sequenceId"],
        released=d["released"],
        startNodeId=d["startNodeId"],
        endNodeId=d["endNodeId"],
        maxSpeed=d.get("maxSpeed"),
        orientation=d.get("orientation"),
        rotationAllowed=d.get("rotationAllowed"),
        trajectory=d.get("trajectory"),
        corridor=parse_corridor(d.get("corridor")),
        actions=[parse_action(a) for a in d.get("actions", [])],
    )


def parse_order(d: dict) -> Order:
    return Order(
        headerId=d["headerId"],
        timestamp=d["timestamp"],
        version=d["version"],
        manufacturer=d["manufacturer"],
        serialNumber=d["serialNumber"],
        orderId=d["orderId"],
        orderUpdateId=d["orderUpdateId"],
        nodes=[parse_node(n) for n in d["nodes"]],
        edges=[parse_edge(e) for e in d["edges"]],
        zoneSetId=d.get("zoneSetId"),
    )


def parse_instant_actions(d: dict) -> InstantActions:
    return InstantActions(
        headerId=d["headerId"],
        timestamp=d["timestamp"],
        version=d["version"],
        manufacturer=d["manufacturer"],
        serialNumber=d["serialNumber"],
        actions=[parse_action(a) for a in d["actions"]],
    )
