"""RECONSTRUCTED VDA5050 enum values used by supplied modules."""

from enum import Enum


class ActionStatus(str, Enum):
    WAITING = 'WAITING'
    INITIALIZING = 'INITIALIZING'
    RUNNING = 'RUNNING'
    PAUSED = 'PAUSED'
    FINISHED = 'FINISHED'
    FAILED = 'FAILED'


class BlockingType(str, Enum):
    NONE = 'NONE'
    SOFT = 'SOFT'
    HARD = 'HARD'


class ConnectionState(str, Enum):
    ONLINE = 'ONLINE'
    OFFLINE = 'OFFLINE'
    CONNECTIONBROKEN = 'CONNECTIONBROKEN'


class CorridorRefPoint(str, Enum):
    KINEMATICCENTER = 'KINEMATICCENTER'


class ErrorLevel(str, Enum):
    WARNING = 'WARNING'
    FATAL = 'FATAL'


class EStopType(str, Enum):
    AUTOACK = 'AUTOACK'
    MANUAL = 'MANUAL'
    REMOTE = 'REMOTE'
    NONE = 'NONE'


class InfoLevel(str, Enum):
    INFO = 'INFO'
    DEBUG = 'DEBUG'


class MapStatus(str, Enum):
    ENABLED = 'ENABLED'
    DISABLED = 'DISABLED'


class OperatingMode(str, Enum):
    AUTOMATIC = 'AUTOMATIC'
    SEMIAUTOMATIC = 'SEMIAUTOMATIC'
    MANUAL = 'MANUAL'
    SERVICE = 'SERVICE'
    TEACHIN = 'TEACHIN'
