#!/usr/bin/env python3
"""VDA5050 Log Visualization GUI

Visualizes VDA5050 order/state/connection logs to help developers
inspect robot paths, order update timing, and base/horizon distinctions.
Supports multiple AGVs with per-robot color coding and filtering.

Usage:
    python vda5050_gui.py [logfile.jsonl]
"""

import asyncio
import copy
import importlib.util
import json
import logging
import math
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Set, Tuple

# 설정 파일 경로 (GUI 실행 파일과 같은 디렉토리)
_SETTINGS_PATH = Path(__file__).parent / "gui_settings.json"


def _common_node_label_prefix(node_ids) -> str:
    """Return a shared delimited prefix that can be hidden in map labels."""
    values = [str(node_id) for node_id in node_ids if node_id]
    if len(values) < 2:
        return ""
    common = values[0]
    for value in values[1:]:
        while common and not value.startswith(common):
            common = common[:-1]
    boundary = max((common.rfind(mark) for mark in ("_", "/", ":", ".", "-")), default=-1)
    prefix = common[: boundary + 1]
    if len(prefix) < 3 or any(len(value) <= len(prefix) for value in values):
        return ""
    return prefix


def _display_node_label(node_id: str, common_prefix: str) -> str:
    if common_prefix and node_id.startswith(common_prefix):
        return node_id[len(common_prefix):]
    return node_id


def _load_settings() -> dict:
    try:
        return json.loads(_SETTINGS_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _save_settings(settings: dict) -> None:
    try:
        _SETTINGS_PATH.write_text(
            json.dumps(settings, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    except Exception:
        pass


import yaml

from PyQt5.QtCore import (
    QEvent,
    QObject,
    QPointF,
    QRectF,
    Qt,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QFontMetrics,
    QIcon,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    QTransform,
    QWheelEvent,
)
from PyQt5.QtWidgets import (
    QAction,
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------
COL_HORIZON_NODE = QColor("#E0E0E0")
COL_HORIZON_EDGE = QColor("#BDBDBD")
COL_CONN_ONLINE = QColor("#4CAF50")
COL_CONN_OFFLINE = QColor("#FFC107")
COL_CONN_BROKEN = QColor("#F44336")
COL_ORDER_MARKER = QColor("#FF5722")
COL_BG = QColor("#FAFAFA")
COL_GRID = QColor("#E8E8E8")
COL_NODE_TEXT = QColor("#333333")

# Per-AGV color palette – each AGV gets a distinct theme color
AGV_PALETTE = [
    QColor("#E74C3C"),  # red
    QColor("#2196F3"),  # blue
    QColor("#4CAF50"),  # green
    QColor("#FF9800"),  # orange
    QColor("#9C27B0"),  # purple
    QColor("#00BCD4"),  # cyan
    QColor("#795548"),  # brown
    QColor("#607D8B"),  # blue-grey
    QColor("#E91E63"),  # pink
    QColor("#009688"),  # teal
]

NODE_RADIUS = 8
ROBOT_SIZE = 18
TRAIL_WIDTH = 2

# Map overlay colors
COL_MAP_NODE = QColor("#90A4AE")       # default map node (grey-blue)
COL_MAP_NODE_CHGE = QColor("#FFC107")  # charging station (amber)
COL_MAP_NODE_PARK = QColor("#8BC34A")  # parking spot (light green)
COL_MAP_NODE_PICK = QColor("#FF5722")  # pick/drop station (deep orange)
COL_MAP_EDGE = QColor("#B0BEC5")       # map edge (light grey)
COL_MAP_MUTEX_EDGE = QColor("#C2185B") # map edge with mutex group
COL_MAP_LABEL = QColor("#455A64")      # label text
MAP_NODE_RADIUS = 5
MAP_LABEL_TYPES = {"CHGE", "PARK", "PICKDROP"}  # special node types counted in map status

GUI_DIR = Path(__file__).resolve().parent
SRC_DIR = GUI_DIR.parent
SIMULATOR_ROOT = SRC_DIR / "vda5050_robot_simulator"
DEFAULT_SIM_CONFIG_PATH = SIMULATOR_ROOT / "config.yaml"


# ---------------------------------------------------------------------------
# MapData – parsed nav_graph map YAML
# ---------------------------------------------------------------------------
@dataclass
class MapVertex:
    x: float
    y: float
    name: str
    vtype: str  # NONE, CHGE, PARK, PICKDROP
    mutex: str = ""


@dataclass
class MapEdge:
    start_idx: int
    end_idx: int
    mutex: str = ""


class MapData:
    """Loads and holds nav_graph map from YAML."""

    def __init__(self):
        self.vertices: List[MapVertex] = []
        self.edges: List[MapEdge] = []
        self.edge_mutex_by_nodes: Dict[Tuple[str, str], str] = {}
        self.label_prefix = ""
        self.loaded = False
        self.file_path = ""

    @staticmethod
    def _prop_value(props: Dict[str, Any], key: str, default: str = "") -> str:
        """Read direct nav_graph props and traffic-editor [type, value] props."""
        if not isinstance(props, dict):
            return default
        value = props.get(key, default)
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            value = value[1]
        if value is None:
            return default
        value = str(value)
        return value if value else default

    @property
    def mutex_edge_count(self) -> int:
        return sum(1 for edge in self.edges if edge.mutex)

    def mutex_for_lane(self, start_node: str, end_node: str) -> str:
        return self.edge_mutex_by_nodes.get((start_node, end_node), "")

    def load(self, path: str) -> int:
        """Load map YAML, return vertex count. Raises on error."""
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        self.vertices.clear()
        self.edges.clear()
        self.edge_mutex_by_nodes.clear()
        self.file_path = path

        # Find the first level
        levels = data.get("levels", {})
        if not levels:
            return 0
        level_data = next(iter(levels.values()))

        # Parse vertices
        for v in level_data.get("vertices", []):
            x = float(v[0])
            y = float(v[1])
            if len(v) > 2 and isinstance(v[2], dict):
                props = v[2]
                name = self._prop_value(props, "name")
            else:
                props = v[4] if len(v) > 4 and isinstance(v[4], dict) else {}
                name = str(v[3]) if len(v) > 3 and v[3] is not None else ""
            vtype = self._prop_value(props, "type", "NONE")
            mutex = self._prop_value(props, "mutex")
            self.vertices.append(
                MapVertex(x=x, y=y, name=name, vtype=vtype, mutex=mutex)
            )
        self.label_prefix = _common_node_label_prefix(
            vertex.name for vertex in self.vertices
        )

        # Parse lanes (edges)
        for lane in level_data.get("lanes", []):
            start_idx = int(lane[0])
            end_idx = int(lane[1])
            props = lane[2] if len(lane) > 2 and isinstance(lane[2], dict) else {}
            mutex = self._prop_value(props, "mutex")
            self.edges.append(
                MapEdge(start_idx=start_idx, end_idx=end_idx, mutex=mutex)
            )
            if mutex and start_idx < len(self.vertices) and end_idx < len(self.vertices):
                start_name = self.vertices[start_idx].name
                end_name = self.vertices[end_idx].name
                if start_name and end_name:
                    self.edge_mutex_by_nodes[(start_name, end_name)] = mutex
                    bidirectional = self._prop_value(props, "bidirectional").lower() == "true"
                    if bidirectional:
                        self.edge_mutex_by_nodes[(end_name, start_name)] = mutex

        self.loaded = True
        return len(self.vertices)

# ---------------------------------------------------------------------------
# LogEntry – parsed JSONL line
# ---------------------------------------------------------------------------
@dataclass
class LogEntry:
    timestamp: str
    topic: str  # "order", "instantActions", "state", "connection"
    data: Dict[str, Any]
    dt: float = 0.0  # epoch seconds for sorting
    agv_id: str = ""

    def __post_init__(self):
        try:
            t = datetime.fromisoformat(self.timestamp.replace("Z", "+00:00"))
            self.dt = t.timestamp()
        except Exception:
            self.dt = 0.0
        if not self.agv_id:
            self.agv_id = self.data.get("serialNumber", "unknown")


# ---------------------------------------------------------------------------
# Snapshot – computed view at a point in time (per AGV)
# ---------------------------------------------------------------------------
@dataclass
class Snapshot:
    order: Optional[Dict[str, Any]] = None
    instant_actions: Optional[Dict[str, Any]] = None
    state: Optional[Dict[str, Any]] = None
    connection: Optional[Dict[str, Any]] = None
    # Derived helpers
    nodes: List[Dict[str, Any]] = field(default_factory=list)
    edges: List[Dict[str, Any]] = field(default_factory=list)
    agv_x: Optional[float] = None
    agv_y: Optional[float] = None
    agv_theta: Optional[float] = None
    last_node_id: Optional[str] = None
    last_node_seq_id: int = -1
    connection_state: str = "OFFLINE"
    order_id: str = ""
    order_update_id: int = 0
    driving: bool = False
    node_states: List[Dict[str, Any]] = field(default_factory=list)
    edge_states: List[Dict[str, Any]] = field(default_factory=list)
    action_states: List[Dict[str, Any]] = field(default_factory=list)
    errors: List[Dict[str, Any]] = field(default_factory=list)
    battery_charge: float = 0.0


def _populate_snapshot(snap: Snapshot, order_data, instant_actions_data, state_data, conn_data):
    """Fill derived fields of a Snapshot from raw message data."""
    snap.order = order_data
    snap.instant_actions = instant_actions_data
    snap.state = state_data
    snap.connection = conn_data

    if order_data:
        snap.nodes = order_data.get("nodes", [])
        snap.edges = order_data.get("edges", [])
        snap.order_id = order_data.get("orderId", "")
        snap.order_update_id = order_data.get("orderUpdateId", 0)

    if state_data:
        pos = state_data.get("agvPosition", {})
        if pos:
            snap.agv_x = pos.get("x")
            snap.agv_y = pos.get("y")
            snap.agv_theta = pos.get("theta", 0.0)
        snap.last_node_id = state_data.get("lastNodeId", "")
        snap.last_node_seq_id = state_data.get("lastNodeSequenceId", -1)
        snap.driving = state_data.get("driving", False)
        snap.node_states = state_data.get("nodeStates", [])
        snap.edge_states = state_data.get("edgeStates", [])
        snap.action_states = state_data.get("actionStates", [])
        snap.errors = state_data.get("errors", [])
        battery = state_data.get("batteryState", {})
        snap.battery_charge = battery.get("batteryCharge", 0.0)

    if conn_data:
        snap.connection_state = conn_data.get("connectionState", "OFFLINE")


# ---------------------------------------------------------------------------
# LogStore – log list management + snapshot computation
# ---------------------------------------------------------------------------
class LogStore:
    RETENTION_SECONDS = 300  # keep last 5 minutes
    _TRIM_BATCH = 200  # trim in batches to avoid frequent reindexing

    def __init__(self):
        self.entries: Deque[LogEntry] = deque()
        self._order_change_indices: List[int] = []
        self.collecting = True  # True = collecting, False = paused
        # Incremental snapshot cache: always reflects state at end of entries
        self._live_last_order: Dict[str, Any] = {}
        self._live_last_instant: Dict[str, Any] = {}
        self._live_last_state: Dict[str, Any] = {}
        self._live_last_conn: Dict[str, Any] = {}
        self._live_agv_ids: Set[str] = set()
        # Trail cache: per-AGV list of (x, y) from state messages
        self._trail_cache: Dict[str, Deque[Tuple[float, float]]] = {}
        self._trail_max_points = 500
        # Offset tracking for order change indices after trims
        self._trim_total = 0

    def clear(self):
        self.entries.clear()
        self._order_change_indices.clear()
        self._live_last_order.clear()
        self._live_last_instant.clear()
        self._live_last_state.clear()
        self._live_last_conn.clear()
        self._live_agv_ids.clear()
        self._trail_cache.clear()
        self._trim_total = 0

    def load_file(self, path: str) -> int:
        self.clear()
        count = 0
        entries_list: List[LogEntry] = []
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                    entry = LogEntry(
                        timestamp=obj.get("timestamp", ""),
                        topic=obj.get("topic", ""),
                        data=obj.get("data", {}),
                    )
                    entries_list.append(entry)
                    count += 1
                except (json.JSONDecodeError, KeyError):
                    continue
        entries_list.sort(key=lambda e: e.dt)
        self.entries = deque(entries_list)
        self._compute_order_changes()
        # Build snapshot cache for offline mode (full scan once)
        self._rebuild_live_cache()
        return count

    def _rebuild_live_cache(self):
        """Full rebuild of live caches from all entries (used after file load)."""
        self._live_last_order.clear()
        self._live_last_instant.clear()
        self._live_last_state.clear()
        self._live_last_conn.clear()
        self._live_agv_ids.clear()
        self._trail_cache.clear()
        for e in self.entries:
            self._update_cache_for_entry(e)

    def _update_cache_for_entry(self, e: LogEntry):
        """Update incremental caches with a single new entry."""
        aid = e.agv_id
        if aid:
            self._live_agv_ids.add(aid)
        if e.topic == "order":
            self._live_last_order[aid] = e.data
        elif e.topic == "instantActions":
            self._live_last_instant[aid] = e.data
        elif e.topic == "state":
            self._live_last_state[aid] = e.data
            # Update trail cache
            pos = e.data.get("agvPosition", {})
            x, y = pos.get("x"), pos.get("y")
            if x is not None and y is not None:
                trail = self._trail_cache.get(aid)
                if trail is None:
                    trail = deque(maxlen=self._trail_max_points)
                    self._trail_cache[aid] = trail
                if not trail or (trail[-1][0] != x or trail[-1][1] != y):
                    trail.append((x, y))
        elif e.topic == "connection":
            self._live_last_conn[aid] = e.data

    def append_entry(self, entry: LogEntry):
        if not self.collecting:
            return
        self.entries.append(entry)
        self._update_cache_for_entry(entry)
        idx = len(self.entries) - 1
        if entry.topic == "order":
            self._order_change_indices.append(idx + self._trim_total)
        self._trim_old_entries()

    def _trim_old_entries(self):
        """Remove entries older than RETENTION_SECONDS (skipped when paused)."""
        if not self.collecting or not self.entries:
            return
        cutoff = time.time() - self.RETENTION_SECONDS
        if self.entries[0].dt >= cutoff:
            return
        trim_count = 0
        for e in self.entries:
            if e.dt >= cutoff:
                break
            trim_count += 1
        if trim_count < self._TRIM_BATCH:
            return  # defer trim until batch size reached
        for _ in range(trim_count):
            self.entries.popleft()
        self._trim_total += trim_count
        # Reindex order change indices
        new_indices = [i - self._trim_total for i in self._order_change_indices
                       if i - self._trim_total >= 0]
        self._order_change_indices = new_indices
        self._trim_total = 0  # reset after reindex

    def _compute_order_changes(self):
        self._order_change_indices.clear()
        prev: Dict[str, Tuple[str, int]] = {}  # agv_id -> (orderId, orderUpdateId)
        for i, e in enumerate(self.entries):
            if e.topic == "order":
                current = (e.data.get("orderId", ""), e.data.get("orderUpdateId", 0))
                if prev.get(e.agv_id) != current:
                    self._order_change_indices.append(i)
                    prev[e.agv_id] = current

    @property
    def order_change_indices(self) -> List[int]:
        return self._order_change_indices

    def get_agv_ids(self) -> List[str]:
        """Return sorted unique AGV IDs seen in all entries."""
        return sorted(self._live_agv_ids)

    def get_snapshots(self, index: int) -> Dict[str, Snapshot]:
        """Compute per-AGV snapshots at entry[index].

        For the latest index (live mode), uses cached state for O(1).
        For arbitrary indices (offline scrubbing), falls back to linear scan.
        """
        if not self.entries or index < 0:
            return {}

        index = min(index, len(self.entries) - 1)

        # Fast path: if requesting the latest entry, use cached state
        if index == len(self.entries) - 1:
            return self._snapshots_from_cache()

        # Slow path: scan from start (offline timeline scrubbing)
        last_order: Dict[str, Any] = {}
        last_instant: Dict[str, Any] = {}
        last_state: Dict[str, Any] = {}
        last_conn: Dict[str, Any] = {}

        for i in range(index + 1):
            e = self.entries[i]
            aid = e.agv_id
            if e.topic == "order":
                last_order[aid] = e.data
            elif e.topic == "instantActions":
                last_instant[aid] = e.data
            elif e.topic == "state":
                last_state[aid] = e.data
            elif e.topic == "connection":
                last_conn[aid] = e.data

        all_agvs = set(last_order.keys()) | set(last_instant.keys()) | set(last_state.keys()) | set(last_conn.keys())
        result: Dict[str, Snapshot] = {}
        for aid in all_agvs:
            snap = Snapshot()
            _populate_snapshot(snap, last_order.get(aid), last_instant.get(aid), last_state.get(aid), last_conn.get(aid))
            result[aid] = snap
        return result

    def _snapshots_from_cache(self) -> Dict[str, Snapshot]:
        """Build snapshots from cached latest state – O(num_agvs)."""
        all_agvs = (set(self._live_last_order.keys()) | set(self._live_last_instant.keys()) |
                    set(self._live_last_state.keys()) | set(self._live_last_conn.keys()))
        result: Dict[str, Snapshot] = {}
        for aid in all_agvs:
            snap = Snapshot()
            _populate_snapshot(
                snap,
                self._live_last_order.get(aid),
                self._live_last_instant.get(aid),
                self._live_last_state.get(aid),
                self._live_last_conn.get(aid),
            )
            result[aid] = snap
        return result

    def get_trail(self, up_to_index: int, agv_id: str, max_points: int = 500) -> List[Tuple[float, float]]:
        """Collect AGV positions from state messages up to index for one AGV.

        For the latest index, uses cached trail for O(1).
        """
        # Fast path: latest index uses trail cache
        if up_to_index >= len(self.entries) - 1:
            cached = self._trail_cache.get(agv_id)
            if cached is not None:
                return list(cached)
            return []

        # Slow path: scan for arbitrary index (offline scrubbing)
        trail: List[Tuple[float, float]] = []
        start = max(0, up_to_index - max_points * 3)
        for i in range(start, min(up_to_index + 1, len(self.entries))):
            e = self.entries[i]
            if e.topic == "state" and e.agv_id == agv_id:
                pos = e.data.get("agvPosition", {})
                x = pos.get("x")
                y = pos.get("y")
                if x is not None and y is not None:
                    if not trail or (trail[-1][0] != x or trail[-1][1] != y):
                        trail.append((x, y))
        if len(trail) > max_points:
            trail = trail[-max_points:]
        return trail


# ---------------------------------------------------------------------------
# MqttLogClient – real-time MQTT listener with Qt signals
# ---------------------------------------------------------------------------
class MqttLogClient(QObject):
    log_received = pyqtSignal(object)  # LogEntry
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._client = None
        self._connected = False
        self._topics: List[Tuple[str, int]] = []

    def connect_to_broker(
        self,
        host: str,
        port: int,
        interface_name: str,
        version: str,
        manufacturer: str,
    ):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            self.error_occurred.emit("paho-mqtt not installed. Run: pip install paho-mqtt")
            return

        # Use '+' wildcard for serialNumber to receive all AGVs under this manufacturer
        prefix = f"{interface_name}/{version}/{manufacturer}/+"
        self._topics = [
            (f"{prefix}/order", 0),
            (f"{prefix}/instantActions", 0),
            (f"{prefix}/state", 0),
            (f"{prefix}/connection", 1),
        ]

        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        try:
            self._client.connect_async(host, port, keepalive=60)
            self._client.loop_start()
        except Exception as exc:
            self.error_occurred.emit(f"MQTT connect error: {exc}")

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            for topic, qos in self._topics:
                client.subscribe(topic, qos)
            self.connected.emit()
        else:
            self.error_occurred.emit(f"MQTT connect failed (rc={rc})")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.disconnected.emit()

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        parts = msg.topic.rsplit("/", 1)
        topic_type = parts[-1] if parts else "unknown"

        ts = payload.get("timestamp", datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.000Z"))
        entry = LogEntry(timestamp=ts, topic=topic_type, data=payload)
        self.log_received.emit(entry)

    def stop(self):
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected


# ---------------------------------------------------------------------------
# MqttConnectionDialog
# ---------------------------------------------------------------------------
class MqttConnectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("MQTT Connection")
        self.setMinimumWidth(400)

        layout = QFormLayout(self)

        # 저장된 설정 로드
        _s = _load_settings().get("mqtt", {})

        self.host_edit = QLineEdit(_s.get("host", "localhost"))
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(_s.get("port", 1883))

        self.interface_edit = QLineEdit(_s.get("interface_name", "uagv"))
        self.version_edit = QLineEdit(_s.get("version", "v2.0.0"))
        self.manufacturer_edit = QLineEdit(_s.get("manufacturer", "inatech"))

        layout.addRow("Broker Host:", self.host_edit)
        layout.addRow("Broker Port:", self.port_spin)
        layout.addRow("Interface Name:", self.interface_edit)
        layout.addRow("Version:", self.version_edit)
        layout.addRow("Manufacturer:", self.manufacturer_edit)

        self.preview_label = QLabel()
        self.preview_label.setStyleSheet("color: #666; font-size: 11px;")
        layout.addRow("Topic Pattern:", self.preview_label)

        hint_label = QLabel("All AGVs under this manufacturer will be discovered automatically.")
        hint_label.setStyleSheet("color: #888; font-size: 10px; font-style: italic;")
        hint_label.setWordWrap(True)
        layout.addRow("", hint_label)

        self._update_preview()
        for edit in (self.interface_edit, self.version_edit, self.manufacturer_edit):
            edit.textChanged.connect(self._update_preview)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def _update_preview(self):
        prefix = (
            f"{self.interface_edit.text()}/{self.version_edit.text()}/"
            f"{self.manufacturer_edit.text()}/+")
        self.preview_label.setText(f"{prefix}/[order|state|connection]")

    def get_params(self) -> dict:
        return {
            "host": self.host_edit.text(),
            "port": self.port_spin.value(),
            "interface_name": self.interface_edit.text(),
            "version": self.version_edit.text(),
            "manufacturer": self.manufacturer_edit.text(),
        }


# ---------------------------------------------------------------------------
# MapCanvas – 2D multi-AGV node/edge/robot visualization
# ---------------------------------------------------------------------------
class MapCanvas(QWidget):
    node_clicked = pyqtSignal(str, dict)  # nodeId, node data
    robot_clicked = pyqtSignal(str)  # agv_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

        self._snapshots: Dict[str, Snapshot] = {}
        self._trails: Dict[str, List[Tuple[float, float]]] = {}
        self._visible_agvs: Set[str] = set()
        self._selected_agv: str = ""
        self._agv_color_cache: Dict[str, QColor] = {}

        # View transform
        self._scale = 40.0
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._panning = False
        self._pan_start = QPointF()
        self._pan_offset_start = (0.0, 0.0)

        # Screen position caches for click detection
        self._node_screen_positions: Dict[str, Tuple[float, float, Dict]] = {}
        self._robot_screen_positions: Dict[str, Tuple[float, float]] = {}

        # Map overlay
        self._map_data: Optional[MapData] = None

    def set_data(
        self,
        snapshots: Dict[str, Snapshot],
        trails: Dict[str, List[Tuple[float, float]]],
        visible_agvs: Set[str],
        selected_agv: str = "",
    ):
        self._snapshots = snapshots
        self._trails = trails
        self._visible_agvs = visible_agvs
        self._selected_agv = selected_agv
        self._node_screen_positions.clear()
        self._robot_screen_positions.clear()
        self._rebuild_color_cache()
        self.update()

    def _rebuild_color_cache(self):
        all_ids = sorted(self._snapshots.keys())
        self._agv_color_cache.clear()
        for i, aid in enumerate(all_ids):
            self._agv_color_cache[aid] = AGV_PALETTE[i % len(AGV_PALETTE)]

    def get_agv_color(self, agv_id: str) -> QColor:
        return self._agv_color_cache.get(agv_id, AGV_PALETTE[0])

    def set_map_data(self, map_data: Optional[MapData]):
        self._map_data = map_data
        self.update()

    def fit_to_content(self):
        xs: List[float] = []
        ys: List[float] = []

        # Include map vertices
        if self._map_data and self._map_data.loaded:
            for v in self._map_data.vertices:
                xs.append(v.x)
                ys.append(v.y)

        for aid in self._visible_agvs:
            snap = self._snapshots.get(aid)
            if not snap:
                continue
            for n in snap.nodes:
                pos = n.get("nodePosition", {})
                x, y = pos.get("x"), pos.get("y")
                if x is not None and y is not None:
                    xs.append(x)
                    ys.append(y)
            if snap.agv_x is not None:
                xs.append(snap.agv_x)
                ys.append(snap.agv_y)

        if not xs:
            return

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        w = max_x - min_x
        h = max_y - min_y

        margin = 60
        canvas_w = self.width() - margin * 2
        canvas_h = self.height() - margin * 2

        if w > 0 or h > 0:
            sx = canvas_w / w if w > 0 else canvas_w
            sy = canvas_h / h if h > 0 else canvas_h
            self._scale = min(sx, sy, 200.0)
            self._scale = max(self._scale, 5.0)
        else:
            self._scale = 40.0

        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0
        self._offset_x = self.width() / 2.0 - cx * self._scale
        self._offset_y = self.height() / 2.0 + cy * self._scale
        self.update()

    def _world_to_screen(self, wx: float, wy: float) -> Tuple[float, float]:
        sx = wx * self._scale + self._offset_x
        sy = -wy * self._scale + self._offset_y
        return sx, sy

    def _screen_to_world(self, sx: float, sy: float) -> Tuple[float, float]:
        wx = (sx - self._offset_x) / self._scale
        wy = -(sy - self._offset_y) / self._scale
        return wx, wy

    @staticmethod
    def _build_node_pos(snap: Snapshot) -> Dict[str, Tuple[float, float]]:
        node_pos: Dict[str, Tuple[float, float]] = {}
        for n in snap.nodes:
            nid = n.get("nodeId", "")
            pos = n.get("nodePosition", {})
            x, y = pos.get("x"), pos.get("y")
            if x is not None and y is not None:
                node_pos[nid] = (x, y)
        return node_pos

    # ---- Paint ----

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), COL_BG)
        self._draw_grid(painter)

        # Draw map overlay (below everything else)
        if self._map_data and self._map_data.loaded:
            self._draw_map(painter)

        if not self._snapshots:
            if not (self._map_data and self._map_data.loaded):
                painter.setPen(QPen(QColor("#999")))
                painter.setFont(QFont("sans-serif", 14))
                painter.drawText(self.rect(), Qt.AlignCenter, "No data loaded\n\nFile → Open Log (Ctrl+O)")
            elif self._map_data.mutex_edge_count:
                self._draw_map_mutex_overlay(painter)
            painter.end()
            return

        visible_sorted = sorted(self._visible_agvs)
        self._node_screen_positions.clear()
        self._robot_screen_positions.clear()

        # Pass 1: edges
        for aid in visible_sorted:
            snap = self._snapshots.get(aid)
            if not snap or not snap.edges:
                continue
            color = self.get_agv_color(aid)
            node_pos = self._build_node_pos(snap)
            is_selected = (aid == self._selected_agv)
            self._draw_edges(painter, snap, node_pos, color, is_selected)

        # Pass 2: trails
        for aid in visible_sorted:
            trail = self._trails.get(aid, [])
            color = self.get_agv_color(aid)
            is_selected = (aid == self._selected_agv)
            self._draw_trail(painter, trail, color, is_selected)

        # Pass 3: nodes
        for aid in visible_sorted:
            snap = self._snapshots.get(aid)
            if not snap or not snap.nodes:
                continue
            color = self.get_agv_color(aid)
            node_pos = self._build_node_pos(snap)
            is_selected = (aid == self._selected_agv)
            self._draw_nodes(painter, snap, node_pos, color, is_selected)

        # Pass 3.5: map mutex lane overlay, kept above routes and below robots.
        if self._map_data and self._map_data.loaded and self._map_data.mutex_edge_count:
            self._draw_map_mutex_overlay(painter)

        # Pass 4: robots (always on top)
        for aid in visible_sorted:
            snap = self._snapshots.get(aid)
            if not snap:
                continue
            color = self.get_agv_color(aid)
            is_selected = (aid == self._selected_agv)
            self._draw_robot(painter, snap, aid, color, is_selected)

        # Legend
        if len(self._agv_color_cache) > 1:
            self._draw_legend(painter)

        painter.end()

    def _draw_grid(self, painter: QPainter):
        pen = QPen(COL_GRID, 1, Qt.DotLine)
        painter.setPen(pen)

        grid_world = 1.0
        if self._scale < 15:
            grid_world = 5.0
        elif self._scale < 8:
            grid_world = 10.0

        w, h = self.width(), self.height()
        wl, wt = self._screen_to_world(0, 0)
        wr, wb = self._screen_to_world(w, h)

        min_wx, max_wx = min(wl, wr), max(wl, wr)
        min_wy, max_wy = min(wt, wb), max(wt, wb)

        x = math.floor(min_wx / grid_world) * grid_world
        while x <= max_wx:
            sx, _ = self._world_to_screen(x, 0)
            painter.drawLine(int(sx), 0, int(sx), h)
            x += grid_world

        y = math.floor(min_wy / grid_world) * grid_world
        while y <= max_wy:
            _, sy = self._world_to_screen(0, y)
            painter.drawLine(0, int(sy), w, int(sy))
            y += grid_world

    def _visible_order_node_ids(self) -> Set[str]:
        node_ids: Set[str] = set()
        for aid in self._visible_agvs:
            snap = self._snapshots.get(aid)
            if not snap:
                continue
            for node in snap.nodes:
                nid = node.get("nodeId", "")
                if nid:
                    node_ids.add(str(nid))
        return node_ids

    def _draw_map(self, painter: QPainter):
        """Draw nav_graph map edges and nodes as background layer."""
        md = self._map_data
        if not md or not md.loaded:
            return

        verts = md.vertices
        order_node_ids = self._visible_order_node_ids()

        # Draw map edges
        pen = QPen(COL_MAP_EDGE, 1, Qt.SolidLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        for edge in md.edges:
            if edge.start_idx >= len(verts) or edge.end_idx >= len(verts):
                continue
            v1 = verts[edge.start_idx]
            v2 = verts[edge.end_idx]
            sx1, sy1 = self._world_to_screen(v1.x, v1.y)
            sx2, sy2 = self._world_to_screen(v2.x, v2.y)
            painter.drawLine(QPointF(sx1, sy1), QPointF(sx2, sy2))

        # Draw map nodes
        label_font = QFont("sans-serif", 7)
        order_label_font = QFont("sans-serif", 8, QFont.Bold)
        for v in verts:
            sx, sy = self._world_to_screen(v.x, v.y)
            # Pick color by type
            if v.vtype == "CHGE":
                color = COL_MAP_NODE_CHGE
                r = MAP_NODE_RADIUS + 2
            elif v.vtype == "PARK":
                color = COL_MAP_NODE_PARK
                r = MAP_NODE_RADIUS + 2
            elif v.vtype == "PICKDROP":
                color = COL_MAP_NODE_PICK
                r = MAP_NODE_RADIUS + 2
            else:
                color = COL_MAP_NODE
                r = MAP_NODE_RADIUS

            painter.setPen(QPen(color.darker(130), 1))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(QPointF(sx, sy), r, r)

            if v.name:
                is_order_node = v.name in order_node_ids
                active_font = order_label_font if is_order_node else label_font
                painter.setFont(active_font)
                metrics = QFontMetrics(active_font)
                label_width = 110 if is_order_node else 90
                display_name = _display_node_label(v.name, md.label_prefix)
                label = metrics.elidedText(display_name, Qt.ElideRight, label_width)
                rect_w = metrics.horizontalAdvance(label) + 8
                rect_h = metrics.height() + 3
                rect = QRectF(sx - rect_w / 2, sy - r - rect_h - 3, rect_w, rect_h)

                bg_alpha = 235 if is_order_node else 185
                painter.setPen(
                    QPen(COL_NODE_TEXT, 1) if is_order_node else Qt.NoPen
                )
                painter.setBrush(QBrush(QColor(255, 255, 255, bg_alpha)))
                painter.drawRoundedRect(rect, 2, 2)

                painter.setPen(QPen(COL_NODE_TEXT if is_order_node else COL_MAP_LABEL))
                painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_map_mutex_overlay(self, painter: QPainter):
        """Draw a visible overlay for map lanes that belong to a mutex group."""
        md = self._map_data
        if not md or not md.loaded:
            return

        verts = md.vertices
        label_font = QFont("sans-serif", 7, QFont.Bold)
        label_metrics = QFontMetrics(label_font)
        label_positions: Dict[str, List[Tuple[float, float]]] = {}

        halo = QColor(255, 255, 255, 220)
        mutex_color = QColor(COL_MAP_MUTEX_EDGE)
        mutex_color.setAlpha(230)

        for edge in md.edges:
            if not edge.mutex:
                continue
            if edge.start_idx >= len(verts) or edge.end_idx >= len(verts):
                continue

            v1 = verts[edge.start_idx]
            v2 = verts[edge.end_idx]
            sx1, sy1 = self._world_to_screen(v1.x, v1.y)
            sx2, sy2 = self._world_to_screen(v2.x, v2.y)

            halo_pen = QPen(halo, 5, Qt.SolidLine)
            halo_pen.setCapStyle(Qt.RoundCap)
            painter.setPen(halo_pen)
            painter.drawLine(QPointF(sx1, sy1), QPointF(sx2, sy2))

            pen = QPen(mutex_color, 3, Qt.SolidLine)
            pen.setCapStyle(Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QPointF(sx1, sy1), QPointF(sx2, sy2))

            self._draw_mutex_edge_tick(painter, sx1, sy1, sx2, sy2)

            if self._scale < 12:
                continue
            mx = (sx1 + sx2) / 2.0
            my = (sy1 + sy2) / 2.0
            if math.hypot(sx2 - sx1, sy2 - sy1) < 32:
                continue

            group_positions = label_positions.setdefault(edge.mutex, [])
            if any(math.hypot(mx - px, my - py) < 130 for px, py in group_positions):
                continue
            group_positions.append((mx, my))

            label = label_metrics.elidedText(edge.mutex, Qt.ElideRight, 120)
            width = label_metrics.horizontalAdvance(label) + 10
            height = label_metrics.height() + 3
            rect = QRectF(mx - width / 2, my - height - 6, width, height)

            painter.setPen(QPen(COL_MAP_MUTEX_EDGE.darker(130), 1))
            painter.setBrush(QBrush(QColor(255, 255, 255, 230)))
            painter.drawRoundedRect(rect, 3, 3)
            painter.setFont(label_font)
            painter.setPen(QPen(COL_MAP_MUTEX_EDGE.darker(120)))
            painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_mutex_edge_tick(self, painter: QPainter, sx1: float, sy1: float, sx2: float, sy2: float):
        dx = sx2 - sx1
        dy = sy2 - sy1
        length = math.hypot(dx, dy)
        if length < 8:
            return
        nx = -dy / length
        ny = dx / length
        mx = (sx1 + sx2) / 2.0
        my = (sy1 + sy2) / 2.0
        tick = 5

        tick_pen = QPen(QColor(255, 255, 255, 240), 4, Qt.SolidLine)
        tick_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(tick_pen)
        painter.drawLine(QPointF(mx - nx * tick, my - ny * tick), QPointF(mx + nx * tick, my + ny * tick))

        tick_pen = QPen(COL_MAP_MUTEX_EDGE.darker(130), 2, Qt.SolidLine)
        tick_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(tick_pen)
        painter.drawLine(QPointF(mx - nx * tick, my - ny * tick), QPointF(mx + nx * tick, my + ny * tick))

    def _draw_edges(self, painter: QPainter, snap: Snapshot, node_pos: Dict, agv_color: QColor, selected: bool = False):
        remaining_edge_ids: Set[str] = set()
        for es in snap.edge_states:
            remaining_edge_ids.add(es.get("edgeId", ""))
        last_seq = snap.last_node_seq_id

        edge_color = agv_color.darker(120)
        visited_color = QColor(agv_color)
        visited_color.setAlpha(80)

        w_mult = 2.5 if selected else 1.0

        for edge in snap.edges:
            eid = edge.get("edgeId", "")
            start_id = edge.get("startNodeId", "")
            end_id = edge.get("endNodeId", "")
            released = edge.get("released", False)
            seq = edge.get("sequenceId", -1)

            if start_id not in node_pos or end_id not in node_pos:
                continue

            sx, sy = self._world_to_screen(*node_pos[start_id])
            ex, ey = self._world_to_screen(*node_pos[end_id])

            is_visited = eid not in remaining_edge_ids and last_seq >= 0 and seq < last_seq

            if is_visited:
                pen = QPen(visited_color, 1.5 * w_mult, Qt.SolidLine)
            elif released:
                pen = QPen(edge_color, 2 * w_mult, Qt.SolidLine)
            else:
                pen = QPen(COL_HORIZON_EDGE, 1 * w_mult, Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(QPointF(sx, sy), QPointF(ex, ey))

            mutex_group = self._mutex_group_for_order_edge(edge, start_id, end_id)
            if mutex_group:
                self._draw_mutex_route_overlay(painter, sx, sy, ex, ey, mutex_group, selected)

            arrow_c = (
                COL_MAP_MUTEX_EDGE if mutex_group
                else visited_color if is_visited
                else edge_color if released
                else COL_HORIZON_EDGE
            )
            self._draw_arrow_head(painter, sx, sy, ex, ey, arrow_c, selected)

    def _mutex_group_for_order_edge(self, edge: Dict[str, Any], start_id: str, end_id: str) -> str:
        for key in ("mutex", "mutexGroup", "mutex_group", "mutexGroups"):
            value = edge.get(key)
            if isinstance(value, (list, tuple)) and value:
                value = value[0]
            if value:
                return str(value)
        if self._map_data and self._map_data.loaded:
            return self._map_data.mutex_for_lane(start_id, end_id)
        return ""

    def _draw_mutex_route_overlay(
        self,
        painter: QPainter,
        sx: float,
        sy: float,
        ex: float,
        ey: float,
        mutex_group: str,
        selected: bool = False,
    ):
        width = 5 if selected else 4
        halo_pen = QPen(QColor(255, 255, 255, 230), width + 3, Qt.SolidLine)
        halo_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(halo_pen)
        painter.drawLine(QPointF(sx, sy), QPointF(ex, ey))

        overlay = QColor(COL_MAP_MUTEX_EDGE)
        overlay.setAlpha(235)
        pen = QPen(overlay, width, Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(QPointF(sx, sy), QPointF(ex, ey))

        self._draw_mutex_edge_tick(painter, sx, sy, ex, ey)

        if self._scale < 10 or math.hypot(ex - sx, ey - sy) < 32:
            return

        font = QFont("sans-serif", 7, QFont.Bold)
        metrics = QFontMetrics(font)
        label = metrics.elidedText(mutex_group, Qt.ElideRight, 120)
        width = metrics.horizontalAdvance(label) + 10
        height = metrics.height() + 3
        mx = (sx + ex) / 2.0
        my = (sy + ey) / 2.0
        rect = QRectF(mx - width / 2, my + 7, width, height)

        painter.setPen(QPen(COL_MAP_MUTEX_EDGE.darker(130), 1))
        painter.setBrush(QBrush(QColor(255, 255, 255, 235)))
        painter.drawRoundedRect(rect, 3, 3)
        painter.setFont(font)
        painter.setPen(QPen(COL_MAP_MUTEX_EDGE.darker(120)))
        painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_arrow_head(self, painter, sx, sy, ex, ey, color, selected=False):
        dx = ex - sx
        dy = ey - sy
        length = math.hypot(dx, dy)
        if length < 1:
            return
        dx /= length
        dy /= length

        mx = sx + dx * length * 0.7
        my = sy + dy * length * 0.7
        al, aw = (12, 6) if selected else (8, 4)

        p1 = QPointF(mx + dx * al, my + dy * al)
        p2 = QPointF(mx - dy * aw, my + dx * aw)
        p3 = QPointF(mx + dy * aw, my - dx * aw)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([p1, p2, p3]))

    def _draw_trail(self, painter: QPainter, trail: List[Tuple[float, float]], agv_color: QColor, selected: bool = False):
        if len(trail) < 2:
            return
        trail_color = QColor(agv_color)
        trail_color.setAlpha(160 if selected else 100)
        pen = QPen(trail_color, TRAIL_WIDTH * (2.5 if selected else 1), Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)

        path = QPainterPath()
        sx, sy = self._world_to_screen(*trail[0])
        path.moveTo(sx, sy)
        for wx, wy in trail[1:]:
            sx, sy = self._world_to_screen(wx, wy)
            path.lineTo(sx, sy)
        painter.drawPath(path)

    def _draw_nodes(self, painter: QPainter, snap: Snapshot, node_pos: Dict, agv_color: QColor, selected: bool = False):
        font = QFont("sans-serif", 8, QFont.Bold)
        painter.setFont(font)

        remaining_ids: Set[str] = set()
        for ns in snap.node_states:
            remaining_ids.add(ns.get("nodeId", ""))

        last_nid = snap.last_node_id or ""
        last_seq = snap.last_node_seq_id

        visited_fill = QColor(agv_color.red(), agv_color.green(), agv_color.blue(), 50)
        visited_pen_c = QColor(agv_color.red(), agv_color.green(), agv_color.blue(), 100)

        r_extra = 3 if selected else 0
        w_mult = 2.0 if selected else 1.0

        for node in snap.nodes:
            nid = node.get("nodeId", "")
            released = node.get("released", False)
            seq = node.get("sequenceId", -1)
            pos = node.get("nodePosition", {})
            x, y = pos.get("x"), pos.get("y")
            if x is None or y is None:
                continue

            sx, sy = self._world_to_screen(x, y)
            # Only store if not already stored (avoid overwrite from another AGV)
            if nid not in self._node_screen_positions:
                self._node_screen_positions[nid] = (sx, sy, node)

            r = NODE_RADIUS + r_extra
            is_current = (nid == last_nid) and last_nid != ""
            is_visited = (not is_current and nid not in remaining_ids
                          and last_seq >= 0 and seq <= last_seq)

            if is_current:
                painter.setPen(QPen(agv_color, 3 * w_mult))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QPointF(sx, sy), r + 4, r + 4)
            elif is_visited:
                painter.setPen(QPen(visited_pen_c, 1.5 * w_mult))
                painter.setBrush(QBrush(visited_fill))
                painter.drawEllipse(QPointF(sx, sy), r, r)
            elif released:
                painter.setPen(QPen(agv_color.darker(120), 1.5 * w_mult))
                painter.setBrush(QBrush(agv_color))
                painter.drawEllipse(QPointF(sx, sy), r, r)
            else:
                painter.setPen(QPen(COL_HORIZON_NODE.darker(130), 1.5 * w_mult, Qt.DashLine))
                painter.setBrush(QBrush(QColor(255, 255, 255, 200)))
                painter.drawEllipse(QPointF(sx, sy), r, r)

            painter.setPen(QPen(COL_NODE_TEXT))
            prefix = (
                self._map_data.label_prefix
                if self._map_data and self._map_data.loaded
                else ""
            )
            label = _display_node_label(nid, prefix)
            painter.drawText(QRectF(sx - 40, sy - r - 16, 80, 14), Qt.AlignCenter, label)

    def _draw_robot(self, painter: QPainter, snap: Snapshot, agv_id: str, agv_color: QColor, selected: bool):
        if snap.agv_x is None or snap.agv_y is None:
            return

        sx, sy = self._world_to_screen(snap.agv_x, snap.agv_y)
        self._robot_screen_positions[agv_id] = (sx, sy)
        theta = snap.agv_theta or 0.0
        s = ROBOT_SIZE

        painter.save()
        painter.translate(sx, sy)
        painter.rotate(-math.degrees(theta))

        tri = QPolygonF([
            QPointF(s, 0),
            QPointF(-s * 0.6, s * 0.7),
            QPointF(-s * 0.6, -s * 0.7),
        ])

        # Selection ring
        if selected:
            painter.setPen(QPen(QColor(255, 255, 255), 6))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(tri)
            painter.setPen(QPen(agv_color, 3))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(tri)
        else:
            # White halo
            painter.setPen(QPen(QColor(255, 255, 255), 4))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(tri)

        painter.setPen(QPen(agv_color.darker(130), 1.5))
        painter.setBrush(QBrush(agv_color))
        painter.drawPolygon(tri)
        painter.restore()

        # AGV ID label below robot
        painter.setPen(QPen(agv_color.darker(150)))
        label_font = QFont("sans-serif", 7, QFont.Bold)
        painter.setFont(label_font)
        painter.drawText(QRectF(sx - 50, sy + s + 2, 100, 14), Qt.AlignCenter, agv_id)

    def _draw_legend(self, painter: QPainter):
        """Draw small color legend in top-left corner."""
        visible_ids = sorted(self._visible_agvs)
        if not visible_ids:
            return

        x0, y0 = 10, 10
        line_h = 16
        painter.setFont(QFont("sans-serif", 8))

        for i, aid in enumerate(visible_ids):
            color = self.get_agv_color(aid)
            y = y0 + i * line_h

            # Color swatch
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(color))
            painter.drawRect(int(x0), int(y + 2), 10, 10)

            # Label
            painter.setPen(QPen(COL_NODE_TEXT))
            painter.drawText(int(x0 + 14), int(y + 12), aid)

    # ---- Mouse events ----

    def wheelEvent(self, event: QWheelEvent):
        pos = event.pos()
        old_wx, old_wy = self._screen_to_world(pos.x(), pos.y())
        delta = event.angleDelta().y()
        factor = 1.15 if delta > 0 else 1 / 1.15
        self._scale = max(1.0, min(self._scale * factor, 500.0))
        new_sx, new_sy = self._world_to_screen(old_wx, old_wy)
        self._offset_x += pos.x() - new_sx
        self._offset_y += pos.y() - new_sy
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton or (
            event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier
        ):
            self._panning = True
            self._pan_start = event.pos()
            self._pan_offset_start = (self._offset_x, self._offset_y)
            self.setCursor(Qt.ClosedHandCursor)
        elif event.button() == Qt.LeftButton:
            # Check robot click first
            for aid, (rx, ry) in self._robot_screen_positions.items():
                dx = event.pos().x() - rx
                dy = event.pos().y() - ry
                if math.hypot(dx, dy) <= ROBOT_SIZE + 4:
                    self.robot_clicked.emit(aid)
                    return
            # Then node click
            self._check_node_click(event.pos())

    def mouseMoveEvent(self, event):
        if self._panning:
            dx = event.pos().x() - self._pan_start.x()
            dy = event.pos().y() - self._pan_start.y()
            self._offset_x = self._pan_offset_start[0] + dx
            self._offset_y = self._pan_offset_start[1] + dy
            self.update()

    def mouseReleaseEvent(self, event):
        if self._panning:
            self._panning = False
            self.setCursor(Qt.ArrowCursor)

    def _check_node_click(self, pos):
        threshold = NODE_RADIUS + 6
        for nid, (sx, sy, node_data) in self._node_screen_positions.items():
            dx = pos.x() - sx
            dy = pos.y() - sy
            if math.hypot(dx, dy) <= threshold:
                self.node_clicked.emit(nid, node_data)
                return


# ---------------------------------------------------------------------------
# InfoPanel – state information display with AGV selector
# ---------------------------------------------------------------------------
class InfoPanel(QWidget):
    agv_selected = pyqtSignal(str)  # agv_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(280)
        self.setMaximumWidth(400)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # AGV selector row
        selector_layout = QHBoxLayout()
        selector_layout.addWidget(QLabel("AGV:"))
        self._agv_combo = QComboBox()
        self._agv_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._agv_combo.currentTextChanged.connect(self._on_agv_combo_changed)
        selector_layout.addWidget(self._agv_combo)
        layout.addLayout(selector_layout)

        # Tab widget: Summary + Raw messages
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(
            "QTabWidget::pane { border: 1px solid #DDD; }"
            "QTabBar::tab { padding: 4px 12px; font-size: 11px; }"
        )

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont("Consolas, Courier", 10))
        self._text.setStyleSheet(
            "QTextEdit { background: #FFFFFF; border: none; }"
        )
        self._tabs.addTab(self._text, "Summary")

        self._raw_tabs = QTabWidget()
        self._raw_tabs.setStyleSheet(
            "QTabWidget::pane { border: none; }"
            "QTabBar::tab { padding: 4px 10px; font-size: 11px; }"
        )
        self._order_raw_text = self._create_raw_text()
        self._instant_raw_text = self._create_raw_text()
        self._raw_tabs.addTab(self._order_raw_text, "Order")
        self._raw_tabs.addTab(self._instant_raw_text, "InstantActions")
        self._tabs.addTab(self._raw_tabs, "Raw JSON")

        layout.addWidget(self._tabs)

        self._current_agvs: List[str] = []
        self._text_scrollbar_pressed = False
        self._deferred_text_updates: Dict[int, Tuple[QTextEdit, Any, str]] = {}

        for text_edit in (self._text, self._order_raw_text, self._instant_raw_text):
            self._track_text_scrollbars(text_edit)

    def _create_raw_text(self) -> QTextEdit:
        text = QTextEdit()
        text.setReadOnly(True)
        text.setFont(QFont("Consolas, Courier", 9))
        text.setStyleSheet(
            "QTextEdit { background: #FAFAFA; border: none; }"
        )
        text.setLineWrapMode(QTextEdit.NoWrap)
        return text

    def _track_text_scrollbars(self, text_edit: QTextEdit):
        for scrollbar in (text_edit.verticalScrollBar(), text_edit.horizontalScrollBar()):
            scrollbar.installEventFilter(self)
            scrollbar.sliderPressed.connect(self._on_text_scrollbar_pressed)
            scrollbar.sliderReleased.connect(self._on_text_scrollbar_released)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.MouseButtonPress:
            self._on_text_scrollbar_pressed()
        elif event.type() == QEvent.MouseButtonRelease:
            self._on_text_scrollbar_released()
        return super().eventFilter(obj, event)

    def _on_text_scrollbar_pressed(self):
        self._text_scrollbar_pressed = True

    def _on_text_scrollbar_released(self):
        if not self._text_scrollbar_pressed:
            return
        self._text_scrollbar_pressed = False
        self._apply_deferred_text_updates()

    def _apply_deferred_text_updates(self):
        if not self._deferred_text_updates:
            return

        updates = list(self._deferred_text_updates.values())
        self._deferred_text_updates.clear()
        for text_edit, update_func, value in updates:
            self._update_text_preserving_scroll(text_edit, update_func, value)

    def _update_text_preserving_scroll(self, text_edit: QTextEdit, update_func, value: str):
        if self._text_scrollbar_pressed:
            self._deferred_text_updates[id(text_edit)] = (text_edit, update_func, value)
            return

        vbar = text_edit.verticalScrollBar()
        hbar = text_edit.horizontalScrollBar()
        old_v = vbar.value()
        old_h = hbar.value()
        was_at_bottom = vbar.maximum() > 0 and old_v >= vbar.maximum() - 2
        was_at_right = hbar.maximum() > 0 and old_h >= hbar.maximum() - 2

        update_func(value)

        vbar.setValue(vbar.maximum() if was_at_bottom else min(old_v, vbar.maximum()))
        hbar.setValue(hbar.maximum() if was_at_right else min(old_h, hbar.maximum()))

    def update_agv_list(self, agv_ids: List[str], selected: str = ""):
        """Update the AGV combo box items."""
        if agv_ids == self._current_agvs:
            # Just update selection if list is same
            if selected and self._agv_combo.currentText() != selected:
                idx = self._agv_combo.findText(selected)
                if idx >= 0:
                    self._agv_combo.setCurrentIndex(idx)
            return

        self._current_agvs = list(agv_ids)
        self._agv_combo.blockSignals(True)
        self._agv_combo.clear()
        for aid in agv_ids:
            self._agv_combo.addItem(aid)
        if selected and selected in agv_ids:
            self._agv_combo.setCurrentText(selected)
        elif agv_ids:
            self._agv_combo.setCurrentIndex(0)
        self._agv_combo.blockSignals(False)

    def set_snapshot(self, snap: Snapshot, agv_id: str = "", timestamp: str = "", agv_color: QColor = None):
        parts = []

        # Header
        color_hex = agv_color.name() if agv_color else "#333"
        if agv_id:
            parts.append(f'<b style="color:{color_hex}">AGV: {agv_id}</b>')
        if timestamp:
            parts.append(f"<b>Time:</b> {timestamp}")

        # Connection
        cs = snap.connection_state
        color_map = {"ONLINE": "#4CAF50", "OFFLINE": "#FFC107", "CONNECTIONBROKEN": "#F44336"}
        c_color = color_map.get(cs, "#999")
        parts.append(f'<b>Connection:</b> <span style="color:{c_color};font-weight:bold">{cs}</span>')

        parts.append("<hr>")

        # Order info
        if snap.order:
            parts.append("<b>Order</b>")
            parts.append(f"  orderId: {snap.order_id}")
            parts.append(f"  orderUpdateId: {snap.order_update_id}")
            n_base = sum(1 for n in snap.nodes if n.get("released"))
            n_horizon = sum(1 for n in snap.nodes if not n.get("released"))
            parts.append(f"  nodes: {len(snap.nodes)} (base={n_base}, horizon={n_horizon})")
            parts.append(f"  edges: {len(snap.edges)}")
        else:
            parts.append("<b>Order:</b> <i>none</i>")

        parts.append("<hr>")

        # AGV state
        if snap.state:
            parts.append("<b>AGV State</b>")
            parts.append(f"  driving: {snap.driving}")
            parts.append(f"  lastNodeId: {snap.last_node_id}")
            if snap.agv_x is not None:
                parts.append(f"  position: ({snap.agv_x:.3f}, {snap.agv_y:.3f})")
                if snap.agv_theta is not None:
                    parts.append(f"  theta: {snap.agv_theta:.3f} rad ({math.degrees(snap.agv_theta):.1f}°)")
            parts.append(f"  battery: {snap.battery_charge:.1f}%")
            op_mode = snap.state.get("operatingMode", "?")
            parts.append(f"  operatingMode: {op_mode}")
            parts.append(f"  paused: {snap.state.get('paused', False)}")
            parts.append(f"  newBaseRequest: {snap.state.get('newBaseRequest', False)}")
        else:
            parts.append("<b>AGV State:</b> <i>none</i>")

        parts.append("<hr>")

        # Node States
        if snap.node_states:
            parts.append(f"<b>Remaining Nodes ({len(snap.node_states)})</b>")
            for ns in snap.node_states:
                rel = "base" if ns.get("released") else "horizon"
                parts.append(f"  {ns.get('nodeId','')} (seq={ns.get('sequenceId','')}, {rel})")

        # Edge States
        if snap.edge_states:
            parts.append(f"<b>Remaining Edges ({len(snap.edge_states)})</b>")
            for es in snap.edge_states:
                rel = "base" if es.get("released") else "horizon"
                parts.append(f"  {es.get('edgeId','')} (seq={es.get('sequenceId','')}, {rel})")

        parts.append("<hr>")

        # Action States
        if snap.action_states:
            parts.append(f"<b>Action States ({len(snap.action_states)})</b>")
            status_colors = {
                "WAITING": "#9E9E9E", "INITIALIZING": "#2196F3",
                "RUNNING": "#4CAF50", "PAUSED": "#FFC107",
                "FINISHED": "#8BC34A", "FAILED": "#F44336",
            }
            for a in snap.action_states:
                st = a.get("actionStatus", "?")
                sc = status_colors.get(st, "#999")
                parts.append(
                    f'  {a.get("actionType","?")} [{a.get("actionId","")}]: '
                    f'<span style="color:{sc}">{st}</span>'
                )

        # Errors
        if snap.errors:
            parts.append("<hr>")
            parts.append(f'<b style="color:#F44336">Errors ({len(snap.errors)})</b>')
            for err in snap.errors:
                lvl = err.get("errorLevel", "?")
                parts.append(f'  [{lvl}] {err.get("errorType","?")} - {err.get("errorDescription","")}')

        html = "<pre style='font-family: Consolas, monospace; font-size: 10pt; white-space: pre-wrap;'>"
        html += "\n".join(parts)
        html += "</pre>"
        self._update_text_preserving_scroll(self._text, self._text.setHtml, html)

        # Raw JSON tab – Order and InstantActions are split into sub-tabs.
        if snap.order:
            self._update_text_preserving_scroll(
                self._order_raw_text,
                self._order_raw_text.setPlainText,
                json.dumps(snap.order, indent=2, ensure_ascii=False)
            )
        else:
            self._update_text_preserving_scroll(
                self._order_raw_text,
                self._order_raw_text.setPlainText,
                "No Order message available",
            )

        if snap.instant_actions:
            self._update_text_preserving_scroll(
                self._instant_raw_text,
                self._instant_raw_text.setPlainText,
                json.dumps(snap.instant_actions, indent=2, ensure_ascii=False)
            )
        else:
            self._update_text_preserving_scroll(
                self._instant_raw_text,
                self._instant_raw_text.setPlainText,
                "No InstantActions message available",
            )

    def _on_agv_combo_changed(self, text):
        if text:
            self.agv_selected.emit(text)


# ---------------------------------------------------------------------------
# TimelineWidget – slider + playback controls
# ---------------------------------------------------------------------------
class TimelineWidget(QWidget):
    index_changed = pyqtSignal(int)
    slider_released = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(80)
        self._max_index = 0
        self._playing = False
        self._speed = 1.0
        self._order_markers: List[int] = []
        self._slider_pressed = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(0)
        self._slider.valueChanged.connect(self._on_slider_changed)
        self._slider.sliderPressed.connect(self._on_slider_pressed)
        self._slider.sliderReleased.connect(self._on_slider_released)
        layout.addWidget(self._slider)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.setSpacing(8)

        self._btn_first = QPushButton("|◀")
        self._btn_prev = QPushButton("◀")
        self._btn_play = QPushButton("▶ Play")
        self._btn_next = QPushButton("▶")
        self._btn_last = QPushButton("▶|")

        for btn in (self._btn_first, self._btn_prev, self._btn_next, self._btn_last):
            btn.setFixedWidth(40)
            btn.setStyleSheet("font-size: 12px;")

        self._btn_play.setFixedWidth(80)
        self._btn_play.setStyleSheet("font-size: 12px; font-weight: bold;")

        self._btn_first.clicked.connect(self.go_first)
        self._btn_prev.clicked.connect(self.go_prev)
        self._btn_play.clicked.connect(self.toggle_play)
        self._btn_next.clicked.connect(self.go_next)
        self._btn_last.clicked.connect(self.go_last)

        ctrl_layout.addWidget(self._btn_first)
        ctrl_layout.addWidget(self._btn_prev)
        ctrl_layout.addWidget(self._btn_play)
        ctrl_layout.addWidget(self._btn_next)
        ctrl_layout.addWidget(self._btn_last)

        ctrl_layout.addSpacing(16)

        self._step_label = QLabel("0 / 0")
        self._step_label.setMinimumWidth(120)
        ctrl_layout.addWidget(self._step_label)

        self._time_label = QLabel("")
        self._time_label.setMinimumWidth(200)
        ctrl_layout.addWidget(self._time_label)

        ctrl_layout.addStretch()

        ctrl_layout.addWidget(QLabel("Speed:"))
        self._speed_combo = QComboBox()
        self._speed_combo.addItems(["0.25x", "0.5x", "1x", "2x", "4x", "10x"])
        self._speed_combo.setCurrentIndex(2)
        self._speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        ctrl_layout.addWidget(self._speed_combo)

        layout.addLayout(ctrl_layout)

        self._play_timer = QTimer(self)
        self._play_timer.timeout.connect(self._play_step)
        self._play_interval_ms = 500

    def set_range(self, max_index: int):
        self._max_index = max_index
        self._slider.setMaximum(max_index)
        self._update_label()

    def set_order_markers(self, indices: List[int]):
        self._order_markers = indices

    def set_timestamp(self, ts: str):
        self._time_label.setText(ts)

    @property
    def current_index(self) -> int:
        return self._slider.value()

    @property
    def is_user_scrubbing(self) -> bool:
        return self._slider_pressed

    def set_index(self, idx: int):
        self._slider.setValue(idx)

    def go_first(self):
        self._slider.setValue(0)

    def go_prev(self):
        self._slider.setValue(max(0, self._slider.value() - 1))

    def go_next(self):
        self._slider.setValue(min(self._max_index, self._slider.value() + 1))

    def go_last(self):
        self._slider.setValue(self._max_index)

    def toggle_play(self):
        self._playing = not self._playing
        if self._playing:
            self._btn_play.setText("⏸ Pause")
            self._play_timer.start(self._play_interval_ms)
        else:
            self._btn_play.setText("▶ Play")
            self._play_timer.stop()

    def stop_play(self):
        self._playing = False
        self._btn_play.setText("▶ Play")
        self._play_timer.stop()

    def _play_step(self):
        if self._slider.value() >= self._max_index:
            self.stop_play()
            return
        self._slider.setValue(self._slider.value() + 1)

    def _on_slider_changed(self, value):
        self._update_label()
        self.index_changed.emit(value)

    def _on_slider_pressed(self):
        self._slider_pressed = True

    def _on_slider_released(self):
        self._slider_pressed = False
        self.slider_released.emit()

    def _on_speed_changed(self, idx):
        speeds = [0.25, 0.5, 1.0, 2.0, 4.0, 10.0]
        if 0 <= idx < len(speeds):
            self._speed = speeds[idx]
            self._play_interval_ms = max(10, int(500 / self._speed))
            if self._playing:
                self._play_timer.start(self._play_interval_ms)

    def _update_label(self):
        self._step_label.setText(f"{self._slider.value()} / {self._max_index}")

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self._order_markers or self._max_index <= 0:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        slider_geom = self._slider.geometry()
        margin = 8
        x_start = slider_geom.x() + margin
        x_end = slider_geom.x() + slider_geom.width() - margin
        y_top = slider_geom.y()

        total_range = x_end - x_start
        if total_range <= 0:
            return

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(COL_ORDER_MARKER))

        for idx in self._order_markers:
            frac = idx / self._max_index if self._max_index > 0 else 0
            x = x_start + frac * total_range
            tri = QPolygonF([
                QPointF(x, y_top),
                QPointF(x - 4, y_top - 8),
                QPointF(x + 4, y_top - 8),
            ])
            painter.drawPolygon(tri)

        painter.end()


# ---------------------------------------------------------------------------
# SimulationController – embeds vda5050_robot_simulator without copying it
# ---------------------------------------------------------------------------
class SimulationController(QObject):
    status_changed = pyqtSignal(object)
    message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._runtime_module = None
        self._config: Dict[str, Any] = {}
        self._robot_configs: Dict[str, Any] = {}
        self._simulators: Dict[str, Any] = {}
        self._stop_events: Dict[str, asyncio.Event] = {}
        self._tasks: Dict[str, asyncio.Task] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._loop_ready = threading.Event()
        self._command_queue: queue.Queue = queue.Queue()
        self._lock = threading.RLock()
        self._configured_loggers: Set[str] = set()

    def _load_runtime_module(self):
        if self._runtime_module is not None:
            return self._runtime_module

        run_path = SIMULATOR_ROOT / "run.py"
        if not run_path.exists():
            raise FileNotFoundError(f"Simulator entrypoint not found: {run_path}")

        sim_root = str(SIMULATOR_ROOT)
        if sim_root not in sys.path:
            sys.path.insert(0, sim_root)

        spec = importlib.util.spec_from_file_location(
            "_vda5050_robot_simulator_runtime",
            str(run_path),
        )
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load simulator runtime: {run_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._runtime_module = module
        return module

    def configure(self, config: Dict[str, Any]):
        if self.has_running_robots():
            raise RuntimeError("Stop running robots before applying config changes.")

        runtime = self._load_runtime_module()
        mqtt_cfg = copy.deepcopy(config.get("mqtt", {}))
        pub_cfg = copy.deepcopy(config.get("publishing", {}))
        defaults = copy.deepcopy(config.get("robot_defaults", {}))
        robots = copy.deepcopy(config.get("robots", []))
        download_map_cfg = copy.deepcopy(config.get("download_map"))
        action_results_cfg = copy.deepcopy(config.get("action_results", {}))

        simulators: Dict[str, Any] = {}
        robot_configs: Dict[str, Any] = {}
        if not robots:
            with self._lock:
                self._config = copy.deepcopy(config)
                self._robot_configs = {}
                self._simulators = {}
                self._stop_events.clear()
                self._tasks.clear()
            self.message.emit("Configured 0 simulator robot(s).")
            self.status_changed.emit(self.get_statuses())
            return

        for robot_entry in robots:
            serial = str(robot_entry.get("serial_number", "")).strip()
            if not serial:
                raise ValueError("Every robot needs a serial_number.")
            if serial in simulators:
                raise ValueError(f"Duplicate serial_number: {serial}")

            robot_config = runtime._build_robot_config(
                mqtt_cfg,
                pub_cfg,
                defaults,
                robot_entry,
            )
            if download_map_cfg:
                robot_config["download_map"] = download_map_cfg
            if action_results_cfg:
                robot_config["action_results"] = action_results_cfg
            robot_configs[serial] = robot_config
            simulators[serial] = self._create_simulator(serial, robot_config)

        self._refresh_peer_robots(simulators)

        with self._lock:
            self._config = copy.deepcopy(config)
            self._robot_configs = robot_configs
            self._simulators = simulators
            self._stop_events.clear()
            self._tasks.clear()

        self.message.emit(f"Configured {len(simulators)} simulator robot(s).")
        self.status_changed.emit(self.get_statuses())


    def has_running_robots(self) -> bool:
        with self._lock:
            return any(task and not task.done() for task in self._tasks.values())

    def _create_simulator(self, serial: str, robot_config: Dict[str, Any]):
        runtime = self._load_runtime_module()
        if serial not in self._configured_loggers:
            runtime._setup_robot_file_handler(serial)
            self._configured_loggers.add(serial)
        return runtime.Simulator(copy.deepcopy(robot_config))

    @staticmethod
    def _refresh_peer_robots(simulators: Dict[str, Any]):
        all_robots = [sim.robot for sim in simulators.values()]
        for sim in simulators.values():
            sim.set_peer_robots(all_robots)

    def _ensure_loop(self):
        with self._lock:
            if self._loop and self._loop.is_running():
                return

            self._loop_ready.clear()
            self._thread = threading.Thread(
                target=self._loop_thread_main,
                name="vda5050-simulation-loop",
                daemon=True,
            )
            self._thread.start()

        if not self._loop_ready.wait(timeout=5.0):
            raise RuntimeError("Simulation asyncio loop did not start.")

    def _loop_thread_main(self):
        try:
            asyncio.run(self._async_loop_main())
        finally:
            with self._lock:
                self._loop = None

    async def _async_loop_main(self):
        loop = asyncio.get_running_loop()
        with self._lock:
            self._loop = loop
        self._loop_ready.set()

        while True:
            try:
                name, args = self._command_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.05)
                continue
            try:
                if name == "start_all":
                    await self._start_all_async()
                elif name == "start_robot":
                    await self._start_robot_async(*args)
                elif name == "stop_all":
                    await self._stop_all_async()
                elif name == "stop_robot":
                    await self._stop_robot_async(*args)
                elif name == "robot_command":
                    await self._apply_robot_command(*args)
                elif name == "shutdown":
                    await self._stop_all_async()
                    break
            except Exception as exc:
                self.error_occurred.emit(str(exc))

    def _enqueue(self, name: str, *args):
        self._ensure_loop()
        self._command_queue.put((name, args))

    def start_all(self):
        self._enqueue("start_all")

    def start_robot(self, serial: str):
        self._enqueue("start_robot", serial)

    def stop_all(self):
        if self._thread and self._thread.is_alive():
            self._command_queue.put(("stop_all", ()))

    def stop_robot(self, serial: str):
        if self._thread and self._thread.is_alive():
            self._command_queue.put(("stop_robot", (serial,)))

    async def _start_all_async(self):
        with self._lock:
            serials = list(self._robot_configs.keys())
        for serial in serials:
            await self._start_robot_async(serial)

    async def _start_robot_async(self, serial: str):
        with self._lock:
            existing = self._tasks.get(serial)
            if existing and not existing.done():
                return
            robot_config = self._robot_configs.get(serial)
        if robot_config is None:
            raise ValueError(f"Unknown robot: {serial}")

        sim = self._create_simulator(serial, robot_config)
        with self._lock:
            self._simulators[serial] = sim
            self._refresh_peer_robots(self._simulators)

        stop_event = asyncio.Event()
        task = asyncio.create_task(self._run_simulator(serial, sim, stop_event))
        with self._lock:
            self._stop_events[serial] = stop_event
            self._tasks[serial] = task
        self.message.emit(f"{serial} started.")
        self.status_changed.emit(self.get_statuses())

    async def _run_simulator(self, serial: str, sim, stop_event: asyncio.Event):
        try:
            await sim.run(stop_event)
        except Exception as exc:
            logging.getLogger(__name__).exception("Simulator failed: %s", serial)
            self.error_occurred.emit(f"{serial}: {exc}")
        finally:
            with self._lock:
                current = self._tasks.get(serial)
                if current is asyncio.current_task():
                    self._tasks.pop(serial, None)
                    self._stop_events.pop(serial, None)
            self.message.emit(f"{serial} stopped.")
            self.status_changed.emit(self.get_statuses())

    async def _stop_all_async(self):
        with self._lock:
            serials = list(self._tasks.keys())
        await asyncio.gather(
            *(self._stop_robot_async(serial) for serial in serials),
            return_exceptions=True,
        )

    async def _stop_robot_async(self, serial: str):
        with self._lock:
            task = self._tasks.get(serial)
            stop_event = self._stop_events.get(serial)
        if stop_event:
            stop_event.set()
        if task and not task.done():
            try:
                await asyncio.wait_for(asyncio.shield(task), timeout=10.0)
            except asyncio.TimeoutError:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
        else:
            with self._lock:
                self._tasks.pop(serial, None)
                self._stop_events.pop(serial, None)
            self.status_changed.emit(self.get_statuses())

    def set_obstacle_stop(self, serial: str, enabled: bool):
        self._enqueue("robot_command",
            serial,
            lambda sim: sim.robot.set_manual_obstacle_stop(enabled),
            f"{serial} obstacle stop {'enabled' if enabled else 'released'}.",
        )

    def set_battery_charge(self, serial: str, charge: float):
        self._enqueue("robot_command",
            serial,
            lambda sim: sim.robot.set_battery_charge(charge),
            f"{serial} battery set to {charge:.1f}%.",
        )

    def raise_error(
        self,
        serial: str,
        error_type: str,
        level: str,
        description: str,
    ):
        self._enqueue("robot_command",
            serial,
            lambda sim: sim.robot.raise_error(error_type, description, level),
            f"{serial} error raised: {error_type}",
        )

    def clear_errors(self, serial: str):
        self._enqueue("robot_command",
            serial,
            lambda sim: sim.robot.clear_errors(),
            f"{serial} errors cleared.",
        )

    async def _apply_robot_command(self, serial: str, command, message: str):
        with self._lock:
            sim = self._simulators.get(serial)
            task = self._tasks.get(serial)
            running = bool(task and not task.done())
        if sim is None:
            raise ValueError(f"Unknown robot: {serial}")

        command(sim)
        if running:
            self._publish_state_now(sim)
        self.message.emit(message)
        self.status_changed.emit(self.get_statuses())

    @staticmethod
    def _publish_state_now(sim):
        try:
            sim._state_publisher.publish_state_now()
        except Exception:
            logging.getLogger(__name__).debug(
                "Immediate state publish skipped", exc_info=True
            )

    def get_statuses(self) -> List[Dict[str, Any]]:
        with self._lock:
            items = list(self._simulators.items())
            tasks = dict(self._tasks)

        statuses: List[Dict[str, Any]] = []
        for serial, sim in items:
            task = tasks.get(serial)
            running = bool(task and not task.done())
            robot = sim.robot
            battery = robot.get_battery_state().batteryCharge
            pos = robot.get_agv_position()
            statuses.append({
                "serial": serial,
                "running": running,
                "battery": battery,
                "driving": robot.driving,
                "paused": robot.paused,
                "field_violation": robot.safety_state.fieldViolation,
                "manual_obstacle": bool(getattr(robot, "_manual_obstacle_stop", False)),
                "errors": len(robot.errors),
                "x": pos.x,
                "y": pos.y,
                "theta": pos.theta,
                "map_id": pos.mapId,
            })
        return statuses

    def shutdown(self):
        with self._lock:
            thread = self._thread

        if thread and thread.is_alive():
            self._command_queue.put(("shutdown", ()))
            thread.join(timeout=3.0)
            if thread.is_alive():
                logging.getLogger(__name__).warning(
                    "Timed out while stopping simulation thread"
                )


# ---------------------------------------------------------------------------
# SimulationTab – config editor and simulator command surface
# ---------------------------------------------------------------------------
class SimulationTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._controller = SimulationController(self)
        self._config: Dict[str, Any] = {}
        self._dirty = False
        self._loading_form = False

        self._setup_ui()
        self._connect_signals()

        if DEFAULT_SIM_CONFIG_PATH.exists():
            self._config_path_edit.setText(str(DEFAULT_SIM_CONFIG_PATH))
            self._load_config(str(DEFAULT_SIM_CONFIG_PATH))

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(500)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        config_group = QGroupBox("Config")
        config_layout = QHBoxLayout(config_group)
        self._config_path_edit = QLineEdit()
        self._config_path_edit.setText(str(DEFAULT_SIM_CONFIG_PATH))
        self._btn_browse_config = QPushButton("Browse")
        self._btn_load_config = QPushButton("Load")
        self._btn_save_config = QPushButton("Save")
        config_layout.addWidget(QLabel("YAML:"))
        config_layout.addWidget(self._config_path_edit, stretch=1)
        config_layout.addWidget(self._btn_browse_config)
        config_layout.addWidget(self._btn_load_config)
        config_layout.addWidget(self._btn_save_config)
        layout.addWidget(config_group)

        controls = QHBoxLayout()
        self._btn_start_all = QPushButton("Start All")
        self._btn_stop_all = QPushButton("Stop All")
        self._status_label = QLabel("Not configured")
        self._status_label.setStyleSheet("color: #555;")
        controls.addWidget(self._btn_start_all)
        controls.addWidget(self._btn_stop_all)
        controls.addStretch()
        controls.addWidget(self._status_label)
        layout.addLayout(controls)

        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 4, 0)
        left_layout.setSpacing(6)

        self._robot_table = QTableWidget(0, 7)
        self._robot_table.setHorizontalHeaderLabels([
            "Robot", "Status", "Battery", "Driving", "Position", "Errors", "Obstacle",
        ])
        self._robot_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._robot_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._robot_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._robot_table.verticalHeader().setVisible(False)
        self._robot_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._robot_table.horizontalHeader().setStretchLastSection(True)
        left_layout.addWidget(self._robot_table)

        robot_buttons = QHBoxLayout()
        self._btn_add_robot = QPushButton("Add")
        self._btn_remove_robot = QPushButton("Remove")
        self._btn_apply_robot = QPushButton("Apply")
        self._btn_start_robot = QPushButton("Start")
        self._btn_stop_robot = QPushButton("Stop")
        for btn in (
            self._btn_add_robot,
            self._btn_remove_robot,
            self._btn_apply_robot,
            self._btn_start_robot,
            self._btn_stop_robot,
        ):
            robot_buttons.addWidget(btn)
        left_layout.addLayout(robot_buttons)
        splitter.addWidget(left)

        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 0, 0)
        right_layout.setSpacing(8)

        robot_group = QGroupBox("Robot Settings")
        form = QFormLayout(robot_group)
        self._serial_edit = QLineEdit()
        self._map_id_edit = QLineEdit()
        self._x_spin = self._make_double_spin(-100000.0, 100000.0, 3)
        self._y_spin = self._make_double_spin(-100000.0, 100000.0, 3)
        self._theta_spin = self._make_double_spin(-6.283, 6.283, 4)
        self._battery_spin = self._make_double_spin(0.0, 100.0, 1)
        self._max_speed_spin = self._make_double_spin(0.0, 20.0, 2)
        self._obstacle_enabled_check = QCheckBox("enabled")
        self._obstacle_front_spin = self._make_double_spin(0.0, 100.0, 2)
        self._obstacle_rear_spin = self._make_double_spin(0.0, 100.0, 2)
        self._obstacle_left_spin = self._make_double_spin(0.0, 100.0, 2)
        self._obstacle_right_spin = self._make_double_spin(0.0, 100.0, 2)
        form.addRow("Serial:", self._serial_edit)
        form.addRow("Map ID:", self._map_id_edit)
        form.addRow("Initial X:", self._x_spin)
        form.addRow("Initial Y:", self._y_spin)
        form.addRow("Initial Theta:", self._theta_spin)
        form.addRow("Initial Battery:", self._battery_spin)
        form.addRow("Max Speed:", self._max_speed_spin)
        form.addRow("Obstacle Detection:", self._obstacle_enabled_check)
        form.addRow("Front Distance:", self._obstacle_front_spin)
        form.addRow("Rear Distance:", self._obstacle_rear_spin)
        form.addRow("Left Distance:", self._obstacle_left_spin)
        form.addRow("Right Distance:", self._obstacle_right_spin)
        right_layout.addWidget(robot_group)

        command_group = QGroupBox("Robot Command")
        command_layout = QFormLayout(command_group)
        self._command_combo = QComboBox()
        self._command_combo.addItems([
            "Obstacle Stop",
            "Release Obstacle",
            "Set Battery",
            "Raise Error",
            "Clear Errors",
        ])
        self._command_battery_spin = self._make_double_spin(0.0, 100.0, 1)
        self._command_battery_spin.setValue(50.0)
        self._error_type_edit = QLineEdit("simulatedError")
        self._error_level_combo = QComboBox()
        self._error_level_combo.addItems(["WARNING", "FATAL"])
        self._error_desc_edit = QLineEdit("Simulated error from GUI")
        self._btn_execute_command = QPushButton("Execute")
        command_layout.addRow("Command:", self._command_combo)
        command_layout.addRow("Battery:", self._command_battery_spin)
        command_layout.addRow("Error Type:", self._error_type_edit)
        command_layout.addRow("Error Level:", self._error_level_combo)
        command_layout.addRow("Description:", self._error_desc_edit)
        command_layout.addRow("", self._btn_execute_command)
        right_layout.addWidget(command_group)

        log_group = QGroupBox("Simulation Log")
        log_layout = QVBoxLayout(log_group)
        self._sim_log = QTextEdit()
        self._sim_log.setReadOnly(True)
        self._sim_log.setMaximumHeight(160)
        self._sim_log.setFont(QFont("Consolas, Courier", 9))
        log_layout.addWidget(self._sim_log)
        right_layout.addWidget(log_group)
        right_layout.addStretch()

        right_scroll.setWidget(right)
        splitter.addWidget(right_scroll)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)

    @staticmethod
    def _make_double_spin(minimum: float, maximum: float, decimals: int) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(0.1)
        return spin

    def _connect_signals(self):
        self._btn_browse_config.clicked.connect(self._browse_config)
        self._btn_load_config.clicked.connect(lambda: self._load_config(self._config_path_edit.text()))
        self._btn_save_config.clicked.connect(self._save_config)
        self._btn_start_all.clicked.connect(self._start_all)
        self._btn_stop_all.clicked.connect(self._controller.stop_all)
        self._btn_add_robot.clicked.connect(self._add_robot)
        self._btn_remove_robot.clicked.connect(self._remove_robot)
        self._btn_apply_robot.clicked.connect(self._apply_selected_robot)
        self._btn_start_robot.clicked.connect(self._start_selected_robot)
        self._btn_stop_robot.clicked.connect(self._stop_selected_robot)
        self._btn_execute_command.clicked.connect(self._execute_command)
        self._robot_table.currentCellChanged.connect(self._on_table_selection_changed)
        self._controller.status_changed.connect(lambda _: self._refresh_status())
        self._controller.message.connect(self._append_log)
        self._controller.error_occurred.connect(self._on_controller_error)

        for widget in (
            self._serial_edit,
            self._map_id_edit,
            self._x_spin,
            self._y_spin,
            self._theta_spin,
            self._battery_spin,
            self._max_speed_spin,
            self._obstacle_enabled_check,
            self._obstacle_front_spin,
            self._obstacle_rear_spin,
            self._obstacle_left_spin,
            self._obstacle_right_spin,
        ):
            signal = getattr(widget, "textChanged", None)
            if signal is not None:
                signal.connect(self._mark_dirty)
                continue
            signal = getattr(widget, "valueChanged", None)
            if signal is not None:
                signal.connect(self._mark_dirty)
                continue
            signal = getattr(widget, "stateChanged", None)
            if signal is not None:
                signal.connect(self._mark_dirty)

    def _browse_config(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Simulator Config",
            str(DEFAULT_SIM_CONFIG_PATH),
            "YAML files (*.yaml *.yml);;All files (*)",
        )
        if path:
            self._config_path_edit.setText(path)
            self._load_config(path)

    def _load_config(self, path: str):
        path = path.strip()
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
            config.setdefault("robots", [])
            self._config = config
            self._dirty = False
            self._controller.configure(self._config)
        except Exception as exc:
            QMessageBox.critical(self, "Simulator Config", f"Failed to load config:\n{exc}")
            return

        self._render_robot_table()
        if self._config.get("robots"):
            self._robot_table.selectRow(0)
            self._load_robot_to_form(0)
        self._append_log(f"Loaded config: {path}")

    def _save_config(self):
        if self._controller.has_running_robots():
            QMessageBox.warning(self, "Simulator Config", "Stop running robots before saving config changes.")
            return
        self._apply_selected_robot(silent=True)
        path = self._config_path_edit.text().strip()
        if not path:
            path, _ = QFileDialog.getSaveFileName(
                self,
                "Save Simulator Config",
                str(DEFAULT_SIM_CONFIG_PATH),
                "YAML files (*.yaml *.yml);;All files (*)",
            )
            if not path:
                return
            self._config_path_edit.setText(path)
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(self._config, f, sort_keys=False, allow_unicode=True)
            self._dirty = False
            self._controller.configure(self._config)
            self._append_log(f"Saved config: {path}")
        except Exception as exc:
            QMessageBox.critical(self, "Simulator Config", f"Failed to save config:\n{exc}")

    def _selected_row(self) -> int:
        row = self._robot_table.currentRow()
        robots = self._config.get("robots", [])
        if row < 0 or row >= len(robots):
            return -1
        return row

    def _selected_serial(self) -> str:
        row = self._selected_row()
        if row < 0:
            return ""
        return str(self._config.get("robots", [])[row].get("serial_number", ""))

    def _on_table_selection_changed(self, current_row, current_col, previous_row, previous_col):
        if current_row >= 0:
            self._load_robot_to_form(current_row)

    def _load_robot_to_form(self, row: int):
        robots = self._config.get("robots", [])
        if row < 0 or row >= len(robots):
            return
        robot = robots[row]
        defaults = self._config.get("robot_defaults", {})
        pos = robot.get("initial_position", {})
        battery = robot.get("battery", defaults.get("battery", {}))
        obstacle = robot.get("obstacle_detection", defaults.get("obstacle_detection", {}))

        self._loading_form = True
        self._serial_edit.setText(str(robot.get("serial_number", "")))
        self._map_id_edit.setText(str(pos.get("map_id", "L1")))
        self._x_spin.setValue(float(pos.get("x", 0.0)))
        self._y_spin.setValue(float(pos.get("y", 0.0)))
        self._theta_spin.setValue(float(pos.get("theta", 0.0)))
        self._battery_spin.setValue(float(battery.get("initial_charge", 90.0)))
        self._max_speed_spin.setValue(float(robot.get("max_speed", defaults.get("max_speed", 1.5))))
        self._obstacle_enabled_check.setChecked(bool(obstacle.get("enabled", False)))
        self._obstacle_front_spin.setValue(float(obstacle.get("front_distance", 0.0)))
        self._obstacle_rear_spin.setValue(float(obstacle.get("rear_distance", 0.0)))
        self._obstacle_left_spin.setValue(float(obstacle.get("left_distance", 0.0)))
        self._obstacle_right_spin.setValue(float(obstacle.get("right_distance", 0.0)))
        self._loading_form = False

    def _mark_dirty(self, *args):
        if self._loading_form:
            return
        self._dirty = True

    def _apply_selected_robot(self, silent: bool = False):
        row = self._selected_row()
        if row < 0:
            return False
        if self._controller.has_running_robots():
            if not silent:
                QMessageBox.warning(self, "Simulation", "Stop running robots before applying config changes.")
            return False

        robots = self._config.setdefault("robots", [])
        robot = copy.deepcopy(robots[row])
        robot["serial_number"] = self._serial_edit.text().strip()
        robot["initial_position"] = {
            "x": self._x_spin.value(),
            "y": self._y_spin.value(),
            "theta": self._theta_spin.value(),
            "map_id": self._map_id_edit.text().strip() or "L1",
        }
        robot["battery"] = dict(robot.get("battery", {}))
        robot["battery"]["initial_charge"] = self._battery_spin.value()
        robot["max_speed"] = self._max_speed_spin.value()
        robot["obstacle_detection"] = {
            "enabled": self._obstacle_enabled_check.isChecked(),
            "front_distance": self._obstacle_front_spin.value(),
            "rear_distance": self._obstacle_rear_spin.value(),
            "left_distance": self._obstacle_left_spin.value(),
            "right_distance": self._obstacle_right_spin.value(),
        }
        robots[row] = robot

        try:
            self._controller.configure(self._config)
        except Exception as exc:
            QMessageBox.critical(self, "Simulation", f"Failed to apply config:\n{exc}")
            return False

        self._dirty = False
        self._render_robot_table()
        self._robot_table.selectRow(row)
        if not silent:
            self._append_log(f"Applied robot config: {robot['serial_number']}")
        return True

    def _add_robot(self):
        if self._controller.has_running_robots():
            QMessageBox.warning(self, "Simulation", "Stop running robots before adding a robot.")
            return
        robots = self._config.setdefault("robots", [])
        existing = {str(r.get("serial_number", "")) for r in robots}
        idx = len(robots) + 1
        serial = f"AGV_{idx:03d}"
        while serial in existing:
            idx += 1
            serial = f"AGV_{idx:03d}"
        robots.append({
            "serial_number": serial,
            "initial_position": {"x": 0.0, "y": 0.0, "theta": 0.0, "map_id": "L1"},
        })
        self._dirty = True
        self._render_robot_table()
        new_row = len(robots) - 1
        self._robot_table.selectRow(new_row)
        self._load_robot_to_form(new_row)

    def _remove_robot(self):
        row = self._selected_row()
        if row < 0:
            return
        if self._controller.has_running_robots():
            QMessageBox.warning(self, "Simulation", "Stop running robots before removing a robot.")
            return
        robots = self._config.get("robots", [])
        serial = robots[row].get("serial_number", "")
        del robots[row]
        self._dirty = True
        if robots:
            new_row = min(row, len(robots) - 1)
            self._render_robot_table()
            self._robot_table.selectRow(new_row)
            self._load_robot_to_form(new_row)
        else:
            self._render_robot_table()
        try:
            self._controller.configure(self._config)
        except Exception:
            pass
        self._append_log(f"Removed robot: {serial}")

    def _prepare_for_start(self) -> bool:
        if self._controller.has_running_robots():
            return True
        if self._selected_row() >= 0:
            return self._apply_selected_robot(silent=True)
        try:
            self._controller.configure(self._config)
            return True
        except Exception as exc:
            QMessageBox.critical(self, "Simulation", f"Failed to configure simulator:\n{exc}")
            return False

    def _start_all(self):
        if self._prepare_for_start():
            self._controller.start_all()

    def _start_selected_robot(self):
        serial = self._selected_serial()
        if not serial:
            return
        if self._prepare_for_start():
            self._controller.start_robot(serial)

    def _stop_selected_robot(self):
        serial = self._selected_serial()
        if serial:
            self._controller.stop_robot(serial)

    def _execute_command(self):
        serial = self._selected_serial()
        if not serial:
            return
        if self._dirty and not self._controller.has_running_robots():
            self._apply_selected_robot(silent=True)

        command = self._command_combo.currentText()
        if command == "Obstacle Stop":
            self._controller.set_obstacle_stop(serial, True)
        elif command == "Release Obstacle":
            self._controller.set_obstacle_stop(serial, False)
        elif command == "Set Battery":
            self._controller.set_battery_charge(serial, self._command_battery_spin.value())
        elif command == "Raise Error":
            self._controller.raise_error(
                serial,
                self._error_type_edit.text().strip() or "simulatedError",
                self._error_level_combo.currentText(),
                self._error_desc_edit.text().strip(),
            )
        elif command == "Clear Errors":
            self._controller.clear_errors(serial)

    def _render_robot_table(self):
        selected = self._selected_serial()
        statuses = {s["serial"]: s for s in self._controller.get_statuses()}
        robots = self._config.get("robots", [])

        self._robot_table.blockSignals(True)
        self._robot_table.setRowCount(len(robots))
        selected_row = 0
        for row, robot in enumerate(robots):
            serial = str(robot.get("serial_number", ""))
            status = statuses.get(serial, {})
            if serial == selected:
                selected_row = row
            pos_text = "-"
            if status:
                pos_text = (
                    f"{status.get('x', 0.0):.2f}, "
                    f"{status.get('y', 0.0):.2f}"
                )
            values = [
                serial,
                "RUNNING" if status.get("running") else "STOPPED",
                f"{status.get('battery', self._robot_initial_battery(robot)):.1f}%",
                "yes" if status.get("driving") else "no",
                pos_text,
                str(status.get("errors", 0)),
                "on" if status.get("manual_obstacle") else "off",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 1 and value == "RUNNING":
                    item.setForeground(QBrush(COL_CONN_ONLINE))
                elif col == 1:
                    item.setForeground(QBrush(COL_CONN_OFFLINE))
                self._robot_table.setItem(row, col, item)
        if robots:
            self._robot_table.selectRow(min(selected_row, len(robots) - 1))
        self._robot_table.blockSignals(False)
        self._update_status_label(statuses)

    def _refresh_status(self):
        self._render_robot_table()

    def _update_status_label(self, statuses: Dict[str, Dict[str, Any]]):
        total = len(self._config.get("robots", []))
        running = sum(1 for status in statuses.values() if status.get("running"))
        dirty = " *" if self._dirty else ""
        self._status_label.setText(f"{running}/{total} running{dirty}")

    def _robot_initial_battery(self, robot: Dict[str, Any]) -> float:
        defaults = self._config.get("robot_defaults", {})
        battery = robot.get("battery", defaults.get("battery", {}))
        return float(battery.get("initial_charge", 0.0))

    def _append_log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self._sim_log.append(f"[{ts}] {text}")

    def _on_controller_error(self, text: str):
        self._append_log(f"ERROR: {text}")
        QMessageBox.warning(self, "Simulation", text)

    def shutdown(self):
        self._controller.shutdown()

# ---------------------------------------------------------------------------
# MainWindow – top-level layout + mode switching + signal wiring
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VDA5050 Log Visualizer")
        self.resize(1280, 800)

        self._store = LogStore()
        self._mqtt_client = MqttLogClient(self)
        self._mode = "offline"
        self._live_follow = True
        self._auto_fit_done = False

        # Multi-AGV state
        self._visible_agvs: Set[str] = set()
        self._selected_agv: str = ""
        self._known_agvs: List[str] = []
        self._live_discovered_agvs: Set[str] = set()  # AGVs discovered via connection messages in live mode

        # Recording state
        self._recording = False
        self._record_file = None
        self._record_path = ""
        self._record_count = 0

        # Live UI throttle: update at most every 100ms
        self._live_update_pending = False
        self._live_throttle_timer = QTimer(self)
        self._live_throttle_timer.setSingleShot(True)
        self._live_throttle_timer.setInterval(100)
        self._live_throttle_timer.timeout.connect(self._flush_live_update)

        self._setup_ui()
        self._setup_menu()
        self._setup_toolbar()
        self._connect_signals()

        self.statusBar().showMessage("Ready – File → Open Log (Ctrl+O) to load a JSONL file")

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self._main_tabs = QTabWidget()
        main_layout.addWidget(self._main_tabs)

        monitor_tab = QWidget()
        monitor_layout = QVBoxLayout(monitor_tab)
        monitor_layout.setContentsMargins(0, 0, 0, 0)
        monitor_layout.setSpacing(0)

        splitter = QSplitter(Qt.Horizontal)
        self._canvas = MapCanvas()
        self._info_panel = InfoPanel()
        splitter.addWidget(self._canvas)
        splitter.addWidget(self._info_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        monitor_layout.addWidget(splitter, stretch=1)

        self._timeline = TimelineWidget()
        monitor_layout.addWidget(self._timeline)

        self._simulation_tab = SimulationTab()
        self._main_tabs.addTab(monitor_tab, "Monitor")
        self._main_tabs.addTab(self._simulation_tab, "Simulation")

    def _setup_menu(self):
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("File")
        open_act = QAction("Open Log...", self)
        open_act.setShortcut("Ctrl+O")
        open_act.triggered.connect(self._open_file)
        file_menu.addAction(open_act)
        open_map_act = QAction("Open Map...", self)
        open_map_act.setShortcut("Ctrl+M")
        open_map_act.triggered.connect(self._open_map)
        file_menu.addAction(open_map_act)
        save_act = QAction("Save Recording...", self)
        save_act.setShortcut("Ctrl+S")
        save_act.triggered.connect(self._save_recording)
        file_menu.addAction(save_act)
        file_menu.addSeparator()
        quit_act = QAction("Quit", self)
        quit_act.setShortcut("Ctrl+Q")
        quit_act.triggered.connect(self.close)
        file_menu.addAction(quit_act)

        # Mode menu
        mode_menu = menubar.addMenu("Mode")
        self._offline_act = QAction("Offline (Log Review)", self)
        self._offline_act.setCheckable(True)
        self._offline_act.setChecked(True)
        self._offline_act.triggered.connect(lambda: self._set_mode("offline"))
        mode_menu.addAction(self._offline_act)
        self._live_act = QAction("Live (MQTT)", self)
        self._live_act.setCheckable(True)
        self._live_act.triggered.connect(lambda: self._set_mode("live"))
        mode_menu.addAction(self._live_act)
        mode_menu.addSeparator()
        self._mqtt_connect_act = QAction("Connect MQTT...", self)
        self._mqtt_connect_act.triggered.connect(self._connect_mqtt)
        mode_menu.addAction(self._mqtt_connect_act)
        self._mqtt_disconnect_act = QAction("Disconnect MQTT", self)
        self._mqtt_disconnect_act.setEnabled(False)
        self._mqtt_disconnect_act.triggered.connect(self._disconnect_mqtt)
        mode_menu.addAction(self._mqtt_disconnect_act)
        mode_menu.addSeparator()
        self._go_live_act = QAction("Go Live (Follow Latest)", self)
        self._go_live_act.setShortcut("Ctrl+L")
        self._go_live_act.setCheckable(True)
        self._go_live_act.setChecked(True)
        self._go_live_act.triggered.connect(self._toggle_go_live)
        mode_menu.addAction(self._go_live_act)

        # View menu
        view_menu = menubar.addMenu("View")
        fit_act = QAction("Fit to Content", self)
        fit_act.setShortcut("Ctrl+F")
        fit_act.triggered.connect(self._canvas.fit_to_content)
        view_menu.addAction(fit_act)

    def _setup_toolbar(self):
        toolbar = self.addToolBar("Main")
        toolbar.setMovable(False)
        toolbar.setStyleSheet(
            "QToolBar { spacing: 6px; padding: 2px 4px; border-bottom: 1px solid #DDD; }"
            "QPushButton, QToolButton { padding: 4px 12px; font-size: 12px; border-radius: 3px; }"
        )

        # Record button
        self._btn_record = QPushButton("⏺ Record")
        self._btn_record.setCheckable(True)
        self._btn_record.setStyleSheet(
            "QPushButton { background: #F5F5F5; border: 1px solid #CCC; }"
            "QPushButton:checked { background: #F44336; color: white; border: 1px solid #D32F2F; }"
        )
        self._btn_record.setToolTip("Start/stop recording MQTT messages to JSONL file")
        self._btn_record.clicked.connect(self._toggle_recording)
        toolbar.addWidget(self._btn_record)

        self._record_label = QLabel("")
        self._record_label.setStyleSheet("color: #666; font-size: 11px; margin-left: 4px;")
        toolbar.addWidget(self._record_label)

        # Collect (start/pause) button
        self._btn_collect = QPushButton("⏸ Pause")
        self._btn_collect.setCheckable(True)
        self._btn_collect.setStyleSheet(
            "QPushButton { background: #E8F5E9; border: 1px solid #81C784; }"
            "QPushButton:checked { background: #FFF3E0; color: #E65100; border: 1px solid #FFB74D; }"
        )
        self._btn_collect.setToolTip("Start/pause message collection (paused = no new messages, no deletion)")
        self._btn_collect.clicked.connect(self._toggle_collecting)
        toolbar.addWidget(self._btn_collect)

        # Clear button (live mode)
        self._btn_clear = QPushButton("Clear")
        self._btn_clear.setStyleSheet(
            "QPushButton { background: #F5F5F5; border: 1px solid #CCC; }"
            "QPushButton:hover { background: #FFEBEE; border: 1px solid #E57373; }"
        )
        self._btn_clear.setToolTip("Clear all accumulated log data (live mode)")
        self._btn_clear.clicked.connect(self._clear_live_data)
        toolbar.addWidget(self._btn_clear)

        toolbar.addSeparator()

        # AGV filter button
        self._agv_filter_btn = QToolButton()
        self._agv_filter_btn.setText("AGV Filter: All")
        self._agv_filter_btn.setPopupMode(QToolButton.InstantPopup)
        self._agv_filter_btn.setStyleSheet(
            "QToolButton { background: #F5F5F5; border: 1px solid #CCC; padding: 4px 10px; }"
            "QToolButton::menu-indicator { image: none; }"
        )
        self._agv_filter_menu = QMenu(self._agv_filter_btn)
        self._agv_filter_btn.setMenu(self._agv_filter_menu)
        toolbar.addWidget(self._agv_filter_btn)

    def _connect_signals(self):
        self._timeline.index_changed.connect(self._on_index_changed)
        self._timeline.slider_released.connect(self._on_timeline_slider_released)
        self._canvas.node_clicked.connect(self._on_node_clicked)
        self._canvas.robot_clicked.connect(self._on_robot_clicked)
        self._info_panel.agv_selected.connect(self._on_agv_selected_from_panel)
        self._mqtt_client.log_received.connect(self._on_mqtt_log)
        self._mqtt_client.connected.connect(self._on_mqtt_connected)
        self._mqtt_client.disconnected.connect(self._on_mqtt_disconnected)
        self._mqtt_client.error_occurred.connect(self._on_mqtt_error)

    # -- AGV filter management --

    def _refresh_agv_filter(self, agv_ids: List[str]):
        """Rebuild the AGV filter menu when the set of known AGVs changes."""
        if agv_ids == self._known_agvs:
            return
        self._known_agvs = list(agv_ids)

        self._agv_filter_menu.clear()

        # Select All / Deselect All
        select_all_act = QAction("Select All", self)
        select_all_act.triggered.connect(self._filter_select_all)
        self._agv_filter_menu.addAction(select_all_act)
        deselect_all_act = QAction("Deselect All", self)
        deselect_all_act.triggered.connect(self._filter_deselect_all)
        self._agv_filter_menu.addAction(deselect_all_act)
        self._agv_filter_menu.addSeparator()


        # Per-AGV checkable actions
        for aid in agv_ids:
            act = QAction(aid, self)
            act.setCheckable(True)
            act.setChecked(aid in self._visible_agvs)
            act.toggled.connect(lambda checked, a=aid: self._on_agv_filter_toggled(a, checked))
            self._agv_filter_menu.addAction(act)

        # Auto-add new AGVs to visible set
        for aid in agv_ids:
            if aid not in self._visible_agvs:
                self._visible_agvs.add(aid)

        # If selected AGV not in list anymore, pick first
        if self._selected_agv not in agv_ids and agv_ids:
            self._selected_agv = agv_ids[0]

        self._update_filter_button_text()

    def _on_agv_filter_toggled(self, agv_id: str, checked: bool):
        if checked:
            self._visible_agvs.add(agv_id)
        else:
            self._visible_agvs.discard(agv_id)
            if self._selected_agv == agv_id:
                remaining = sorted(self._visible_agvs)
                self._selected_agv = remaining[0] if remaining else ""

        self._update_filter_button_text()
        self._on_index_changed(self._timeline.current_index)

    def _filter_select_all(self):
        self._visible_agvs = set(self._known_agvs)
        for act in self._agv_filter_menu.actions():
            if act.isCheckable():
                act.setChecked(True)
        self._update_filter_button_text()
        self._on_index_changed(self._timeline.current_index)

    def _filter_deselect_all(self):
        self._visible_agvs.clear()
        for act in self._agv_filter_menu.actions():
            if act.isCheckable():
                act.setChecked(False)
        self._update_filter_button_text()
        self._on_index_changed(self._timeline.current_index)

    def _update_filter_button_text(self):
        n_visible = len(self._visible_agvs)
        n_total = len(self._known_agvs)
        if n_visible == n_total:
            self._agv_filter_btn.setText(f"AGV Filter: All ({n_total})")
        elif n_visible == 0:
            self._agv_filter_btn.setText("AGV Filter: None")
        else:
            self._agv_filter_btn.setText(f"AGV Filter: {n_visible}/{n_total}")

    # -- File operations --

    def _open_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open JSONL Log", "", "JSONL files (*.jsonl);;All files (*)"
        )
        if not path:
            return

        try:
            count = self._store.load_file(path)
        except Exception as exc:
            QMessageBox.critical(self, "Error", f"Failed to load file:\n{exc}")
            return

        if count == 0:
            QMessageBox.warning(self, "Warning", "No valid log entries found in file.")
            return

        self._set_mode("offline")
        agv_ids = self._store.get_agv_ids()
        self._visible_agvs = set(agv_ids)
        self._selected_agv = agv_ids[0] if agv_ids else ""
        self._refresh_agv_filter(agv_ids)

        self._timeline.set_range(count - 1)
        self._timeline.set_order_markers(self._store.order_change_indices)
        self._timeline.set_index(0)
        self._auto_fit_done = False
        self._on_index_changed(0)
        self.statusBar().showMessage(f"Loaded {count} entries from {path} ({len(agv_ids)} AGV(s))")

    def _open_map(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Map YAML", "", "YAML files (*.yaml *.yml);;All files (*)"
        )
        if not path:
            return

        try:
            md = MapData()
            count = md.load(path)
        except Exception as exc:
            QMessageBox.critical(self, "Error", f"Failed to load map:\n{exc}")
            return

        if count == 0:
            QMessageBox.warning(self, "Warning", "No vertices found in map file.")
            return

        self._canvas.set_map_data(md)
        self._canvas.fit_to_content()
        n_special = sum(1 for v in md.vertices if v.vtype in MAP_LABEL_TYPES)
        self.statusBar().showMessage(
            f"Map loaded: {count} nodes, {len(md.edges)} edges, "
            f"{md.mutex_edge_count} mutex edges, "
            f"{n_special} special nodes (CHGE/PARK/PICKDROP) – {path}"
        )
        # 마지막 열린 맵 경로 저장
        settings = _load_settings()
        settings["last_map"] = path
        _save_settings(settings)

    def _save_recording(self):
        if not self._store.entries:
            QMessageBox.information(self, "Info", "No data to save.")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Recording", "recording.jsonl", "JSONL files (*.jsonl)"
        )
        if not path:
            return

        try:
            with open(path, "w", encoding="utf-8") as f:
                for e in self._store.entries:
                    line = json.dumps(
                        {"timestamp": e.timestamp, "topic": e.topic, "data": e.data},
                        ensure_ascii=False,
                    )
                    f.write(line + "\n")
            self.statusBar().showMessage(f"Saved {len(self._store.entries)} entries to {path}")
        except Exception as exc:
            QMessageBox.critical(self, "Error", f"Failed to save:\n{exc}")

    # -- Recording --

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        default_name = f"vda5050_log_{ts}.jsonl"
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Recording As", default_name, "JSONL files (*.jsonl)"
        )
        if not path:
            self._btn_record.setChecked(False)
            return

        try:
            self._record_file = open(path, "w", encoding="utf-8")
        except Exception as exc:
            QMessageBox.critical(self, "Error", f"Cannot open file:\n{exc}")
            self._btn_record.setChecked(False)
            return

        self._recording = True
        self._record_path = path
        self._record_count = 0
        self._btn_record.setText("⏹ Stop")
        self._record_label.setText(f"Recording → {path}")
        self.statusBar().showMessage(f"Recording to {path}")

    def _stop_recording(self):
        if self._record_file:
            self._record_file.close()
            self._record_file = None
        self._recording = False
        self._btn_record.setChecked(False)
        self._btn_record.setText("⏺ Record")
        msg = f"Saved {self._record_count} entries → {self._record_path}"
        self._record_label.setText(msg)
        self.statusBar().showMessage(msg)

    def _record_entry(self, entry: LogEntry):
        if not self._recording or not self._record_file:
            return
        try:
            line = json.dumps(
                {"timestamp": entry.timestamp, "topic": entry.topic, "data": entry.data},
                ensure_ascii=False,
            )
            self._record_file.write(line + "\n")
            self._record_file.flush()
            self._record_count += 1
            self._record_label.setText(f"⏺ {self._record_count} entries → {self._record_path}")
        except Exception as exc:
            self.statusBar().showMessage(f"Recording write error: {exc}")
            self._stop_recording()

    # -- Mode switching --

    def _set_mode(self, mode: str):
        self._mode = mode
        self._offline_act.setChecked(mode == "offline")
        self._live_act.setChecked(mode == "live")
        if mode == "offline":
            self._timeline.stop_play()
            self.statusBar().showMessage("Offline mode – use timeline to navigate")
        else:
            self.statusBar().showMessage("Live mode – waiting for MQTT data")

    # -- Clear live data --

    def _toggle_collecting(self):
        """Toggle message collection on/off."""
        paused = self._btn_collect.isChecked()
        self._store.collecting = not paused
        if paused:
            self._btn_collect.setText("▶ Collect")
            self.statusBar().showMessage("Message collection paused – data frozen")
        else:
            self._btn_collect.setText("⏸ Pause")
            self.statusBar().showMessage("Message collection resumed")

    def _clear_live_data(self):
        """Clear accumulated log data while keeping MQTT connection and robot info."""
        if self._mode != "live":
            self.statusBar().showMessage("Clear is only available in live mode")
            return

        self._store.clear()
        self._timeline.set_range(0)
        self._timeline.set_order_markers([])
        self._timeline.set_timestamp("")
        self._auto_fit_done = False
        self._canvas.set_data({}, {}, set())
        self._info_panel._text.clear()
        self._info_panel._order_raw_text.clear()
        self._info_panel._instant_raw_text.clear()
        self.statusBar().showMessage("Live data cleared – waiting for new messages")

    # -- MQTT operations --

    def _connect_mqtt(self):
        dlg = MqttConnectionDialog(self)
        if dlg.exec_() != QDialog.Accepted:
            return

        params = dlg.get_params()
        self._store.clear()
        self._visible_agvs.clear()
        self._selected_agv = ""
        self._known_agvs.clear()
        self._live_discovered_agvs.clear()
        self._agv_filter_menu.clear()
        self._timeline.set_range(0)
        self._auto_fit_done = False
        self._mqtt_client.connect_to_broker(**params)
        self._set_mode("live")
        mfr = params["manufacturer"]
        self.statusBar().showMessage(
            f"Connecting to {params['host']}:{params['port']} – subscribing to all {mfr} AGVs..."
        )
        # 설정 저장
        settings = _load_settings()
        settings["mqtt"] = params
        _save_settings(settings)

    def _disconnect_mqtt(self):
        self._mqtt_client.stop()
        self._mqtt_disconnect_act.setEnabled(False)
        self._mqtt_connect_act.setEnabled(True)
        self.statusBar().showMessage("MQTT disconnected")

    def _toggle_go_live(self):
        self._live_follow = self._go_live_act.isChecked()

    @pyqtSlot(object)
    def _on_mqtt_log(self, entry: LogEntry):
        self._store.append_entry(entry)
        self._record_entry(entry)

        # Discover new AGVs from connection messages
        if entry.topic == "connection" and entry.agv_id:
            if entry.agv_id not in self._live_discovered_agvs:
                self._live_discovered_agvs.add(entry.agv_id)
                agv_ids = sorted(self._live_discovered_agvs)
                self._refresh_agv_filter(agv_ids)
                conn_state = entry.data.get("connectionState", "?")
                self.statusBar().showMessage(
                    f"Discovered AGV: {entry.agv_id} ({conn_state}) – {len(agv_ids)} robot(s) total"
                )

        # Throttled UI update: schedule instead of immediate
        self._live_update_pending = True
        if not self._live_throttle_timer.isActive():
            self._live_throttle_timer.start()

    def _flush_live_update(self):
        """Batch-apply pending live updates to UI."""
        if not self._live_update_pending:
            return
        self._live_update_pending = False

        count = len(self._store.entries)
        self._timeline.set_range(count - 1)
        self._timeline.set_order_markers(self._store.order_change_indices)

        if self._live_follow and not self._timeline.is_user_scrubbing:
            self._timeline.set_index(count - 1)

    @pyqtSlot()
    def _on_timeline_slider_released(self):
        if self._mode != "live" or not self._live_follow or not self._store.entries:
            return
        self._timeline.set_index(len(self._store.entries) - 1)

    @pyqtSlot()
    def _on_mqtt_connected(self):
        self._mqtt_disconnect_act.setEnabled(True)
        self._mqtt_connect_act.setEnabled(False)
        self.statusBar().showMessage("MQTT connected – receiving data")

    @pyqtSlot()
    def _on_mqtt_disconnected(self):
        self._mqtt_disconnect_act.setEnabled(False)
        self._mqtt_connect_act.setEnabled(True)
        self.statusBar().showMessage("MQTT disconnected")

    @pyqtSlot(str)
    def _on_mqtt_error(self, msg: str):
        self.statusBar().showMessage(f"MQTT Error: {msg}")
        QMessageBox.warning(self, "MQTT Error", msg)

    # -- Display update --

    @pyqtSlot(int)
    def _on_index_changed(self, index: int):
        if not self._store.entries:
            return

        snapshots = self._store.get_snapshots(index)
        trails: Dict[str, List[Tuple[float, float]]] = {}
        for aid in self._visible_agvs:
            trails[aid] = self._store.get_trail(index, aid)

        timestamp = self._store.entries[index].timestamp if index < len(self._store.entries) else ""

        self._canvas.set_data(snapshots, trails, self._visible_agvs, self._selected_agv)
        self._timeline.set_timestamp(timestamp)

        # Update InfoPanel
        agv_ids = sorted(self._visible_agvs & set(snapshots.keys()))
        self._info_panel.update_agv_list(agv_ids, self._selected_agv)
        if self._selected_agv in snapshots:
            agv_color = self._canvas.get_agv_color(self._selected_agv)
            self._info_panel.set_snapshot(snapshots[self._selected_agv], self._selected_agv, timestamp, agv_color)
        elif agv_ids:
            self._selected_agv = agv_ids[0]
            agv_color = self._canvas.get_agv_color(self._selected_agv)
            self._info_panel.set_snapshot(snapshots[self._selected_agv], self._selected_agv, timestamp, agv_color)

        # Auto-fit on first data display
        if not self._auto_fit_done and snapshots:
            has_nodes = any(s.nodes for s in snapshots.values())
            if has_nodes:
                self._canvas.fit_to_content()
                self._auto_fit_done = True

    # -- Robot / Node click --

    @pyqtSlot(str)
    def _on_robot_clicked(self, agv_id: str):
        self._selected_agv = agv_id
        self._on_index_changed(self._timeline.current_index)

    @pyqtSlot(str)
    def _on_agv_selected_from_panel(self, agv_id: str):
        self._selected_agv = agv_id
        self._on_index_changed(self._timeline.current_index)

    @pyqtSlot(str, dict)
    def _on_node_clicked(self, node_id: str, node_data: dict):
        pos = node_data.get("nodePosition", {})
        released = node_data.get("released", False)
        actions = node_data.get("actions", [])
        seq = node_data.get("sequenceId", "?")

        info = (
            f"Node: {node_id}\n"
            f"Sequence ID: {seq}\n"
            f"Released: {released} ({'base' if released else 'horizon'})\n"
            f"Position: ({pos.get('x', '?')}, {pos.get('y', '?')})\n"
            f"Map: {pos.get('mapId', '?')}\n"
        )
        if actions:
            info += f"\nActions ({len(actions)}):\n"
            for a in actions:
                info += f"  - {a.get('actionType', '?')} [{a.get('actionId', '')}] ({a.get('blockingType', '?')})\n"

        QMessageBox.information(self, f"Node: {node_id}", info)

    def closeEvent(self, event):
        if self._recording:
            self._stop_recording()
        self._simulation_tab.shutdown()
        self._mqtt_client.stop()
        event.accept()


# ---------------------------------------------------------------------------
# Application entry point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet("""
        QMainWindow { background: #F5F5F5; }
        QMenuBar { background: #FFFFFF; border-bottom: 1px solid #DDD; }
        QStatusBar { background: #FFFFFF; border-top: 1px solid #DDD; }
    """)

    window = MainWindow()

    # Parse arguments: [logfile.jsonl] [--map map.yaml]
    args = sys.argv[1:]
    log_path = None
    map_path = None
    i = 0
    while i < len(args):
        if args[i] == "--map" and i + 1 < len(args):
            map_path = args[i + 1]
            i += 2
        else:
            if log_path is None:
                log_path = args[i]
            i += 1

    # Load map if provided
    if map_path:
        try:
            md = MapData()
            md.load(map_path)
            window._canvas.set_map_data(md)
        except Exception as exc:
            print(f"Error loading map {map_path}: {exc}", file=sys.stderr)

    # Load log file if provided
    if log_path:
        try:
            count = window._store.load_file(log_path)
            if count > 0:
                agv_ids = window._store.get_agv_ids()
                window._visible_agvs = set(agv_ids)
                window._selected_agv = agv_ids[0] if agv_ids else ""
                window._refresh_agv_filter(agv_ids)
                window._timeline.set_range(count - 1)
                window._timeline.set_order_markers(window._store.order_change_indices)
                window._timeline.set_index(0)
                window._on_index_changed(0)
                window.statusBar().showMessage(f"Loaded {count} entries from {log_path} ({len(agv_ids)} AGV(s))")
        except Exception as exc:
            print(f"Error loading {log_path}: {exc}", file=sys.stderr)

    # Auto fit if map loaded
    if map_path:
        window._canvas.fit_to_content()

    # ── 저장된 설정으로 자동 시작 ──────────────────────────────────────────
    _s = _load_settings()

    # 마지막 열린 맵 자동 로드 (명령줄 인자로 맵이 지정되지 않은 경우)
    if not map_path and _s.get("last_map"):
        _map_p = _s["last_map"]
        try:
            from pathlib import Path as _Path
            if _Path(_map_p).exists():
                md = MapData()
                md.load(_map_p)
                window._canvas.set_map_data(md)
                window._canvas.fit_to_content()
                window.statusBar().showMessage(f"Map auto-loaded: {_map_p}")
        except Exception:
            pass

    # 저장된 MQTT 설정으로 자동 연결
    if _s.get("mqtt"):
        _m = _s["mqtt"]
        try:
            window._store.clear()
            window._mqtt_client.connect_to_broker(
                host=_m.get("host", "localhost"),
                port=_m.get("port", 1883),
                interface_name=_m.get("interface_name", "uagv"),
                version=_m.get("version", "v2.0.0"),
                manufacturer=_m.get("manufacturer", "inatech"),
            )
            window._set_mode("live")
            window.statusBar().showMessage(
                f"Auto-connecting to {_m.get('host')}:{_m.get('port')} "
                f"(manufacturer={_m.get('manufacturer')})..."
            )
        except Exception:
            pass
    # ───────────────────────────────────────────────────────────────────────

    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
