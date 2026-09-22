"""Materialize one field configuration for the Fleet Adapter and Compose."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml

from .deployment import DeploymentConfigError, DeploymentProfile
from .corridor_registry import CorridorRegistry


_CONTAINER_ROOT = "/run/rmf-field"
_FLEET_CONFIG = f"{_CONTAINER_ROOT}/fleet_config.yaml"
_NAV_GRAPH = f"{_CONTAINER_ROOT}/nav_graph.yaml"
_MQTT_CA = f"{_CONTAINER_ROOT}/mqtt-ca.pem"
_MQTT_CERT = f"{_CONTAINER_ROOT}/mqtt-client.pem"
_MQTT_KEY = f"{_CONTAINER_ROOT}/mqtt-client.key"
_API_CONFIG = f"{_CONTAINER_ROOT}/api-server-config.py"


@dataclass(frozen=True)
class PreparedProductionRuntime:
    fleet_config: Path
    compose_override: Path
    api_server_config: Path


def _load_mapping(path: Path, error: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise DeploymentConfigError(error) from exc
    if not isinstance(value, dict):
        raise DeploymentConfigError(error)
    return value


def _topic_prefix(profile: DeploymentProfile) -> str:
    suffix = "/+/state"
    topic = profile.mqtt.state_topic
    if not topic.endswith(suffix):
        raise DeploymentConfigError("mqtt.state_topic.invalid")
    return topic[: -len(suffix)]


def _geometry_contains(geometry: Any, x: float, y: float) -> bool:
    if not isinstance(geometry, dict):
        return False
    if isinstance(geometry.get("circle"), dict):
        circle = geometry["circle"]
        try:
            dx = x - float(circle["x"])
            dy = y - float(circle["y"])
            radius = float(circle["radius"])
        except (KeyError, TypeError, ValueError):
            return False
        return radius > 0 and dx * dx + dy * dy <= radius * radius
    if isinstance(geometry.get("bounds"), dict):
        bounds = geometry["bounds"]
        try:
            return (
                float(bounds["min_x"]) <= x <= float(bounds["max_x"])
                and float(bounds["min_y"]) <= y <= float(bounds["max_y"])
            )
        except (KeyError, TypeError, ValueError):
            return False
    return False


def _valid_geometry(geometry: Any) -> bool:
    if not isinstance(geometry, dict):
        return False
    if set(geometry) == {"circle"} and isinstance(geometry["circle"], dict):
        circle = geometry["circle"]
        try:
            values = (
                float(circle["x"]),
                float(circle["y"]),
                float(circle["radius"]),
            )
        except (KeyError, TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values) and values[2] > 0
    if set(geometry) == {"bounds"} and isinstance(geometry["bounds"], dict):
        bounds = geometry["bounds"]
        try:
            values = (
                float(bounds["min_x"]),
                float(bounds["max_x"]),
                float(bounds["min_y"]),
                float(bounds["max_y"]),
            )
        except (KeyError, TypeError, ValueError):
            return False
        return (
            all(math.isfinite(value) for value in values)
            and values[0] < values[1]
            and values[2] < values[3]
        )
    return False


def _segment_intersects_geometry(
    geometry: Any,
    start: tuple[float, float],
    end: tuple[float, float],
) -> bool:
    """Return whether a lane's interior crosses a managed geometry."""
    if not isinstance(geometry, dict):
        return False
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    if isinstance(geometry.get("bounds"), dict):
        bounds = geometry["bounds"]
        try:
            minimum_x = float(bounds["min_x"])
            maximum_x = float(bounds["max_x"])
            minimum_y = float(bounds["min_y"])
            maximum_y = float(bounds["max_y"])
        except (KeyError, TypeError, ValueError):
            return False
        lower = 0.0
        upper = 1.0
        for origin, delta, minimum, maximum in (
            (x1, dx, minimum_x, maximum_x),
            (y1, dy, minimum_y, maximum_y),
        ):
            if abs(delta) < 1e-12:
                if not minimum < origin < maximum:
                    return False
                continue
            first = (minimum - origin) / delta
            second = (maximum - origin) / delta
            entry, exit_ = sorted((first, second))
            lower = max(lower, entry)
            upper = min(upper, exit_)
            if lower >= upper:
                return False
        return max(lower, 1e-9) < min(upper, 1.0 - 1e-9)
    if isinstance(geometry.get("circle"), dict):
        circle = geometry["circle"]
        try:
            center_x = float(circle["x"])
            center_y = float(circle["y"])
            radius = float(circle["radius"])
        except (KeyError, TypeError, ValueError):
            return False
        length_squared = dx * dx + dy * dy
        if length_squared <= 0.0 or radius <= 0.0:
            return False
        closest = (
            (center_x - x1) * dx + (center_y - y1) * dy
        ) / length_squared
        closest = max(1e-9, min(1.0 - 1e-9, closest))
        distance_squared = (
            (x1 + closest * dx - center_x) ** 2
            + (y1 + closest * dy - center_y) ** 2
        )
        return distance_squared < radius * radius
    return False


def prepare_production_runtime(
    profile: DeploymentProfile,
    output_dir: Path,
) -> PreparedProductionRuntime:
    """Render runtime-only files without copying credentials into either file."""
    profile.require_valid()
    if profile.paths.fleet_config is None or profile.paths.nav_graph is None:
        raise DeploymentConfigError("paths.runtime.missing")

    source_fleet = _load_mapping(
        profile.paths.fleet_config, "fleet_config.unreadable"
    )
    nav = _load_mapping(profile.paths.nav_graph, "paths.nav_graph.unreadable")
    levels = nav.get("levels")
    if not isinstance(levels, dict) or len(levels) != 1:
        raise DeploymentConfigError("nav_graph.single_level_required")
    level_name, level = next(iter(levels.items()))
    if not isinstance(level, dict):
        raise DeploymentConfigError("paths.nav_graph.unreadable")
    try:
        vertices = level.get("vertices", [])
        if not isinstance(vertices, list):
            raise TypeError("vertices must be a list")
        index_names: list[str] = []
        node_positions: dict[str, tuple[float, float]] = {}
        for vertex in vertices:
            if (
                not isinstance(vertex, list)
                or len(vertex) < 3
                or not isinstance(vertex[2], dict)
                or not vertex[2].get("name")
            ):
                raise TypeError("every production vertex must have a name")
            name = str(vertex[2]["name"])
            if name in node_positions:
                raise ValueError("duplicate navigation node name")
            index_names.append(name)
            position = (float(vertex[0]), float(vertex[1]))
            if not all(math.isfinite(value) for value in position):
                raise ValueError("nonfinite navigation coordinate")
            node_positions[name] = position
        lanes = {
            (index_names[int(lane[0])], index_names[int(lane[1])])
            for lane in level.get("lanes", [])
        }
    except (IndexError, KeyError, TypeError, ValueError) as exc:
        raise DeploymentConfigError("paths.nav_graph.unreadable") from exc
    node_names = set(node_positions)

    if profile.paths.corridor_config is None:
        raise DeploymentConfigError("paths.corridor_config.unreadable")
    try:
        registry = CorridorRegistry.from_yaml(profile.paths.corridor_config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise DeploymentConfigError("corridor_config.invalid") from exc
    if not registry.enabled or not registry.blocks:
        raise DeploymentConfigError("corridor_config.no_blocks")
    for holding_bay in registry.holding_bays.values():
        if not _valid_geometry(holding_bay.geometry):
            raise DeploymentConfigError("corridor.holding_bay_geometry.invalid")
        if holding_bay.node_id not in node_names:
            raise DeploymentConfigError("corridor.holding_bay_node.unknown")
        x, y = node_positions[holding_bay.node_id]
        if not _geometry_contains(holding_bay.geometry, x, y):
            raise DeploymentConfigError(
                "corridor.holding_bay_geometry.mismatch"
            )
    for block in registry.blocks.values():
        if not _valid_geometry(block.geometry):
            raise DeploymentConfigError("corridor.block_geometry.invalid")
        block_edges: set[tuple[str, str]] = set()
        for edge in block.edges_a_to_b | block.edges_b_to_a:
            parts = edge.split(">", 1)
            if len(parts) != 2 or tuple(parts) not in lanes:
                raise DeploymentConfigError("corridor.edge.unknown")
            if not _segment_intersects_geometry(
                block.geometry,
                node_positions[parts[0]],
                node_positions[parts[1]],
            ):
                raise DeploymentConfigError("corridor.edge.geometry_mismatch")
            block_edges.add((parts[0], parts[1]))
        for release_node in (
            block.release_node_a_to_b,
            block.release_node_b_to_a,
        ):
            if release_node is not None and release_node not in node_names:
                raise DeploymentConfigError("corridor.release_node.unknown")
        for start, end in lanes:
            if (start, end) in block_edges:
                continue
            if _segment_intersects_geometry(
                block.geometry, node_positions[start], node_positions[end]
            ):
                raise DeploymentConfigError("corridor.lane.unmanaged")
    for route in registry.routes:
        if not (route.start_nodes | route.goal_nodes) <= node_names:
            raise DeploymentConfigError("corridor.route_node.unknown")

    fleet = deepcopy(source_fleet)
    rmf_fleet = fleet.get("rmf_fleet")
    manager = fleet.get("fleet_manager")
    if not isinstance(rmf_fleet, dict) or not isinstance(manager, dict):
        raise DeploymentConfigError("fleet_config.structure")
    configured_robots = rmf_fleet.get("robots")
    if not isinstance(configured_robots, dict):
        raise DeploymentConfigError("fleet_config.robots.missing")
    if set(configured_robots) != set(profile.robots):
        raise DeploymentConfigError("fleet_config.robots.mismatch")
    for robot_id, config in configured_robots.items():
        if not isinstance(config, dict):
            raise DeploymentConfigError(f"fleet_config.{robot_id}.invalid")
        charger = str(config.get("charger") or "")
        if charger not in node_names:
            raise DeploymentConfigError(
                f"fleet_config.{robot_id}.charger.unknown"
            )

    manufacturers = {robot.manufacturer for robot in profile.robots.values()}
    if len(manufacturers) != 1:
        raise DeploymentConfigError("robots.manufacturer.multiple")
    manufacturer = next(iter(manufacturers))
    prefix = _topic_prefix(profile)
    if not prefix.endswith(f"/{manufacturer}"):
        raise DeploymentConfigError("mqtt.state_topic.manufacturer_mismatch")

    limits = rmf_fleet.setdefault("limits", {})
    if not isinstance(limits, dict):
        raise DeploymentConfigError("fleet_config.limits.invalid")
    limits["linear"] = [
        profile.physical.max_linear_speed,
        min(
            profile.physical.max_acceleration,
            profile.physical.max_deceleration,
        ),
    ]
    robot_profile = rmf_fleet.setdefault("profile", {})
    if not isinstance(robot_profile, dict):
        raise DeploymentConfigError("fleet_config.profile.invalid")
    robot_profile["footprint"] = profile.physical.footprint_radius
    robot_profile["vicinity"] = profile.physical.vicinity_radius

    manager.update({
        "ip": profile.mqtt.host,
        "port": profile.mqtt.port,
        "prefix": prefix,
        "manufacturer": manufacturer,
        "keepalive_sec": profile.mqtt.keepalive_sec,
        "reconnect_max_delay_sec": profile.mqtt.reconnect_max_delay_sec,
        "security": {
            "username_env": "RMF_FIELD_MQTT_USERNAME",
            "password_env": "RMF_FIELD_MQTT_PASSWORD",
            "ca_file": _MQTT_CA,
            "client_cert_file": _MQTT_CERT if profile.mqtt.cert_file else "",
            "client_key_file": _MQTT_KEY if profile.mqtt.key_file else "",
            "tls_required": True,
        },
    })
    robot_map_ids: dict[str, str] = {}
    for robot_id, robot in profile.robots.items():
        if len(robot.allowed_map_ids) != 1:
            raise DeploymentConfigError(
                f"robots.{robot_id}.single_map_required"
            )
        robot_map_ids[robot_id] = robot.allowed_map_ids[0]
    adapter_config = fleet.setdefault("adapter", {})
    if not isinstance(adapter_config, dict):
        raise DeploymentConfigError("fleet_config.adapter.invalid")
    adapter_config["rmf_map_name"] = str(level_name)
    adapter_config["robot_map_ids"] = robot_map_ids
    fleet["reference_coordinates"] = {
        str(level_name): {
            "rmf": [list(point) for point in profile.calibration.rmf],
            "robot": [list(point) for point in profile.calibration.robot],
        }
    }

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    fleet_path = output / "production.fleet.yaml"
    compose_path = output / "production.compose.override.yml"
    api_config_path = output / "production.api_server_config.py"
    fleet_path.write_text(
        yaml.safe_dump(fleet, sort_keys=False), encoding="utf-8"
    )
    api_config_path.write_text(
        '"""Generated field API configuration; do not edit."""\n\n'
        "from api_server.default_config import config\n\n"
        "config.update({\n"
        '    "db_url": "sqlite:////api_server_ws/run/db.sqlite3",\n'
        '    "cache_directory": "/api_server_ws/run/cache",\n'
        '    "ros_args": [],\n'
        '    "log_level": "INFO",\n'
        '    "host": "127.0.0.1",\n'
        '    "port": 8100,\n'
        '    "timezone": "Asia/Seoul",\n'
        "})\n",
        encoding="utf-8",
    )

    workspace = Path(__file__).resolve().parents[1]
    adapter_source = (
        workspace / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"
    ).resolve()
    volumes = [
        f"{adapter_source}:/vda5050_ws/src/vda5050_fleet_adapter:ro",
        f"{fleet_path}:{_FLEET_CONFIG}:ro",
        f"{profile.paths.nav_graph.resolve()}:{_NAV_GRAPH}:ro",
        f"{profile.mqtt.ca_file.resolve()}:{_MQTT_CA}:ro",
    ]
    if profile.mqtt.cert_file is not None:
        volumes.append(
            f"{profile.mqtt.cert_file.resolve()}:{_MQTT_CERT}:ro"
        )
    if profile.mqtt.key_file is not None:
        volumes.append(f"{profile.mqtt.key_file.resolve()}:{_MQTT_KEY}:ro")
    compose = {
        "services": {
            "rmf_api_server": {
                "environment": {
                    "RMF_API_SERVER_CONFIG": _API_CONFIG,
                },
                "volumes": [
                    f"{api_config_path}:{_API_CONFIG}:ro",
                ],
            },
            "vda5050_fleet_adapter": {
                "volumes": volumes,
                "environment": {
                    "RMF_FIELD_MQTT_USERNAME": "${RMF_FIELD_MQTT_USERNAME:?}",
                    "RMF_FIELD_MQTT_PASSWORD": "${RMF_FIELD_MQTT_PASSWORD:?}",
                },
                "command": [
                    "bash",
                    "-c",
                    "cd /vda5050_ws && "
                    "colcon build --packages-select vda5050_fleet_adapter "
                    "--symlink-install && source install/setup.bash && "
                    "ros2 run vda5050_fleet_adapter fleet_adapter "
                    f"-c {_FLEET_CONFIG} -n {_NAV_GRAPH}",
                ],
            }
        }
    }
    compose_path.write_text(
        yaml.safe_dump(compose, sort_keys=False), encoding="utf-8"
    )
    return PreparedProductionRuntime(
        fleet_path, compose_path, api_config_path
    )
