"""Typed deployment profiles and fail-closed production preflight checks."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
from typing import Any, Literal, Mapping

import yaml


class DeploymentConfigError(ValueError):
    """Configuration is structurally invalid and cannot be interpreted."""


def _resolved_path(root: Path, value: Any) -> Path | None:
    if value in (None, ""):
        return None
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else root / path


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _is_placeholder(value: str) -> bool:
    normalized = value.strip().upper()
    return (
        not normalized
        or normalized.startswith(("REPLACE_ME", "CHANGE_ME"))
        or normalized in {"EXAMPLE", "PLACEHOLDER"}
    )


def _non_collinear(points: tuple[tuple[float, float], ...]) -> bool:
    if len(points) < 3:
        return False
    first = points[0]
    for left_index in range(1, len(points) - 1):
        left = points[left_index]
        for right in points[left_index + 1 :]:
            cross = (
                (left[0] - first[0]) * (right[1] - first[1])
                - (left[1] - first[1]) * (right[0] - first[0])
            )
            if abs(cross) > 1e-9:
                return True
    return False


def _points(value: Any) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list):
        return ()
    result: list[tuple[float, float]] = []
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            return ()
        try:
            result.append((float(item[0]), float(item[1])))
        except (TypeError, ValueError):
            return ()
    return tuple(result)


@dataclass(frozen=True)
class SecretRef:
    env: str | None = None
    file: Path | None = None

    @classmethod
    def from_raw(cls, value: Any, *, root: Path) -> "SecretRef | None":
        if value in (None, ""):
            return None
        if not isinstance(value, dict):
            raise DeploymentConfigError("secret.source")
        env = str(value["env"]).strip() if value.get("env") else None
        file = _resolved_path(root, value.get("file"))
        if (env is None) == (file is None):
            raise DeploymentConfigError("secret.source")
        return cls(env=env, file=file)

    def resolve(self, environ: Mapping[str, str]) -> str:
        if self.env is not None:
            value = environ.get(self.env, "")
        else:
            try:
                value = self.file.read_text(encoding="utf-8").strip() if self.file else ""
            except OSError:
                value = ""
        if not value:
            raise DeploymentConfigError("secret.unresolved")
        return value


@dataclass(frozen=True)
class MqttDeploymentConfig:
    host: str
    port: int
    keepalive_sec: int
    reconnect_max_delay_sec: int
    state_topic: str = ""
    username: SecretRef | None = None
    password: SecretRef | None = None
    ca_file: Path | None = None
    cert_file: Path | None = None
    key_file: Path | None = None
    tls_required: bool = False


@dataclass(frozen=True)
class RobotDeploymentConfig:
    manufacturer: str
    serial_number: str
    allowed_map_ids: tuple[str, ...]
    required: bool = True


@dataclass(frozen=True)
class CalibrationConfig:
    rmf: tuple[tuple[float, float], ...] = ()
    robot: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class PhysicalConfig:
    footprint_radius: float = 0.0
    vicinity_radius: float = 0.0
    max_linear_speed: float = 0.0
    max_acceleration: float = 0.0
    max_deceleration: float = 0.0


@dataclass(frozen=True)
class TelemetryConfig:
    state_timeout: float = 5.0
    connection_timeout: float = 10.0
    operational_checks_required: bool = False


@dataclass(frozen=True)
class RmfApiConfig:
    url: str
    bearer_token: SecretRef | None = None


@dataclass(frozen=True)
class DeploymentPaths:
    fleet_config: Path | None = None
    nav_graph: Path | None = None
    corridor_config: Path | None = None
    compose_files: tuple[Path, ...] = ()


@dataclass(frozen=True)
class DeploymentProfile:
    mode: Literal["simulation", "production"] | str
    mqtt: MqttDeploymentConfig
    robots: Mapping[str, RobotDeploymentConfig]
    calibration: CalibrationConfig
    physical: PhysicalConfig
    telemetry: TelemetryConfig
    rmf_api: RmfApiConfig
    paths: DeploymentPaths
    source: Path
    _environ: Mapping[str, str] = field(repr=False, compare=False)

    @classmethod
    def load(
        cls,
        path: Path,
        environ: Mapping[str, str] | None = None,
    ) -> "DeploymentProfile":
        source = Path(path).expanduser().resolve()
        try:
            raw = yaml.safe_load(source.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            raise DeploymentConfigError("profile.unreadable") from error
        if not isinstance(raw, dict):
            raise DeploymentConfigError("profile.mapping")
        root = source.parent
        deployment = raw.get("deployment") or {}
        mqtt_raw = raw.get("mqtt") or {}
        tls_raw = mqtt_raw.get("tls") or {}
        robots_raw = raw.get("robots") or {}
        calibration_raw = raw.get("calibration") or {}
        physical_raw = raw.get("physical") or {}
        telemetry_raw = raw.get("telemetry") or {}
        rmf_raw = raw.get("rmf_api") or {}
        paths_raw = raw.get("paths") or {}
        if not all(isinstance(item, dict) for item in (
            deployment, mqtt_raw, tls_raw, robots_raw, calibration_raw,
            physical_raw, telemetry_raw, rmf_raw, paths_raw,
        )):
            raise DeploymentConfigError("profile.mapping")

        robots = {
            str(robot_id): RobotDeploymentConfig(
                manufacturer=str(item.get("manufacturer", "")),
                serial_number=str(item.get("serial_number", "")),
                allowed_map_ids=tuple(str(value) for value in item.get("allowed_map_ids", [])),
                required=bool(item.get("required", True)),
            )
            for robot_id, item in robots_raw.items()
            if isinstance(item, dict)
        }
        return cls(
            mode=str(deployment.get("mode", "simulation")).lower(),
            mqtt=MqttDeploymentConfig(
                host=str(mqtt_raw.get("host", "")),
                port=int(mqtt_raw.get("port", 1883)),
                keepalive_sec=int(mqtt_raw.get("keepalive_sec", 60)),
                reconnect_max_delay_sec=int(mqtt_raw.get("reconnect_max_delay_sec", 60)),
                state_topic=str(mqtt_raw.get("state_topic", "")),
                username=SecretRef.from_raw(mqtt_raw.get("username"), root=root),
                password=SecretRef.from_raw(mqtt_raw.get("password"), root=root),
                ca_file=_resolved_path(root, tls_raw.get("ca_file")),
                cert_file=_resolved_path(root, tls_raw.get("client_cert_file")),
                key_file=_resolved_path(root, tls_raw.get("client_key_file")),
                tls_required=bool(tls_raw.get("required", False)),
            ),
            robots=robots,
            calibration=CalibrationConfig(
                rmf=_points(calibration_raw.get("rmf")),
                robot=_points(calibration_raw.get("robot")),
            ),
            physical=PhysicalConfig(**{
                name: _float(physical_raw.get(name))
                for name in PhysicalConfig.__dataclass_fields__
            }),
            telemetry=TelemetryConfig(
                state_timeout=_float(telemetry_raw.get("state_timeout"), 5.0),
                connection_timeout=_float(telemetry_raw.get("connection_timeout"), 10.0),
                operational_checks_required=bool(
                    telemetry_raw.get("operational_checks_required", False)
                ),
            ),
            rmf_api=RmfApiConfig(
                url=str(rmf_raw.get("url", "")),
                bearer_token=SecretRef.from_raw(rmf_raw.get("bearer_token"), root=root),
            ),
            paths=DeploymentPaths(
                fleet_config=_resolved_path(root, paths_raw.get("fleet_config")),
                nav_graph=_resolved_path(root, paths_raw.get("nav_graph")),
                corridor_config=_resolved_path(root, paths_raw.get("corridor_config")),
                compose_files=tuple(
                    item for item in (
                        _resolved_path(root, value)
                        for value in paths_raw.get("compose_files", [])
                    ) if item is not None
                ),
            ),
            source=source,
            _environ=dict(os.environ if environ is None else environ),
        )

    def validate(self) -> tuple[str, ...]:
        errors: set[str] = set()
        if self.mode not in {"simulation", "production"}:
            errors.add("deployment.mode.invalid")
            return tuple(sorted(errors))
        if self.mode == "simulation":
            return ()

        if _is_placeholder(self.mqtt.host):
            errors.add("mqtt.host.placeholder")
        if _is_placeholder(self.mqtt.state_topic):
            errors.add("mqtt.state_topic.placeholder")
        for name in ("port", "keepalive_sec", "reconnect_max_delay_sec"):
            if getattr(self.mqtt, name) <= 0:
                errors.add(f"mqtt.{name}.nonpositive")
        if not self.mqtt.tls_required:
            errors.add("mqtt.tls.required")
        if self.mqtt.ca_file is None or not self.mqtt.ca_file.is_file():
            errors.add("mqtt.tls.ca_file.unreadable")
        if (self.mqtt.cert_file is None) != (self.mqtt.key_file is None):
            errors.add("mqtt.tls.client_pair")
        elif self.mqtt.cert_file is not None:
            if not self.mqtt.cert_file.is_file():
                errors.add("mqtt.tls.client_cert_file.unreadable")
            if self.mqtt.key_file is None or not self.mqtt.key_file.is_file():
                errors.add("mqtt.tls.client_key_file.unreadable")
        self._validate_secret(errors, "mqtt.username", self.mqtt.username)
        self._validate_secret(errors, "mqtt.password", self.mqtt.password)
        self._validate_secret(errors, "rmf_api.bearer_token", self.rmf_api.bearer_token)

        if not self.robots:
            errors.add("robots.empty")
        identities: set[tuple[str, str]] = set()
        for robot_id, robot in self.robots.items():
            identity = (robot.manufacturer, robot.serial_number)
            if not all(identity) or any(_is_placeholder(item) for item in identity):
                errors.add(f"robots.{robot_id}.identity.placeholder")
            if identity in identities:
                errors.add("robots.identity.duplicate")
            identities.add(identity)
            if not robot.allowed_map_ids:
                errors.add(f"robots.{robot_id}.allowed_map_ids.empty")

        if len(self.calibration.rmf) != len(self.calibration.robot):
            errors.add("calibration.count.mismatch")
        for name, points in (
            ("rmf", self.calibration.rmf), ("robot", self.calibration.robot)
        ):
            if len(points) < 3:
                errors.add(f"calibration.{name}.insufficient")
            elif not _non_collinear(points):
                errors.add(f"calibration.{name}.collinear")

        for name in PhysicalConfig.__dataclass_fields__:
            if getattr(self.physical, name) <= 0:
                errors.add(f"physical.{name}.nonpositive")
        for name in ("state_timeout", "connection_timeout"):
            if getattr(self.telemetry, name) <= 0:
                errors.add(f"telemetry.{name}.nonpositive")
        if not self.telemetry.operational_checks_required:
            errors.add("telemetry.operational_checks_required")
        if _is_placeholder(self.rmf_api.url):
            errors.add("rmf_api.url.placeholder")

        required_paths = {
            "fleet_config": self.paths.fleet_config,
            "nav_graph": self.paths.nav_graph,
            "corridor_config": self.paths.corridor_config,
        }
        for name, path in required_paths.items():
            if path is None or not path.is_file():
                errors.add(f"paths.{name}.unreadable")
        for index, path in enumerate(self.paths.compose_files):
            if not path.is_file():
                errors.add(f"paths.compose_files.{index}.unreadable")
        if not self.paths.compose_files:
            errors.add("paths.compose_files.empty")
        if self.paths.nav_graph is not None and self.paths.nav_graph.is_file():
            try:
                nav = yaml.safe_load(self.paths.nav_graph.read_text(encoding="utf-8")) or {}
                if bool((nav.get("metadata") or {}).get("simulation_only", False)):
                    errors.add("map.simulation_only")
            except (OSError, yaml.YAMLError, AttributeError):
                errors.add("paths.nav_graph.unreadable")
        return tuple(sorted(errors))

    def _validate_secret(
        self, errors: set[str], name: str, secret: SecretRef | None
    ) -> None:
        if secret is None:
            errors.add(f"{name}.unresolved")
            return
        try:
            secret.resolve(self._environ)
        except DeploymentConfigError:
            errors.add(f"{name}.unresolved")

    def require_valid(self) -> None:
        errors = self.validate()
        if errors:
            raise DeploymentConfigError(";".join(errors))

    def redacted_snapshot(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "mqtt": {
                "host": self.mqtt.host,
                "port": self.mqtt.port,
                "state_topic": self.mqtt.state_topic,
                "username": "<redacted>",
                "password": "<redacted>",
                "tls_required": self.mqtt.tls_required,
            },
            "robots": {
                robot_id: {
                    "manufacturer": item.manufacturer,
                    "serial_number": item.serial_number,
                    "allowed_map_ids": list(item.allowed_map_ids),
                    "required": item.required,
                }
                for robot_id, item in self.robots.items()
            },
            "rmf_api": {"url": self.rmf_api.url, "bearer_token": "<redacted>"},
            "validation_errors": list(self.validate()),
        }

    def to_json(self) -> str:
        return json.dumps(self.redacted_snapshot(), sort_keys=True)
