"""RECONSTRUCTED ROS 2 entry point for the baseline T2 adapter."""

from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Mapping

from vda5050_fleet_adapter.infra.config.yaml_config_loader import (
    YamlConfigLoader,
)

logger = logging.getLogger(__name__)


def mqtt_config_from_mapping(
    manager: Mapping[str, Any],
    environ: Mapping[str, str] | None = None,
):
    """Build MQTT settings without exposing resolved credentials."""
    from vda5050_fleet_adapter.usecase.ports.config_port import MqttConfig

    values = os.environ if environ is None else environ
    security = manager.get('security') or {}
    if not isinstance(security, dict):
        raise ValueError('fleet_manager.security must be a mapping')

    def secret(name: str) -> str:
        reference = str(security.get(f'{name}_env', '')).strip()
        if not reference:
            return ''
        value = values.get(reference, '')
        if not value:
            raise ValueError(
                f'fleet_manager.security.{name}_env references missing {reference}'
            )
        return value

    return MqttConfig(
        broker_host=str(manager.get('ip', '127.0.0.1')),
        broker_port=int(manager.get('port', 1883)),
        keepalive_sec=int(manager.get('keepalive_sec', 60)),
        reconnect_max_delay_sec=int(
            manager.get('reconnect_max_delay_sec', 60)
        ),
        username=secret('username'),
        password=secret('password'),
        ca_file=str(security.get('ca_file', '')),
        cert_file=str(security.get('client_cert_file', '')),
        key_file=str(security.get('client_key_file', '')),
        tls_required=bool(security.get('tls_required', False)),
    )


def create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Reconstructed VDA5050 Open-RMF fleet adapter',
    )
    parser.add_argument(
        '-c', '--config_file', required=True,
        help='Path to the fleet adapter YAML configuration',
    )
    parser.add_argument(
        '-n', '--nav_graph', required=True,
        help='Path to the RMF navigation graph YAML',
    )
    parser.add_argument(
        '-s', '--server_uri', default='',
        help='Optional RMF API server websocket URI',
    )
    parser.add_argument(
        '-sim', '--use_sim_time', action='store_true',
        help='Use ROS simulation time',
    )
    return parser


def load_and_validate_config(path: str | Path) -> dict[str, Any]:
    data = YamlConfigLoader().load(str(path), '')
    if not data:
        raise ValueError(f'configuration is empty or unreadable: {path}')
    for key in ('rmf_fleet', 'fleet_manager'):
        if not isinstance(data.get(key), dict):
            raise ValueError(f'configuration is missing mapping: {key}')
    fleet = data['rmf_fleet']
    manager = data['fleet_manager']
    if not fleet.get('name'):
        raise ValueError('rmf_fleet.name is required')
    if not isinstance(fleet.get('robots'), dict) or not fleet['robots']:
        raise ValueError('rmf_fleet.robots must contain at least one robot')
    if not manager.get('prefix'):
        raise ValueError('fleet_manager.prefix is required')
    return data


def _attach_update_handle(robot: Any, result: Any) -> None:
    if result is None:
        return
    then = getattr(result, 'then', None)
    if callable(then):
        then(lambda handle: setattr(robot, 'update_handle', handle))
    else:
        robot.update_handle = result


def run_adapter(args: argparse.Namespace) -> None:
    """Run the ROS adapter. Imports ROS only after CLI/config validation."""
    import rclpy
    import rclpy.node
    from rclpy.parameter import Parameter

    import rmf_adapter
    from rmf_adapter import Adapter
    import rmf_adapter.easy_full_control as rmf_easy

    from vda5050_fleet_adapter.infra.mqtt.mqtt_client import MqttClient
    from vda5050_fleet_adapter.infra.mqtt.vda5050_robot_api import (
        Vda5050RobotAPI,
    )
    from vda5050_fleet_adapter.presentation.robot_adapter import RobotAdapter
    from vda5050_fleet_adapter.usecase.actions.tool_pick_drop_handler import (
        ToolPickDropHandler,
    )
    from vda5050_fleet_adapter.usecase.graph_utils import load_nav_graph
    raw = load_and_validate_config(args.config_file)
    nodes, edges, graph, _map_name = load_nav_graph(args.nav_graph)
    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        args.config_file, args.nav_graph
    )
    if not fleet_config:
        raise RuntimeError('RMF rejected fleet configuration or nav graph')

    rmf_adapter.init_rclcpp()
    fleet_name = raw['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_vda5050_command_handle')
    adapter = Adapter.make(f'{fleet_name}_vda5050_fleet_adapter')
    if adapter is None:
        raise RuntimeError(
            'unable to create RMF adapter; verify schedule node and ROS_DOMAIN_ID'
        )
    if args.use_sim_time:
        node.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        adapter.node.use_sim_time()
    if args.server_uri:
        fleet_config.server_uri = args.server_uri
    fleet_handle = adapter.add_easy_fleet(fleet_config)
    adapter.start()

    adapter_config = raw.get('adapter') or {}
    robot_map_ids = adapter_config.get('robot_map_ids') or {}
    if not isinstance(robot_map_ids, dict):
        raise ValueError('adapter.robot_map_ids must be a mapping')
    rmf_map_name = str(adapter_config.get('rmf_map_name') or _map_name)
    transformations = fleet_config.transformations_to_robot_coordinates or {}
    coordinate_transform = transformations.get(rmf_map_name)
    if robot_map_ids:
        missing_map_ids = set(fleet_config.known_robots) - set(robot_map_ids)
        if missing_map_ids:
            raise ValueError(
                'adapter.robot_map_ids is missing robots: '
                + ', '.join(sorted(missing_map_ids))
            )
        if coordinate_transform is None:
            raise ValueError(
                f'no RMF-to-robot coordinate transform for {rmf_map_name}'
            )

    manager = raw['fleet_manager']
    mqtt = MqttClient(
        mqtt_config_from_mapping(manager),
        client_id=f'{fleet_name}_rmf_adapter',
    )
    api = Vda5050RobotAPI(
        mqtt,
        prefix=str(manager['prefix']),
        manufacturer=str(manager.get('manufacturer', 'inatech')),
        download_map_config=raw.get('download_map'),
    )
    for robot_name in fleet_config.known_robots:
        api.subscribe_robot(robot_name)
    api.connect()

    robots = {
        robot_name: RobotAdapter(
            name=robot_name,
            configuration=fleet_config.get_known_robot_configuration(
                robot_name
            ),
            node=node,
            api=api,
            fleet_handle=fleet_handle,
            nav_nodes=nodes,
            nav_edges=edges,
            nav_graph=graph,
            arrival_threshold=float(
                raw.get('adapter', {}).get('arrival_threshold', 0.5)
            ),
            action_handler=ToolPickDropHandler(),
            coordinate_transform=coordinate_transform,
            robot_map_id=str(robot_map_ids.get(robot_name) or ''),
            rmf_map_name=rmf_map_name,
        )
        for robot_name in fleet_config.known_robots
    }
    registration_started: set[str] = set()
    map_mismatch_logged: set[str] = set()
    period = 1.0 / float(
        raw['rmf_fleet'].get('robot_state_update_frequency', 10.0)
    )

    def update_loop() -> None:
        while rclpy.ok():
            started_at = time.monotonic()
            for robot_name, robot in robots.items():
                data = api.get_data(robot_name)
                if data is None:
                    continue
                if robot.robot_map_id and data.map_name != robot.robot_map_id:
                    if robot_name not in map_mismatch_logged:
                        logger.error(
                            'Robot map mismatch: robot=%s expected=%s actual=%s',
                            robot_name, robot.robot_map_id, data.map_name,
                        )
                        map_mismatch_logged.add(robot_name)
                    continue
                map_mismatch_logged.discard(robot_name)
                state = rmf_easy.RobotState(
                    robot.rmf_map_name or data.map_name,
                    data.position,
                    data.battery_soc,
                )
                if robot.update_handle is None:
                    if robot_name not in registration_started:
                        registration_started.add(robot_name)
                        result = robot.add_to_fleet(state)
                        _attach_update_handle(robot, result)
                    continue
                robot.update(state, data)
            remaining = period - (time.monotonic() - started_at)
            if remaining > 0:
                time.sleep(remaining)

    thread = threading.Thread(
        target=update_loop, name='vda5050-rmf-update', daemon=True
    )
    thread.start()
    logger.info(
        'VDA5050 Fleet Adapter v1.17.0 reconstructed starting: fleet=%s',
        fleet_name,
    )
    try:
        rclpy.spin(node)
    finally:
        api.disconnect()
        node.destroy_node()


def main(argv: list[str] | None = None) -> None:
    import rclpy

    actual_argv = list(sys.argv if argv is None else argv)
    rclpy.init(args=actual_argv)
    try:
        args_without_ros = rclpy.utilities.remove_ros_args(actual_argv)
        args = create_argument_parser().parse_args(args_without_ros[1:])
        run_adapter(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
