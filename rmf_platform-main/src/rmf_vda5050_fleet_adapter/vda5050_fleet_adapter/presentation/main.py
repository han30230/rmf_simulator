"""RECONSTRUCTED ROS 2 entry point for the baseline T2 adapter."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys
import threading
import time
from typing import Any

from vda5050_fleet_adapter.infra.config.yaml_config_loader import (
    YamlConfigLoader,
)

logger = logging.getLogger(__name__)


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
    from vda5050_fleet_adapter.usecase.ports.config_port import MqttConfig

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

    manager = raw['fleet_manager']
    mqtt = MqttClient(MqttConfig(
        broker_host=str(manager.get('ip', '127.0.0.1')),
        broker_port=int(manager.get('port', 1883)),
        keepalive_sec=int(manager.get('keepalive_sec', 60)),
        reconnect_max_delay_sec=int(
            manager.get('reconnect_max_delay_sec', 60)
        ),
    ), client_id=f'{fleet_name}_rmf_adapter')
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
        )
        for robot_name in fleet_config.known_robots
    }
    registration_started: set[str] = set()
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
                state = rmf_easy.RobotState(
                    data.map_name, data.position, data.battery_soc
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
