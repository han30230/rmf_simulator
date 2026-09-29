from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import yaml

from traffic_control.deployment import DeploymentConfigError, DeploymentProfile
from traffic_control.production_runtime import prepare_production_runtime


class ProductionRuntimePreparationTests(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary.name)
        for name in ("ca.pem", "client.pem", "client.key"):
            (self.root / name).write_text("test\n", encoding="utf-8")
        (self.root / "compose.yaml").write_text("services: {}\n", encoding="utf-8")
        (self.root / "corridor.yaml").write_text(
            yaml.safe_dump({
                "traffic_control": {
                    "enabled": True,
                    "holding_bays": {
                        "LEFT_HB": {
                            "node_id": "LEFT", "capacity": 1,
                            "geometry": {"circle": {"x": 0, "y": 0, "radius": 0.5}},
                        },
                        "RIGHT_HB": {
                            "node_id": "RIGHT", "capacity": 1,
                            "geometry": {"circle": {"x": 5, "y": 0, "radius": 0.5}},
                        },
                    },
                    "safe_stop_groups": {
                        "LEFT_STOP": {"members": ["LEFT_HB"]},
                        "RIGHT_STOP": {"members": ["RIGHT_HB"]},
                    },
                    "blocks": [{
                        "id": "DSR_C1", "entry_a": "LEFT_STOP",
                        "entry_b": "RIGHT_STOP", "capacity": 1,
                        "geometry": {"bounds": {
                            "min_x": 0.25, "max_x": 4.75,
                            "min_y": -0.5, "max_y": 0.5,
                        }},
                        "edges_a_to_b": ["LEFT>RIGHT"],
                        "edges_b_to_a": ["RIGHT>LEFT"],
                        "release_node_a_to_b": "RIGHT",
                        "release_node_b_to_a": "LEFT",
                    }],
                    "corridor_chains": [{
                        "id": "DSR_LINE", "blocks": ["DSR_C1"],
                        "safe_stops": ["LEFT_STOP", "RIGHT_STOP"],
                        "max_active_robots": 1,
                    }],
                }
            }, sort_keys=False),
            encoding="utf-8",
        )
        (self.root / "map.yaml").write_text(
            yaml.safe_dump({
                "building_name": "dsr_lab",
                "levels": {
                    "L1": {
                        "vertices": [
                            [0.0, 0.0, {"name": "LEFT"}],
                            [5.0, 0.0, {"name": "RIGHT"}],
                        ],
                        "lanes": [[0, 1, {}], [1, 0, {}]],
                    }
                },
            }),
            encoding="utf-8",
        )
        (self.root / "fleet.yaml").write_text(
            yaml.safe_dump({
                "rmf_fleet": {
                    "name": "TOOL",
                    "limits": {"linear": [9.0, 9.0], "angular": [0.5, 0.5]},
                    "profile": {"footprint": 9.0, "vicinity": 9.0},
                    "robots": {"ROBOT_01": {"charger": "LEFT"}},
                },
                "fleet_manager": {
                    "ip": "simulation.invalid",
                    "port": 1883,
                    "prefix": "simulation/prefix",
                    "manufacturer": "simulation",
                },
                "reference_coordinates": {"OLD": {"rmf": [], "robot": []}},
            }, sort_keys=False),
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self._temporary.cleanup()

    def _profile(self) -> DeploymentProfile:
        raw = {
            "deployment": {"mode": "production"},
            "mqtt": {
                "host": "mqtt.dsr.example",
                "port": 8883,
                "state_topic": "uagv/v2.0.0/vendor/+/state",
                "keepalive_sec": 30,
                "reconnect_max_delay_sec": 20,
                "username": {"env": "FAB_MQTT_USER"},
                "password": {"env": "FAB_MQTT_PASSWORD"},
                "tls": {
                    "required": True,
                    "ca_file": "ca.pem",
                    "client_cert_file": "client.pem",
                    "client_key_file": "client.key",
                },
            },
            "robots": {
                "ROBOT_01": {
                    "manufacturer": "vendor",
                    "serial_number": "ROBOT_01",
                    "allowed_map_ids": ["ROBOT_MAP"],
                    "required": True,
                }
            },
            "calibration": {
                "rmf": [[0.0, 0.0], [5.0, 0.0], [0.0, 1.0]],
                "robot": [[10.0, 20.0], [15.0, 20.0], [10.0, 21.0]],
            },
            "physical": {
                "footprint_radius": 0.4,
                "vicinity_radius": 0.65,
                "max_linear_speed": 0.25,
                "max_acceleration": 0.15,
                "max_deceleration": 0.1,
            },
            "telemetry": {
                "state_timeout": 5.0,
                "connection_timeout": 10.0,
                "operational_checks_required": True,
            },
            "rmf_api": {
                "url": "http://127.0.0.1:8100/tasks/robot_task",
                "bearer_token": {"env": "RMF_API_TOKEN"},
            },
            "paths": {
                "fleet_config": "fleet.yaml",
                "nav_graph": "map.yaml",
                "corridor_config": "corridor.yaml",
                "compose_files": ["compose.yaml"],
            },
        }
        path = self.root / "deployment.yaml"
        path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
        return DeploymentProfile.load(path, environ={
            "FAB_MQTT_USER": "operator",
            "FAB_MQTT_PASSWORD": "secret",
            "RMF_API_TOKEN": "token",
        })

    def test_materializes_one_consistent_adapter_configuration(self) -> None:
        prepared = prepare_production_runtime(
            self._profile(), self.root / "runtime"
        )

        fleet = yaml.safe_load(prepared.fleet_config.read_text(encoding="utf-8"))
        manager = fleet["fleet_manager"]
        self.assertEqual(manager["ip"], "mqtt.dsr.example")
        self.assertEqual(manager["port"], 8883)
        self.assertEqual(manager["prefix"], "uagv/v2.0.0/vendor")
        self.assertEqual(manager["manufacturer"], "vendor")
        self.assertEqual(manager["security"]["username_env"], "RMF_FIELD_MQTT_USERNAME")
        self.assertEqual(manager["security"]["password_env"], "RMF_FIELD_MQTT_PASSWORD")
        self.assertTrue(manager["security"]["tls_required"])
        self.assertEqual(fleet["rmf_fleet"]["limits"]["linear"], [0.25, 0.1])
        self.assertEqual(fleet["rmf_fleet"]["profile"], {
            "footprint": 0.4, "vicinity": 0.65,
        })
        self.assertEqual(fleet["reference_coordinates"], {
            "L1": {
                "rmf": [[0.0, 0.0], [5.0, 0.0], [0.0, 1.0]],
                "robot": [[10.0, 20.0], [15.0, 20.0], [10.0, 21.0]],
            }
        })
        self.assertEqual(fleet["adapter"]["rmf_map_name"], "L1")
        self.assertEqual(fleet["adapter"]["robot_map_ids"], {
            "ROBOT_01": "ROBOT_MAP"
        })

        compose = prepared.compose_override.read_text(encoding="utf-8")
        self.assertIn(str(prepared.fleet_config), compose)
        self.assertIn(str(self.root / "map.yaml"), compose)
        self.assertIn(str(self.root / "ca.pem"), compose)
        self.assertNotIn("operator", compose)
        self.assertNotIn("secret", compose)
        self.assertNotIn("token", compose)
        api_config = prepared.api_server_config.read_text(encoding="utf-8")
        self.assertIn('"host": "127.0.0.1"', api_config)
        self.assertNotIn("use_sim_time", api_config)
        self.assertIn(str(prepared.api_server_config), compose)

    def test_rejects_unmonitored_adapter_robot(self) -> None:
        fleet_path = self.root / "fleet.yaml"
        fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
        fleet["rmf_fleet"]["robots"]["ROBOT_02"] = {"charger": "RIGHT"}
        fleet_path.write_text(yaml.safe_dump(fleet), encoding="utf-8")

        with self.assertRaisesRegex(
            DeploymentConfigError, "fleet_config.robots.mismatch"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def test_rejects_unknown_robot_charger_node(self) -> None:
        fleet_path = self.root / "fleet.yaml"
        fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
        fleet["rmf_fleet"]["robots"]["ROBOT_01"]["charger"] = "MISSING"
        fleet_path.write_text(yaml.safe_dump(fleet), encoding="utf-8")

        with self.assertRaisesRegex(
            DeploymentConfigError, "fleet_config.ROBOT_01.charger.unknown"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def test_rejects_corridor_edge_missing_from_navigation_graph(self) -> None:
        corridor_path = self.root / "corridor.yaml"
        corridor = yaml.safe_load(corridor_path.read_text(encoding="utf-8"))
        corridor["traffic_control"]["blocks"][0]["edges_a_to_b"].append(
            "LEFT>MISSING"
        )
        corridor_path.write_text(yaml.safe_dump(corridor), encoding="utf-8")

        with self.assertRaisesRegex(
            DeploymentConfigError, "corridor.edge.unknown"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def test_rejects_holding_bay_geometry_that_misses_its_node(self) -> None:
        corridor_path = self.root / "corridor.yaml"
        corridor = yaml.safe_load(corridor_path.read_text(encoding="utf-8"))
        corridor["traffic_control"]["holding_bays"]["RIGHT_HB"]["geometry"] = {
            "circle": {"x": 50.0, "y": 0.0, "radius": 0.5}
        }
        corridor_path.write_text(yaml.safe_dump(corridor), encoding="utf-8")

        with self.assertRaisesRegex(
            DeploymentConfigError, "corridor.holding_bay_geometry.mismatch"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def test_rejects_malformed_navigation_vertex_as_configuration_error(self) -> None:
        map_path = self.root / "map.yaml"
        nav = yaml.safe_load(map_path.read_text(encoding="utf-8"))
        nav["levels"]["L1"]["vertices"][0][0] = "not-a-coordinate"
        map_path.write_text(yaml.safe_dump(nav), encoding="utf-8")

        with self.assertRaisesRegex(
            DeploymentConfigError, "paths.nav_graph.unreadable"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def test_rejects_multiple_robot_map_ids_until_mapping_is_unambiguous(self) -> None:
        profile = self._profile()
        robot = profile.robots["ROBOT_01"]
        object.__setattr__(robot, "allowed_map_ids", ("MAP_A", "MAP_B"))

        with self.assertRaisesRegex(
            DeploymentConfigError, "robots.ROBOT_01.single_map_required"
        ):
            prepare_production_runtime(profile, self.root / "runtime")

    def test_rejects_navigation_lane_crossing_block_without_tracking(self) -> None:
        map_path = self.root / "map.yaml"
        nav = yaml.safe_load(map_path.read_text(encoding="utf-8"))
        level = nav["levels"]["L1"]
        level["vertices"].append([2.5, 0.2, {"name": "BYPASS"}])
        level["lanes"].extend([[0, 2, {}], [2, 1, {}]])
        map_path.write_text(yaml.safe_dump(nav), encoding="utf-8")

        with self.assertRaisesRegex(
            DeploymentConfigError, "corridor.lane.unmanaged"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def test_rejects_nonfinite_navigation_and_geometry_values(self) -> None:
        map_path = self.root / "map.yaml"
        nav = yaml.safe_load(map_path.read_text(encoding="utf-8"))
        nav["levels"]["L1"]["vertices"][0][0] = float("nan")
        map_path.write_text(yaml.safe_dump(nav), encoding="utf-8")
        with self.assertRaisesRegex(
            DeploymentConfigError, "paths.nav_graph.unreadable"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

        self.setUp_map_with_valid_coordinates()
        corridor_path = self.root / "corridor.yaml"
        corridor = yaml.safe_load(corridor_path.read_text(encoding="utf-8"))
        corridor["traffic_control"]["blocks"][0]["geometry"]["bounds"][
            "max_x"
        ] = float("inf")
        corridor_path.write_text(yaml.safe_dump(corridor), encoding="utf-8")
        with self.assertRaisesRegex(
            DeploymentConfigError, "corridor.block_geometry.invalid"
        ):
            prepare_production_runtime(self._profile(), self.root / "runtime")

    def setUp_map_with_valid_coordinates(self) -> None:
        (self.root / "map.yaml").write_text(
            yaml.safe_dump({
                "building_name": "dsr_lab",
                "levels": {"L1": {
                    "vertices": [
                        [0.0, 0.0, {"name": "LEFT"}],
                        [5.0, 0.0, {"name": "RIGHT"}],
                    ],
                    "lanes": [[0, 1, {}], [1, 0, {}]],
                }},
            }),
            encoding="utf-8",
        )


if __name__ == "__main__":
    unittest.main()
