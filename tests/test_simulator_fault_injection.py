from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SIM_ROOT = ROOT / "rmf_dev_tool-main" / "vda5050_robot_simulator"
sys.path.insert(0, str(SIM_ROOT))


class FakeRobot:
    def __init__(
        self,
        serial_number: str,
        *,
        node: str = "",
        driving: bool = False,
    ) -> None:
        self.serial_number = serial_number
        self.last_node_id = node
        self.driving = driving


class RecordingMqtt:
    def __init__(self) -> None:
        self.states: list[dict] = []
        self.connections: list[str] = []

    def publish_state(self, state: dict) -> None:
        self.states.append(state)

    def publish_connection(self, state: str) -> None:
        self.connections.append(state)


class SimulatorFaultInjectionTests(unittest.TestCase):
    def setUp(self) -> None:
        from vda5050_simulator.fault_injection import FaultInjectionController

        self.controller_type = FaultInjectionController

    def test_rule_fires_once_and_persists_state_override(self) -> None:
        controller = self.controller_type.from_config(
            [
                {
                    "robot": "R1",
                    "when": {"at_node": "N2", "driving": True},
                    "action": {"type": "set_estop", "value": "AUTOACK"},
                }
            ]
        )
        robot = FakeRobot("R1", node="N2", driving=True)

        first = controller.apply(robot, elapsed=3.0)
        second = controller.apply(robot, elapsed=4.0)

        self.assertTrue(first.triggered)
        self.assertEqual(first.state_overrides["safetyState"]["eStop"], "AUTOACK")
        self.assertFalse(second.triggered)
        self.assertEqual(second.state_overrides["safetyState"]["eStop"], "AUTOACK")

    def test_combined_trigger_and_robot_scope(self) -> None:
        controller = self.controller_type.from_config(
            [
                {
                    "robot": "R1",
                    "when": {"elapsed_at_least": 5.0, "at_node": "N2"},
                    "action": {"type": "set_operating_mode", "value": "MANUAL"},
                }
            ]
        )

        self.assertFalse(
            controller.apply(FakeRobot("R2", node="N2"), elapsed=6.0).triggered
        )
        self.assertFalse(
            controller.apply(FakeRobot("R1", node="N1"), elapsed=6.0).triggered
        )
        self.assertFalse(
            controller.apply(FakeRobot("R1", node="N2"), elapsed=4.0).triggered
        )
        self.assertTrue(
            controller.apply(FakeRobot("R1", node="N2"), elapsed=5.0).triggered
        )

    def test_all_supported_state_actions_are_generic(self) -> None:
        cases = [
            ("set_operating_mode", "MANUAL", ("operatingMode", "MANUAL")),
            ("set_paused", True, ("paused", True)),
            ("set_map_id", "OTHER", ("agvPosition.mapId", "OTHER")),
            (
                "set_position_initialized",
                False,
                ("agvPosition.positionInitialized", False),
            ),
        ]
        for action_type, value, (path, expected) in cases:
            with self.subTest(action=action_type):
                controller = self.controller_type.from_config(
                    [
                        {
                            "robot": "any-name",
                            "when": {"elapsed_at_least": 0},
                            "action": {"type": action_type, "value": value},
                        }
                    ]
                )
                effect = controller.apply(FakeRobot("any-name"), elapsed=0)
                actual = effect.state_overrides
                for key in path.split("."):
                    actual = actual[key]
                self.assertEqual(actual, expected)

    def test_state_suppression_does_not_hide_connection_action(self) -> None:
        controller = self.controller_type.from_config(
            [
                {
                    "robot": "R1",
                    "when": {"elapsed_at_least": 0},
                    "action": {"type": "suppress_state", "value": True},
                },
                {
                    "robot": "R1",
                    "when": {"elapsed_at_least": 0},
                    "action": {"type": "publish_connection", "value": "OFFLINE"},
                },
            ]
        )

        effect = controller.apply(FakeRobot("R1"), elapsed=0)

        self.assertTrue(effect.suppress_state)
        self.assertEqual(effect.connection_states, ("OFFLINE",))

    def test_state_publisher_emits_connection_while_state_is_suppressed(self) -> None:
        from vda5050_simulator.state_publisher import StatePublisher

        controller = self.controller_type.from_config(
            [
                {
                    "robot": "R1",
                    "when": {},
                    "action": {"type": "suppress_state", "value": True},
                },
                {
                    "robot": "R1",
                    "when": {},
                    "action": {"type": "publish_connection", "value": "OFFLINE"},
                },
            ]
        )
        publisher = StatePublisher.__new__(StatePublisher)
        publisher._faults = controller
        publisher._robot = FakeRobot("R1")
        publisher._mqtt = RecordingMqtt()
        publisher._serial_number = "R1"
        publisher._started_at = 10.0
        publisher._build_state_dict = lambda: {"operatingMode": "AUTOMATIC"}

        with patch(
            "vda5050_simulator.state_publisher.time.monotonic", return_value=11.0
        ):
            publisher.publish_state_now()

        self.assertEqual(publisher._mqtt.connections, ["OFFLINE"])
        self.assertEqual(publisher._mqtt.states, [])

    def test_invalid_trigger_or_action_fails_at_startup(self) -> None:
        invalid_items = [
            [{"robot": "R1", "when": {"unknown": 1}, "action": {"type": "set_paused", "value": True}}],
            [{"robot": "R1", "when": {}, "action": {"type": "unknown", "value": True}}],
            [{"robot": "", "when": {}, "action": {"type": "set_paused", "value": True}}],
        ]
        for items in invalid_items:
            with self.subTest(items=items):
                with self.assertRaises(ValueError):
                    self.controller_type.from_config(items)

    def test_scenario_values_live_in_yaml_not_controller_source(self) -> None:
        source_path = (
            SIM_ROOT / "vda5050_simulator" / "fault_injection.py"
        )
        source = source_path.read_text(encoding="utf-8")
        for scenario_value in ("AGV_A1", "AGV_B1", "2104", "2106", "C2"):
            self.assertNotIn(scenario_value, source)

        scenario = (
            SIM_ROOT / "connected_corridor_fault_scenarios.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("fault_injection:", scenario)
        self.assertIn("enabled: false", scenario)


if __name__ == "__main__":
    unittest.main()
