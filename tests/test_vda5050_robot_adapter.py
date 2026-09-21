from __future__ import annotations

from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
ADAPTER_ROOT = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"
sys.path.insert(0, str(ADAPTER_ROOT))

from vda5050_fleet_adapter.presentation.robot_adapter import (  # noqa: E402
    NavigationState,
    RobotAdapter,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import (  # noqa: E402
    RobotAPIResult, RobotCommandState,
    RobotUpdateData,
)


class FakeApi:
    def __init__(self, *, completed: bool) -> None:
        self.completed = completed
        self.navigate_calls = []
        self.stop_calls = []
        self.pause_calls = []
        self.command_state = None

    def get_command_state(self, _robot_name):
        return self.command_state

    def is_robot_connected(self, _robot_name):
        return True

    def is_manual_mode(self, _robot_name):
        return False

    def is_command_completed(self, _robot_name, _cmd_id):
        return self.completed

    def navigate(self, *args, **kwargs):
        self.navigate_calls.append((args, kwargs))
        return RobotAPIResult.SUCCESS

    def stop(self, *args, **kwargs):
        self.stop_calls.append((args, kwargs))
        return RobotAPIResult.SUCCESS

    def pause(self, *args):
        self.pause_calls.append(args)
        return RobotAPIResult.SUCCESS


class OffsetTransform:
    def apply(self, position):
        return [position[0] + 100.0, position[1] + 200.0, position[2] + 0.5]


class Destination:
    name = "RIGHT"
    map = "L1"
    position = [105.0, 200.0, 0.5]
    waypoint_names = ["LEFT", "RIGHT"]


class FakeExecution:
    def __init__(self) -> None:
        self.finished_count = 0

    def finished(self) -> None:
        self.finished_count += 1


class Identifier:
    def is_same(self, _other) -> bool:
        return True


class Vda5050RobotAdapterTests(unittest.TestCase):
    def _adapter(self, *, completed: bool):
        adapter = RobotAdapter("ROBOT_01", api=FakeApi(completed=completed))
        execution = FakeExecution()
        adapter.execution = execution
        adapter._nav = NavigationState(
            is_navigating=True,
            target_node="RIGHT",
            # RMF coordinates deliberately differ from VDA5050 coordinates.
            target_position=[5.0, 0.0],
            cmd_id=7,
            order_id="order_7",
        )
        return adapter, execution

    def test_navigation_transforms_rmf_path_and_uses_robot_map_id(self) -> None:
        api = FakeApi(completed=False)
        adapter = RobotAdapter(
            "ROBOT_01",
            api=api,
            nav_nodes={
                "LEFT": {"x": 0.0, "y": 0.0, "attributes": {}},
                "RIGHT": {"x": 5.0, "y": 0.0, "attributes": {}},
            },
            nav_edges={
                "left_right": {
                    "start": "LEFT", "end": "RIGHT", "attributes": {}
                }
            },
            coordinate_transform=OffsetTransform(),
            robot_map_id="ROBOT_MAP",
        )
        adapter.last_node_id = "LEFT"
        execution = FakeExecution()
        execution.identifier = Identifier()

        adapter.navigate(Destination(), execution)

        self.assertEqual(len(api.navigate_calls), 1)
        args, _kwargs = api.navigate_calls[0]
        nodes = args[2]
        self.assertEqual(args[4], "ROBOT_MAP")
        self.assertEqual(
            [(node.node_position.x, node.node_position.y) for node in nodes],
            [(100.0, 200.0), (105.0, 200.0)],
        )
        self.assertTrue(all(
            node.node_position.map_id == "ROBOT_MAP" for node in nodes
        ))

    def test_navigation_requires_last_node_when_coordinate_frames_differ(self) -> None:
        api = FakeApi(completed=False)
        adapter = RobotAdapter(
            "ROBOT_01",
            api=api,
            nav_nodes={
                "LEFT": {"x": 0.0, "y": 0.0, "attributes": {}},
                "RIGHT": {"x": 5.0, "y": 0.0, "attributes": {}},
            },
            coordinate_transform=OffsetTransform(),
            robot_map_id="ROBOT_MAP",
        )
        # Telemetry positions are in the robot map. Comparing this directly
        # with RMF graph coordinates would silently choose an arbitrary node.
        adapter.position = [100.0, 200.0, 0.0]
        adapter.last_node_id = ""
        execution = FakeExecution()
        execution.identifier = Identifier()

        with self.assertRaisesRegex(ValueError, "lastNodeId"):
            adapter.navigate(Destination(), execution)

        self.assertEqual(api.navigate_calls, [])

    def test_arrival_uses_vda5050_completion_across_coordinate_frames(self) -> None:
        adapter, execution = self._adapter(completed=True)
        data = RobotUpdateData(
            robot_name="ROBOT_01",
            map_name="ROBOT_MAP",
            position=[105.0, 200.0, 0.0],
            battery_soc=0.8,
            last_node_id="RIGHT",
            driving=False,
        )

        adapter.update(None, data)

        self.assertEqual(execution.finished_count, 1)
        self.assertIsNone(adapter.execution)

    def test_last_node_and_stopped_are_not_enough_without_order_completion(self) -> None:
        adapter, execution = self._adapter(completed=False)
        data = RobotUpdateData(
            robot_name="ROBOT_01",
            map_name="ROBOT_MAP",
            position=[105.0, 200.0, 0.0],
            battery_soc=0.8,
            last_node_id="RIGHT",
            driving=False,
        )

        adapter.update(None, data)

        self.assertEqual(execution.finished_count, 0)
        self.assertIs(adapter.execution, execution)

    def test_rmf_stop_sends_acknowledged_cancel_order_instead_of_pause(self) -> None:
        api = FakeApi(completed=False)
        adapter = RobotAdapter("ROBOT_01", api=api)
        execution = FakeExecution()
        execution.identifier = Identifier()
        adapter.execution = execution
        adapter._nav = NavigationState(
            is_navigating=True, target_node="RIGHT", cmd_id=1,
            order_id="order_1",
        )

        adapter.stop(Identifier())

        self.assertEqual(len(api.stop_calls), 1)
        _args, kwargs = api.stop_calls[0]
        self.assertRegex(kwargs["action_id"], r"^cancel_1_")
        self.assertEqual(api.pause_calls, [])
        self.assertIsNone(adapter.execution)

    def test_new_order_waits_for_matching_cancel_action_completion(self) -> None:
        api = FakeApi(completed=False)
        adapter = RobotAdapter(
            "ROBOT_01",
            api=api,
            nav_nodes={
                "LEFT": {"x": 0.0, "y": 0.0, "attributes": {}},
                "RIGHT": {"x": 5.0, "y": 0.0, "attributes": {}},
            },
        )
        old_execution = FakeExecution()
        old_execution.identifier = Identifier()
        adapter.execution = old_execution
        adapter._nav = NavigationState(
            is_navigating=True, target_node="RIGHT", cmd_id=1,
            order_id="order_1",
        )

        adapter.stop(Identifier())
        cancel_action_id = api.stop_calls[0][1]["action_id"]
        api.command_state = RobotCommandState(
            state_received_at=time.monotonic() + 1.0,
            driving=False,
            node_states=[],
            edge_states=[],
            action_states=[],
        )
        adapter._command_hsm.pump()
        adapter.last_node_id = "LEFT"
        new_execution = FakeExecution()
        new_execution.identifier = Identifier()
        adapter.navigate(Destination(), new_execution)
        self.assertEqual(api.navigate_calls, [])

        api.command_state = RobotCommandState(
            state_received_at=time.monotonic() + 1.0,
            action_states=[SimpleNamespace(
                action_id=cancel_action_id,
                action_status="FINISHED",
            )],
            driving=True,
        )
        adapter._command_hsm.pump()
        self.assertEqual(api.navigate_calls, [])
        api.command_state.driving = False
        adapter._command_hsm.pump()

        self.assertEqual(len(api.navigate_calls), 1)


if __name__ == "__main__":
    unittest.main()
