from __future__ import annotations

import json
import unittest

from traffic_control.corridor_registry import CorridorRegistry
from traffic_control.direction_arbiter import DirectionArbiter
from traffic_control.models import Decision, Direction
from traffic_control.robot_tracker import MqttStateMonitor, RobotTracker


def make_registry() -> CorridorRegistry:
    return CorridorRegistry.from_dict(
        {
            "traffic_control": {"enabled": True},
            "holding_bays": {
                "HB_A": {
                    "node_id": "A",
                    "geometry": {"circle": {"x": 0.0, "y": 0.0, "radius": 0.5}},
                },
                "HB_B": {
                    "node_id": "B",
                    "geometry": {"circle": {"x": 10.0, "y": 0.0, "radius": 0.5}},
                },
                "HB_C": {
                    "node_id": "C",
                    "geometry": {"circle": {"x": 20.0, "y": 0.0, "radius": 0.5}},
                },
                "HB_D": {
                    "node_id": "D",
                    "geometry": {"circle": {"x": 30.0, "y": 0.0, "radius": 0.5}},
                },
            },
            "blocks": [
                {
                    "id": "GEOMETRY_BLOCK",
                    "entry_a": "HB_A",
                    "entry_b": "HB_B",
                    "geometry": {
                        "bounds": {
                            "min_x": 4.0,
                            "max_x": 6.0,
                            "min_y": -1.0,
                            "max_y": 1.0,
                        }
                    },
                    "edges_a_to_b": ["A>B"],
                    "edges_b_to_a": ["B>A"],
                },
                {
                    "id": "EDGE_BLOCK",
                    "entry_a": "HB_C",
                    "entry_b": "HB_D",
                    "geometry": {
                        "bounds": {
                            "min_x": 24.0,
                            "max_x": 26.0,
                            "min_y": -1.0,
                            "max_y": 1.0,
                        }
                    },
                    "edges_a_to_b": ["C>D"],
                    "edges_b_to_a": ["D>C"],
                },
            ],
        }
    )


def state_payload(
    *,
    robot_id: str,
    header_id: int,
    last_node_id: str,
    x: float,
    y: float,
    driving: bool,
    edge_ids: tuple[str, ...] = (),
) -> dict:
    return {
        "headerId": header_id,
        "timestamp": f"2026-09-22T00:00:{header_id:02d}Z",
        "manufacturer": "YujinRobot",
        "serialNumber": robot_id,
        "lastNodeId": last_node_id,
        "driving": driving,
        "operatingMode": "AUTOMATIC",
        "paused": False,
        "agvPosition": {
            "x": x,
            "y": y,
            "mapId": "L1",
            "positionInitialized": True,
        },
        "safetyState": {"eStop": "NONE", "fieldViolation": False},
        "nodeStates": [],
        "edgeStates": [{"edgeId": item} for item in edge_ids],
        "errors": [],
    }


class FakeMessage:
    def __init__(self, topic: str, payload: dict, *, retain: bool) -> None:
        self.topic = topic
        self.payload = json.dumps(payload).encode("utf-8")
        self.retain = retain


class DsrFieldHardeningTests(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = make_registry()
        self.arbiter = DirectionArbiter(self.registry)
        self.tracker = RobotTracker(self.registry, self.arbiter)

    def test_retained_state_is_ignored_by_mqtt_monitor(self) -> None:
        monitor = MqttStateMonitor(
            self.tracker,
            host="127.0.0.1",
            port=1883,
            topic="uagv/v2/YujinRobot/+/state",
        )
        message = FakeMessage(
            "uagv/v2/YujinRobot/SIM_A/state",
            state_payload(
                robot_id="SIM_A",
                header_id=1,
                last_node_id="A",
                x=0.0,
                y=0.0,
                driving=False,
            ),
            retain=True,
        )

        monitor._on_message(None, None, message)

        self.assertNotIn("SIM_A", self.tracker.snapshot())

    def test_live_state_is_ingested(self) -> None:
        monitor = MqttStateMonitor(
            self.tracker,
            host="127.0.0.1",
            port=1883,
            topic="uagv/v2/YujinRobot/+/state",
        )
        message = FakeMessage(
            "uagv/v2/YujinRobot/SIM_A/state",
            state_payload(
                robot_id="SIM_A",
                header_id=2,
                last_node_id="A",
                x=0.0,
                y=0.0,
                driving=False,
            ),
            retain=False,
        )

        monitor._on_message(None, None, message)

        state = self.tracker.snapshot()["SIM_A"]
        self.assertEqual(state["last_node_id"], "A")
        self.assertEqual(state["current_hb"], "HB_A")

    def test_driving_robot_is_not_a_safe_stop_even_if_last_node_is_holding_bay(self) -> None:
        self.tracker.ingest_state(
            "SIM_A",
            state_payload(
                robot_id="SIM_A",
                header_id=3,
                last_node_id="A",
                x=0.0,
                y=0.0,
                driving=True,
            ),
        )

        self.assertIsNone(self.tracker.current_safe_node("SIM_A"))

    def test_edge_and_active_grant_override_conflicting_geometry(self) -> None:
        decision = self.arbiter.request(
            "SIM_A",
            "EDGE_BLOCK",
            Direction.A_TO_B,
            "HB_D",
            source_hb="HB_C",
        )
        self.assertIs(decision, Decision.ADMIT)

        self.tracker.ingest_state(
            "SIM_A",
            state_payload(
                robot_id="SIM_A",
                header_id=4,
                last_node_id="C",
                x=5.0,
                y=0.0,
                driving=True,
                edge_ids=("C>D",),
            ),
        )

        state = self.tracker.snapshot()["SIM_A"]
        self.assertEqual(state["current_block"], "EDGE_BLOCK")
        self.assertIsNone(state["current_hb"])


if __name__ == "__main__":
    unittest.main()
