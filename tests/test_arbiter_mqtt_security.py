from __future__ import annotations

from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

from traffic_control.deployment import MqttDeploymentConfig, SecretRef
from traffic_control.robot_tracker import MqttStateMonitor


class ArbiterMqttSecurityTests(unittest.TestCase):
    @patch("paho.mqtt.client.Client")
    def test_monitor_applies_security_and_subscribes_to_state_and_connection(
        self, client_type
    ) -> None:
        client = client_type.return_value
        config = MqttDeploymentConfig(
            host="broker.example",
            port=8883,
            keepalive_sec=30,
            reconnect_max_delay_sec=20,
            username=SecretRef(env="FAB_MQTT_USER"),
            password=SecretRef(env="FAB_MQTT_PASSWORD"),
            ca_file=Path("/certs/ca.pem"),
            cert_file=Path("/certs/client.pem"),
            key_file=Path("/certs/client.key"),
            tls_required=True,
        )
        monitor = MqttStateMonitor(
            MagicMock(),
            host="ignored",
            port=1883,
            topic="uagv/v2.0.0/vendor/+/state",
            mqtt_config=config,
            environ={
                "FAB_MQTT_USER": "operator",
                "FAB_MQTT_PASSWORD": "secret",
            },
        )

        client.username_pw_set.assert_called_once_with("operator", "secret")
        client.tls_set.assert_called_once_with(
            ca_certs="/certs/ca.pem",
            certfile="/certs/client.pem",
            keyfile="/certs/client.key",
        )
        client.tls_insecure_set.assert_not_called()
        self.assertFalse(monitor.is_connected)

        monitor._on_connect(client, None, None, 0)

        self.assertTrue(monitor.is_connected)
        self.assertEqual(
            [(call.args[0], call.kwargs["qos"]) for call in client.subscribe.call_args_list],
            [
                ("uagv/v2.0.0/vendor/+/state", 1),
                ("uagv/v2.0.0/vendor/+/connection", 1),
            ],
        )

        monitor._on_disconnect(client, None, None, 1)
        self.assertFalse(monitor.is_connected)

    @patch("paho.mqtt.client.Client")
    def test_monitor_preserves_anonymous_plain_mqtt_for_simulation(
        self, client_type
    ) -> None:
        client = client_type.return_value

        monitor = MqttStateMonitor(
            MagicMock(), host="127.0.0.1", port=1883, topic="sim/+/state"
        )

        client.username_pw_set.assert_not_called()
        client.tls_set.assert_not_called()
        monitor.start()
        client.connect.assert_called_once_with("127.0.0.1", 1883, keepalive=60)


if __name__ == "__main__":
    unittest.main()
