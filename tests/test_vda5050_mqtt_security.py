from __future__ import annotations

import os
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch


ROOT = Path(__file__).resolve().parents[1]
ADAPTER_ROOT = ROOT / "rmf_platform-main/src/rmf_vda5050_fleet_adapter"
sys.path.insert(0, str(ADAPTER_ROOT))

from vda5050_fleet_adapter.infra.mqtt.mqtt_client import MqttClient  # noqa: E402
from vda5050_fleet_adapter.presentation.main import mqtt_config_from_mapping  # noqa: E402
from vda5050_fleet_adapter.usecase.ports.config_port import MqttConfig  # noqa: E402


class FleetAdapterMqttSecurityTests(unittest.TestCase):
    @patch("vda5050_fleet_adapter.infra.mqtt.mqtt_client.mqtt.Client")
    def test_client_applies_password_and_verified_mtls(self, client_type) -> None:
        client = client_type.return_value
        config = MqttConfig(
            broker_host="broker.example",
            broker_port=8883,
            username="operator",
            password="secret",
            ca_file="/certs/ca.pem",
            cert_file="/certs/client.pem",
            key_file="/certs/client.key",
            tls_required=True,
        )

        MqttClient(config)

        client.username_pw_set.assert_called_once_with("operator", "secret")
        client.tls_set.assert_called_once_with(
            ca_certs="/certs/ca.pem",
            certfile="/certs/client.pem",
            keyfile="/certs/client.key",
        )
        client.tls_insecure_set.assert_not_called()

    @patch("vda5050_fleet_adapter.infra.mqtt.mqtt_client.mqtt.Client")
    def test_client_rejects_incomplete_mtls_pair(self, client_type) -> None:
        with self.assertRaisesRegex(ValueError, "certificate and key"):
            MqttClient(MqttConfig(
                broker_host="broker.example",
                broker_port=8883,
                ca_file="/certs/ca.pem",
                cert_file="/certs/client.pem",
                tls_required=True,
            ))
        client_type.return_value.connect.assert_not_called()

    def test_mapping_resolves_credentials_from_environment(self) -> None:
        manager = {
            "ip": "broker.example",
            "port": 8883,
            "security": {
                "username_env": "FAB_MQTT_USER",
                "password_env": "FAB_MQTT_PASSWORD",
                "ca_file": "/certs/ca.pem",
                "client_cert_file": "/certs/client.pem",
                "client_key_file": "/certs/client.key",
                "tls_required": True,
            },
        }

        config = mqtt_config_from_mapping(manager, {
            "FAB_MQTT_USER": "operator",
            "FAB_MQTT_PASSWORD": "secret",
        })

        self.assertEqual(config.username, "operator")
        self.assertEqual(config.password, "secret")
        self.assertTrue(config.tls_required)

    def test_mapping_reports_missing_reference_without_secret_values(self) -> None:
        manager = {
            "security": {
                "username_env": "FAB_MQTT_USER",
                "password_env": "FAB_MQTT_PASSWORD",
                "tls_required": True,
            }
        }
        environment = dict(os.environ)
        environment["FAB_MQTT_USER"] = "operator-secret-value"
        environment.pop("FAB_MQTT_PASSWORD", None)

        with self.assertRaises(ValueError) as context:
            mqtt_config_from_mapping(manager, environment)

        rendered = str(context.exception)
        self.assertIn("FAB_MQTT_PASSWORD", rendered)
        self.assertNotIn("operator-secret-value", rendered)


if __name__ == "__main__":
    unittest.main()
