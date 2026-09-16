from __future__ import annotations

from pathlib import Path
import unittest

from traffic_control.corridor_registry import CorridorRegistry


ROOT = Path(__file__).resolve().parents[1]


class ConfigurationFileTests(unittest.TestCase):
    def test_default_configuration_is_safe_disabled(self) -> None:
        registry = CorridorRegistry.from_yaml(ROOT / "config" / "corridor_blocks.yaml")

        self.assertFalse(registry.enabled)
        self.assertEqual(registry.blocks, {})

    def test_segmented_example_has_three_independent_blocks(self) -> None:
        registry = CorridorRegistry.from_yaml(
            ROOT / "config" / "corridor_blocks_segmented_example.yaml"
        )

        self.assertTrue(registry.enabled)
        self.assertEqual(set(registry.blocks), {"TOP_1", "TOP_2", "TOP_3"})
        self.assertEqual(
            {block.direction_domain for block in registry.blocks.values()},
            {"TOP_1", "TOP_2", "TOP_3"},
        )
        self.assertEqual(len(registry.routes), 2)


if __name__ == "__main__":
    unittest.main()
