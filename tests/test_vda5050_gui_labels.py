from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / "rmf_dev_tool-main/vda5050_gui/vda5050_gui.py"


def load_gui_module():
    spec = importlib.util.spec_from_file_location("vda5050_gui_under_test", GUI)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Vda5050GuiLabelTests(unittest.TestCase):
    def test_common_map_prefix_is_hidden_only_in_display_labels(self) -> None:
        gui = load_gui_module()
        self.assertEqual(
            gui._common_node_label_prefix(["CHAIN_C1_A", "CHAIN_C2_A", "CHAIN_R1"]),
            "CHAIN_",
        )
        self.assertEqual(gui._display_node_label("CHAIN_SIDE_1", "CHAIN_"), "SIDE_1")
        self.assertEqual(gui._display_node_label("P4", ""), "P4")


if __name__ == "__main__":
    unittest.main()
