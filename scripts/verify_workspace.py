#!/usr/bin/env python3
"""Check the local RMF/VDA5050 simulation workspace without changing it."""

from __future__ import annotations

import ast
import importlib.util
import json
import py_compile
import shutil
import socket
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM_ROOT = ROOT / "rmf_dev_tool-main" / "vda5050_robot_simulator"
GUI_ROOT = ROOT / "rmf_dev_tool-main" / "vda5050_gui"
PLATFORM_ROOT = ROOT / "rmf_platform-main"


def mark(ok: bool) -> str:
    return "OK" if ok else "MISSING/FAIL"


def check_file(path: Path, required: bool = True) -> bool:
    ok = path.is_file()
    label = "required" if required else "optional"
    print(f"[{mark(ok)}] {path.relative_to(ROOT)} ({label})")
    return ok


def check_path(path: Path, required: bool = True) -> bool:
    ok = path.exists()
    label = "required" if required else "optional"
    print(f"[{mark(ok)}] {path.relative_to(ROOT)} ({label})")
    return ok


def check_python_module(name: str) -> bool:
    try:
        ok = importlib.util.find_spec(name) is not None
    except (ImportError, ModuleNotFoundError):
        ok = False
    print(f"[{mark(ok)}] Python module: {name}")
    return ok


def check_mqtt(host: str = "127.0.0.1", port: int = 1883) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.5):
            ok = True
    except OSError:
        ok = False
    print(f"[{mark(ok)}] MQTT broker: {host}:{port}")
    return ok


def main() -> int:
    print("\n== Robot Simulator files ==")
    simulator_files = [
        SIM_ROOT / "run.py",
        SIM_ROOT / "requirements.txt",
        SIM_ROOT / "p4_scenario.yaml",
        SIM_ROOT / "vda5050_simulator" / "__init__.py",
        SIM_ROOT / "vda5050_simulator" / "models.py",
        SIM_ROOT / "vda5050_simulator" / "mqtt_client.py",
        SIM_ROOT / "vda5050_simulator" / "robot.py",
        SIM_ROOT / "vda5050_simulator" / "order_manager.py",
        SIM_ROOT / "vda5050_simulator" / "action_handler.py",
        SIM_ROOT / "vda5050_simulator" / "state_publisher.py",
    ]
    sim_files_ok = all(check_file(path) for path in simulator_files)

    print("\n== Python syntax ==")
    syntax_ok = True
    for path in sorted((ROOT / "rmf_dev_tool-main").rglob("*.py")):
        try:
            py_compile.compile(str(path), doraise=True)
            print(f"[OK] {path.relative_to(ROOT)}")
        except py_compile.PyCompileError as exc:
            syntax_ok = False
            print(f"[FAIL] {path.relative_to(ROOT)}: {exc.msg}")

    print("\n== Config formats ==")
    config_ok = True
    try:
        json.loads((GUI_ROOT / "gui_settings.json").read_text(encoding="utf-8"))
        print("[OK] GUI JSON")
    except Exception as exc:
        config_ok = False
        print(f"[FAIL] GUI JSON: {exc}")

    try:
        import yaml
        yaml.safe_load((SIM_ROOT / "p4_scenario.yaml").read_text(encoding="utf-8"))
        yaml.safe_load((PLATFORM_ROOT / "docker-compose.yml").read_text(encoding="utf-8"))
        print("[OK] YAML files")
    except Exception as exc:
        config_ok = False
        print(f"[FAIL] YAML files: {exc}")

    for name in ("cyclonedds.xml", "cyclonedds_rmf.xml"):
        try:
            ET.parse(PLATFORM_ROOT / name)
            print(f"[OK] {name}")
        except Exception as exc:
            config_ok = False
            print(f"[FAIL] {name}: {exc}")

    print("\n== GUI completeness ==")
    gui_path = GUI_ROOT / "vda5050_gui.py"
    gui_complete = False
    if check_file(gui_path):
        tree = ast.parse(gui_path.read_text(encoding="utf-8"))
        class_names = {node.name for node in tree.body if isinstance(node, ast.ClassDef)}
        function_names = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
        required_classes = {"SimulationTab", "MainWindow"}
        gui_complete = required_classes.issubset(class_names) and "main" in function_names
        print(f"[{mark(gui_complete)}] SimulationTab + MainWindow + main()")

    print("\n== Runtime dependencies ==")
    yaml_ok = check_python_module("yaml")
    paho_ok = check_python_module("paho")
    pyqt_ok = check_python_module("PyQt5")
    docker_ok = shutil.which("docker") is not None
    print(f"[{mark(docker_ok)}] Command: docker")
    mqtt_ok = check_mqtt()

    print("\n== RMF full-stack inputs ==")
    rmf_inputs = [
        PLATFORM_ROOT / "McAfee_Certificate.crt",
        PLATFORM_ROOT / "src" / "rmf_core",
        PLATFORM_ROOT / "src" / "rmf_vda5050_fleet_adapter" / "map" / "map_dsr_0427.yaml",
        PLATFORM_ROOT / "src" / "rmf_vda5050_fleet_adapter" / "building_map" / "map_dsr.building.yml",
        PLATFORM_ROOT / "src" / "rmf_vda5050_fleet_adapter" / "vda5050_fleet_adapter" / "config" / "config.yaml",
        PLATFORM_ROOT / "src" / "rmf_battery_management",
        PLATFORM_ROOT / "src" / "rmf_commission_manager",
        PLATFORM_ROOT / "src" / "rmf_vda5050_rmf_bridge",
        PLATFORM_ROOT / "src" / "rmf_dev_tool" / "rmf_web_custom" / "packages" / "api-server" / "sqlite_local_config.py",
    ]
    rmf_ready = all(check_path(path) for path in rmf_inputs)

    print("\n== Summary ==")
    simulator_ready = sim_files_ok and syntax_ok and config_ok and yaml_ok and paho_ok and mqtt_ok
    print(f"Robot Simulator ready: {simulator_ready}")
    print(f"GUI ready: {gui_complete and syntax_ok and yaml_ok and paho_ok and pyqt_ok}")
    print(f"RMF full stack ready: {rmf_ready and docker_ok}")
    return 0 if simulator_ready else 1


if __name__ == "__main__":
    sys.exit(main())
