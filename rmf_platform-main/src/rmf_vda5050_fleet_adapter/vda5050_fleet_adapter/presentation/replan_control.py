"""Opt-in simulation control: request RMF replan through the upstream API."""
from typing import Any, Mapping

def request_replan(robots: Mapping[str, Any], robot_name: str) -> bool:
    robot = robots.get(robot_name)
    if robot is None or robot.update_handle is None:
        return False
    core = robot.update_handle.more()
    if core is None:
        return False
    core.replan()
    return True
