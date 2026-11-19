"""LeRobot収録の操作先。ROS 2は独立した制御経路。"""
from pathlib import Path
import yaml
from topoarm_sdk import Device


def recording_target(path):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    if cfg["backend"] == "direct":
        return Device(path)
    if cfg["backend"] == "mujoco":
        from .simulation import SimTarget
        return SimTarget(cfg, path.parent)
    raise ValueError("LeRobotの操作先はdirectまたはmujocoです")
