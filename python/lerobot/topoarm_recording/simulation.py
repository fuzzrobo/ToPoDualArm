"""1指令で設定周期だけ進めるMuJoCoバックエンド。"""
import math
import time
import yaml
from topoarm_mujoco import Simulation


from topoarm_sdk._session import PositionSession


class SimTarget(PositionSession):
    def __init__(self, cfg, root, *, scene=None, step_on_command=True, cameras=("front", "left", "right")):
        self.hardware = (root / cfg["hardware"]).resolve()
        rate = float(cfg["hz"])
        if not math.isfinite(rate) or not 1 <= rate <= 1000:
            raise ValueError("シミュレーション周期が不正です")
        self.step_on_command = step_on_command
        self.dt = 1 / rate
        self.sim = Simulation(self.hardware, simulation_config=root / cfg["simulation"],
                              scene_config=scene, width=320, height=240, cameras=cameras)
        super().__init__(self.sim.names, cfg["role"], cfg.get("state_timeout_s", .25))
        self.home = yaml.safe_load((root / cfg["home"]).read_text())

    def _connect(self):
        self.sim.connect()
        h = self.home
        pose = {**h["home_rad"], **h["neck_hold_rad"]}
        for side in ("left", "right"):
            pose[side[0].upper()+"_gripper_joint"] = math.radians(h["gripper_deg"][side]["closed"])
        self.sim.reset_from_state(pose, dict.fromkeys(pose, 0.))

    def _read(self):
        q = self.sim.state()
        return {n: v["position"] for n, v in q.items()}, time.monotonic_ns()

    def _command(self, q):
        accepted = self.sim.command(q)
        if self.step_on_command:
            self.sim.step(self.dt)
        return accepted

    def _enable(self):
        pass

    def _disable(self):
        pass

    def _close(self):
        self.sim.close()
