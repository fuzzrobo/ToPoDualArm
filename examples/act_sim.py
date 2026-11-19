"""学習済みACTと正規化器を読み、MuJoCoの観測だけで閉ループ推論する。"""
import argparse
import json
from pathlib import Path
import torch
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from topoarm_sdk import Device
from topoarm_recording import recording_target

ROOT = Path(__file__).resolve().parents[1]


@torch.inference_mode()
def run(checkpoint, robot_path, steps):
    policy = ACTPolicy.from_pretrained(str(checkpoint), local_files_only=True)
    policy.eval()
    policy.reset()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(checkpoint))
    schema = json.loads((checkpoint/"topoarm_schema.json").read_text())
    robot = recording_target(robot_path)
    if isinstance(robot, Device):
        raise ValueError("学習直後の方策の試験先はMuJoCoに限定します")
    if any(schema["features"][k]["names"] != list(robot.joints) for k in ("observation.state", "action")):
        raise ValueError("学習時と関節順序が違います")
    if abs(robot.dt - 1/schema["fps"]) > 1e-9:
        raise ValueError("学習時と制御周期が違います")
    with robot:
        robot.enable()
        for _ in range(steps):
            obs = {"observation.images."+n: torch.from_numpy(img.copy()).permute(2,0,1).float()/255.
                   for n, img in robot.sim.images().items()}
            q = robot.read().position
            obs["observation.state"] = torch.tensor([q[n] for n in robot.joints], dtype=torch.float32)
            target = post(policy.select_action(pre(obs))).reshape(-1).tolist()
            result = robot.command(dict(zip(robot.joints, target, strict=True)))
    print(f"ACT → MuJoCo: {steps}周期、{len(result)}軸、学習済み重み: {checkpoint}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("checkpoint", type=Path)
    p.add_argument("--robot", type=Path, default=ROOT/"config/robots/mujoco.yaml")
    p.add_argument("--steps", type=int, default=8)
    a = p.parse_args()
    if a.steps <= 0:
        p.error("stepsは正にしてください")
    run(a.checkpoint, a.robot, a.steps)
