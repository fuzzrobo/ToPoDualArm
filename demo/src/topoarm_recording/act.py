"""左右手首の収録データからACTを学習する。"""
import json
import math
import os
import shutil
import subprocess
import sys


def inputs_from_dataset(root):
    info = json.loads((root / "meta/info.json").read_text())
    features = info["features"]
    state = features["observation.state"]
    action = features["action"]
    names = state["names"]
    count = len(names)
    if count not in (16, 18) or len(set(names)) != count or action["names"] != names or any(
        feature["shape"] != [count] for feature in (state, action)
    ):
        raise ValueError("実測角と指令角には、同じ順番の16軸または18軸を指定してください")
    if info["total_episodes"] < 1:
        raise ValueError("採用済みのエピソードがありません")
    inputs = {"observation.state": {"type": "STATE", "shape": [count]}}
    for side in ("left", "right"):
        key = f"observation.images.{side}"
        image = features.get(key, {})
        if image.get("dtype") not in ("image", "video") or image.get("shape") != [240, 320, 3]:
            raise ValueError(f"{key}に320×240のRGB画像が必要です")
        inputs[key] = {"type": "VISUAL", "shape": [3, 240, 320]}
    return names, inputs


def write_initial_pose(root, names, output):
    from lerobot.datasets import LeRobotDataset

    dataset = LeRobotDataset("local/topoarm", root=root, video_backend="pyav")
    first_state = dataset[0]["observation.state"].tolist()
    if len(first_state) != len(names) or not all(math.isfinite(value) for value in first_state):
        raise ValueError("収録の先頭フレームに有効な全軸角度がありません")
    pose = dict(zip(names, first_state, strict=True))
    output.write_text(json.dumps(pose, indent=2, allow_nan=False) + "\n")


def train(dataset, output, steps, device):
    names, features = inputs_from_dataset(dataset)
    backbone = os.environ.get("ACT_BACKBONE_WEIGHTS") or "ResNet18_Weights.IMAGENET1K_V1"
    # Python環境を変えず、LeRobotの学習CLIを呼び出す。
    subprocess.run([
        sys.executable, "-m", "lerobot.scripts.lerobot_train",
        "--dataset.repo_id=local/topoarm", f"--dataset.root={dataset}", "--dataset.video_backend=pyav",
        "--policy.type=act", f"--policy.device={device}", "--policy.push_to_hub=false",
        f"--policy.input_features={json.dumps(features, separators=(',', ':'))}",
        "--policy.vision_backbone=resnet18", f"--policy.pretrained_backbone_weights={backbone}",
        f"--steps={steps}", "--batch_size=8", "--num_workers=0", f"--save_freq={steps}", "--log_freq=100",
        "--wandb.enable=false", f"--output_dir={output}",
    ], check=True, env={**os.environ, "HF_HUB_OFFLINE": "1", "WANDB_MODE": "disabled"})
    # 学習データの関節順、収録周波数、開始姿勢を重みと一緒に渡す。
    checkpoint = output / "checkpoints/last/pretrained_model"
    shutil.copyfile(dataset / "meta/info.json", checkpoint / "topoarm_schema.json")
    write_initial_pose(dataset, names, checkpoint / "initial_pose.json")
