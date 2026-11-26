"""左右手首デモの学習入力と初期姿勢を、収録データから取り出す。"""
import argparse
import json
import math
from pathlib import Path


def inputs_from_dataset(root):
    info = json.loads((root / "meta/info.json").read_text())
    features = info["features"]
    state = features["observation.state"]
    action = features["action"]
    names = state["names"]
    if state["shape"] != [18] or action["shape"] != [18] or action["names"] != names or len(set(names)) != 18:
        raise ValueError("実測角と指令角に同じ順番の18関節が必要です")
    if info["total_episodes"] < 1:
        raise ValueError("採用済みのエピソードがありません")
    inputs = {"observation.state": {"type": "STATE", "shape": [18]}}
    for side in ("left", "right"):
        key = f"observation.images.{side}"
        image = features.get(key, {})
        if image.get("dtype") not in ("image", "video") or image.get("shape") != [240, 320, 3]:
            raise ValueError(f"{key}に320×240のRGB画像が必要です")
        inputs[key] = {"type": "VISUAL", "shape": [3, 240, 320]}
    return names, inputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--initial-pose", type=Path)
    args = parser.parse_args()
    names, inputs = inputs_from_dataset(args.dataset)
    if args.initial_pose is None:
        print(json.dumps(inputs, separators=(",", ":")))
        return

    from lerobot.datasets import LeRobotDataset

    dataset = LeRobotDataset("local/topoarm", root=args.dataset, video_backend="pyav")
    first_state = dataset[0]["observation.state"].tolist()
    if len(first_state) != len(names) or not all(math.isfinite(value) for value in first_state):
        raise ValueError("収録の先頭フレームに有効な全軸角度がありません")
    pose = dict(zip(names, first_state, strict=True))
    args.initial_pose.write_text(json.dumps(pose, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
