"""左右手首デモの学習入力と初期姿勢を、収録データから取り出す。"""
import argparse
import json
from pathlib import Path

from topoarm_recording.act import inputs_from_dataset, write_initial_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--initial-pose", type=Path)
    args = parser.parse_args()
    names, inputs = inputs_from_dataset(args.dataset)
    if args.initial_pose is None:
        print(json.dumps(inputs, separators=(",", ":")))
    else:
        write_initial_pose(args.dataset, names, args.initial_pose)


if __name__ == "__main__":
    main()
