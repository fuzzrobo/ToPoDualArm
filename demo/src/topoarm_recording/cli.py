"""収録・ACT学習・推論を起動する。"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml

from topoarm_recording import ROOT
from topoarm_recording.act import train
from topoarm_recording.config import DEFAULT_CONFIG, read_config, settings, training_output


def default_dataset(record, folder):
    root = folder / record["root"]
    sources = [root, root / "vsido"]
    if record.get("input") == "vsido":
        sources.reverse()
    for source in sources:
        metadata = source / "meta/info.json"
        if metadata.is_file() and json.loads(metadata.read_text())["total_episodes"] > 0:
            return source
    return sources[0]


def main():
    parser = argparse.ArgumentParser(description="ToPoDualArm ACTデモ")
    commands = parser.add_subparsers(dest="command", required=True)
    record = commands.add_parser("record", help="収録GUIを開く")
    record.add_argument("input_config", type=Path, help="入力設定YAML（例 config/quest.yaml）")
    record.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="共通のデモ設定YAML")
    infer = commands.add_parser("infer", help="推論GUIを開く")
    infer.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="デモ設定YAML")
    learning = commands.add_parser("train", aliases=["act-train"], help="ローカルのLeRobotデータからACTを学習する")
    learning.add_argument("dataset", nargs="?", type=Path, help="収録データのディレクトリ")
    learning.add_argument("output", nargs="?", type=Path, help="学習結果の保存先")
    learning.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="デモ設定YAML")
    learning.add_argument("--steps", type=int)
    learning.add_argument("--device", help="学習デバイス。既定はCUDAが使えればcuda、それ以外はcpu")
    args = parser.parse_args()
    if args.command in ("record", "infer"):
        viewer = ROOT / "viewer/record_quest.py"
        flag = "--record" if args.command == "record" else "--inference"
        command = [sys.executable, str(viewer), flag, str(args.config)]
        if args.command == "record":
            command += ["--input-config", str(args.input_config)]
        return os.execv(sys.executable, command)

    try:
        config, folder = read_config(args.config)
        train_settings = settings(config, "train")
        steps = args.steps if args.steps is not None else train_settings.get("steps", 20000)
        if type(steps) is not int or steps < 1:
            learning.error("stepsは1以上の整数を指定してください")
        dataset = args.dataset or default_dataset(settings(config, "record"), folder)
        output = args.output or training_output(config, folder)
        device = args.device or os.environ.get("ACT_DEVICE") or train_settings.get("device")
        if not device:
            import torch

            device = "cuda" if torch.cuda.is_available() else "cpu"
        train(dataset, output, steps, device)
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode) from None
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        learning.error(str(error))
