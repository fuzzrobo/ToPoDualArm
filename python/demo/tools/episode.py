"""実機とV4L2の3台のカメラから1エピソード収録する。"""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time

import draccus
from lerobot.datasets import LeRobotDataset
import numpy as np

from lerobot_teleoperator_yubi import YubiTeleoperatorConfig
from topoarm_sdk import Device
from topoarm_recording import ROOT
from topoarm_recording.inputs import QuestReader, inputs
from sensors import rgb_reader

CAMERAS = ("front", "left", "right")
FPS = 20
HEIGHT, WIDTH = 240, 320


def record(args):
    if not args.allow_hardware:
        raise ValueError("実機への収録には --allow-hardware を指定してください")
    if args.root.exists():
        raise FileExistsError(f"新しい保存先を指定してください: {args.root}")
    cfg = (None if args.input == "leader" else
           draccus.decode(YubiTeleoperatorConfig, json.loads(args.config.read_text())))
    args.root.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=args.root.parent) as temporary:
        pending = Path(temporary) / "episode"
        with ExitStack() as cleanup:
            robot = Device(args.robot)
            cleanup.enter_context(robot)
            names = list(robot.joints)
            images_at_tick = rgb_reader(cleanup, args.cameras)
            pose = robot.read().position
            if cfg is not None:
                cfg.urdf_path = robot.cfg.options.robot_model
                cfg.validate()
            read = inputs(args.input, cfg, cleanup, pose, names, robot.hardware, args.leader)
            features = {key: {"dtype": "float32", "shape": (len(names),), "names": names}
                        for key in ("action", "observation.state")}
            for camera in CAMERAS:
                features["observation.images." + camera] = {
                    "dtype": "image", "shape": (HEIGHT, WIDTH, 3), "names": ["height", "width", "channels"]}
            dataset = LeRobotDataset.create("local/topoarm", FPS, features, root=pending,
                                            robot_type="topoarm", use_videos=False)
            cleanup.callback(dataset.finalize)
            robot.enable()
            for index in range(args.frames):
                started = time.monotonic()
                images = images_at_tick()
                state = robot.read().position
                applied = robot.command(read(index, state))
                if isinstance(read, QuestReader):
                    read.accepted(applied)
                frame = {"observation.state": np.array([state[n] for n in names], np.float32),
                         "action": np.array([applied[n] for n in names], np.float32),
                         "task": args.task}
                frame.update({"observation.images." + name: images[name] for name in CAMERAS})
                dataset.add_frame(frame)
                time.sleep(max(0, 1 / FPS - (time.monotonic() - started)))
            robot.disable()
            dataset.save_episode()
        pending.rename(args.root)
    print(f"保存: {args.root} ({args.frames}フレーム)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    rec = commands.add_parser("record")
    rec.add_argument("root", type=Path)
    rec.add_argument("--input", choices=["glove", "meta", "quest", "leader"], required=True)
    rec.add_argument("--robot", type=Path, required=True)
    rec.add_argument("--leader", type=Path)
    rec.add_argument("--cameras", type=Path, required=True)
    rec.add_argument("--allow-hardware", action="store_true")
    rec.add_argument("--frames", type=int, default=40)
    rec.add_argument("--task", default="Open and close both grippers")
    rec.add_argument("--config", type=Path, default=ROOT / "config/yubi.json")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("framesは1以上を指定してください")
    record(args)
