"""実機とV4L2の3台のカメラから1エピソード収録する。"""
import argparse
from contextlib import ExitStack
from pathlib import Path
from tempfile import TemporaryDirectory
import time

import draccus
from lerobot.datasets import LeRobotDataset
import numpy as np

from lerobot_teleoperator_yubi import YubiTeleoperatorConfig
from topoarm_sdk import Device
from topoarm_recording import ROOT
from topoarm_recording.config import DEFAULT_CONFIG, camera_devices, device_arguments, read_config, settings
from topoarm_recording.inputs import QuestReader, calibrated_input, inputs, quest_configuration
from sensors import rgb_reader

CAMERAS = ("front", "left", "right")
FPS = 20
HEIGHT, WIDTH = 240, 320


def input_config(mode, document, record, robot):
    if mode == "leader":
        return None
    if "record" not in document:
        cfg = draccus.decode(YubiTeleoperatorConfig, document)
        cfg.urdf_path = robot.cfg.options.robot_model
        cfg.validate()
        return cfg

    quest = {}
    if mode in ("meta", "quest"):
        quest = record["quest"] if "quest" in record else read_config(ROOT / "config/quest.yaml")[0]["quest"]
    glove = {}
    if mode in ("glove", "quest"):
        glove = record["glove"] if "glove" in record else read_config(ROOT / "config/glove.yaml")[0]["glove"]
    if mode == "meta":
        options = quest
    elif mode == "glove":
        return calibrated_input(glove, robot.cfg)
    else:
        options = {**quest, **glove}
        quest_arms, glove_arms = quest.get("arms", {}), glove.get("arms", {})
        options["arms"] = {side: {**quest_arms.get(side, {}), **glove_arms.get(side, {})}
                           for side in quest_arms.keys() | glove_arms.keys()}
        options["hardware_verified"] = (quest.get("hardware_verified") is True
                                        and glove.get("hardware_verified") is True)
    return quest_configuration(options, robot.cfg)[0]


def record(args):
    if not args.allow_hardware:
        raise ValueError("実機への収録には --allow-hardware を指定してください")
    if args.root.exists():
        raise FileExistsError(f"新しい保存先を指定してください: {args.root}")
    document, folder = read_config(args.config)
    options = settings(document, "record")
    robot_args = device_arguments(args.robot or options.get("robot"), Path.cwd() if args.robot else folder)
    cameras = camera_devices(args.cameras or options.get("cameras"), Path.cwd() if args.cameras else folder)
    leader = args.leader
    if args.input == "leader" and leader is None:
        profile, profile_folder = (options, folder) if "leader" in options else read_config(ROOT / "config/u2d2.yaml")
        leader = device_arguments(profile["leader"], profile_folder)
    robot = Device(**robot_args, role="follower")
    cfg = input_config(args.input, document, options, robot)
    args.root.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=args.root.parent) as temporary:
        pending = Path(temporary) / "episode"
        with ExitStack() as cleanup:
            cleanup.enter_context(robot)
            names = list(robot.joints)
            images_at_tick = rgb_reader(cleanup, cameras)
            pose = robot.read().position
            read = inputs(args.input, cfg, cleanup, pose, names, robot.hardware, leader,
                          follower_port=robot.cfg.port)
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
    rec.add_argument("--robot", type=Path)
    rec.add_argument("--leader", type=Path)
    rec.add_argument("--cameras", type=Path)
    rec.add_argument("--allow-hardware", action="store_true")
    rec.add_argument("--frames", type=int, default=40)
    rec.add_argument("--task", default="Open and close both grippers")
    rec.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("framesは1以上を指定してください")
    record(args)
