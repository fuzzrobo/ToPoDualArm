"""操作元・ロボットを選んで1エピソード収録する。再生先はMuJoCoに限定する。"""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace

import draccus
from lerobot.datasets import LeRobotDataset
import numpy as np

from lerobot_teleoperator_yubi import YubiTeleoperator, YubiTeleoperatorConfig
from lerobot_teleoperator_yubi.gripper import GripperInput, opening
from lerobot_teleoperator_yubi.joints import SIDES
from topoarm_sdk import Device
from topoarm_recording import recording_target
from sensors import rgb_reader

ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ("front", "left", "right")
FPS = 20
HEIGHT, WIDTH = 240, 320


DEFAULT_ROBOT = ROOT / "config/robots/mujoco.yaml"


class QuestReader:
    def __init__(self, step, receive, names):
        self.step, self.receive, self.names = step, receive, names
        self.tracking = {}

    def __call__(self, index, state):
        observed = {name + ".pos": state[name] for name in self.names}
        raw = self.receive()
        self.tracking = {s: bool(raw.get(f"{s}.tracked", True)) for s in SIDES}
        self.tracking["received"] = raw["received"]
        action = self.step.compute(raw, observed)
        return {name: action[name + ".pos"] for name in self.names}

    def accepted(self, applied):
        self.step.last = dict(applied)

    def pause(self, applied):
        self.accepted(applied)
        self.step.head_anchor = None
        for side in SIDES:
            self.step.active[side] = self.step.released[side] = False


def inputs(mode, cfg, cleanup, pose, names, hardware, leader_config, *, real_robot,
           button_event=None, position_first=False, direct_quest_pose=None):
    if mode == "leader":
        if not leader_config:
            raise ValueError("--leader に読み取り専用の個体YAMLを指定してください")
        from topoarm_leader_follower import LeaderFollowerTransform
        leader = recording_target(leader_config)
        if not leader.read_only:
            raise ValueError("リーダーのroleはleaderにしてください")
        cleanup.enter_context(leader)
        transform = LeaderFollowerTransform({k: k for k in names}, neck_mode="fixed")
        return lambda index, state: transform.targets(leader.read().position)
    if mode in ("meta", "quest"):
        from lerobot_teleoperator_yubi.processor import YubiActionStep
        from lerobot_teleoperator_yubi.joints import JOINTS
        if set(names) != set(JOINTS):
            raise ValueError("Meta入力には18軸モデルを指定してください")
        robot = SimpleNamespace(type="topoarm" if real_robot else "topoarm_mujoco",
                                joints=JOINTS, config_file=str(hardware))
        step = YubiActionStep(cfg, robot, position_first=position_first, direct_quest_pose=direct_quest_pose)
        if mode == "meta":
            if real_robot and not cfg.hardware_verified:
                raise ValueError("Metaの取付け・座標変換を確認してください")
            from lerobot_teleoperator_yubi.quest import QuestInput
            device = QuestInput(cfg, allow_untracked=True, button_event=button_event)
            cleanup.callback(device.close)
            device.connect()
            def receive():
                raw = dict(device.snapshot())
                for side in SIDES:
                    if raw.get(f"{side}.tracked") is False:
                        continue
                    raw[f"{side}.opening"] = 0.0 if raw[f"{side}.trigger"] else cfg.meta_trigger_released_opening
                    raw[f"{side}.opening_received"] = raw["received"]
                return raw
        else:
            device = YubiTeleoperator(cfg)
            device.connect()
            cleanup.callback(device.disconnect)
            receive = device.get_action

        return QuestReader(step, receive, names)

    ports = {}
    if mode == "glove":
        if not cfg.hardware_verified:
            raise ValueError("左右のポートと開閉端を校正し、Yubi設定のhardware_verifiedを有効にしてください")
        for side in SIDES:
            port = GripperInput(cfg.arms[side], input_timeout_s=cfg.input_timeout_s,
                                connect_timeout_s=cfg.connect_timeout_s)
            cleanup.callback(port.close)
            port.connect()
            ports[side] = port

    def read_gloves(index, state):
        target = pose.copy()
        for side in SIDES:
            arm = cfg.arms[side]
            if mode == "mock":
                phase = index / FPS * (4 if side == "left" else 3)
                fraction = 0.5 * (1 - math.cos(phase))
                angle = arm.encoder_closed_rad + fraction * (arm.encoder_open_rad - arm.encoder_closed_rad)
                _, fraction = opening(f"{arm.encoder_id},{angle}", arm)
            else:
                fraction = ports[side].snapshot()["opening"]
            target[side[0].upper() + "_gripper_joint"] = (
                arm.gripper_closed_rad + fraction * (arm.gripper_open_rad - arm.gripper_closed_rad))
        return target
    return read_gloves


def record(args):
    if args.root.exists():
        raise FileExistsError(f"新しい保存先を指定してください: {args.root}")
    cfg = (None if args.input == "leader" else
           draccus.decode(YubiTeleoperatorConfig, json.loads(args.config.read_text())))
    args.root.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=args.root.parent) as temporary:
        pending = Path(temporary) / "episode"
        with ExitStack() as cleanup:
            robot = recording_target(args.robot)
            if isinstance(robot, Device) and (args.input == "mock" or not args.allow_hardware):
                raise ValueError("実機への収録は実入力と --allow-hardware を明示してください")
            cleanup.enter_context(robot)
            names = list(robot.joints)
            if not isinstance(robot, Device) and abs(robot.dt - 1/FPS) > 1e-9:
                raise ValueError("収録には20 Hzのロボット設定が必要です")
            images_at_tick = rgb_reader(robot, cleanup, args.cameras)
            pose = robot.read().position
            if cfg is not None:
                from topoarm_sdk import load_config
                cfg.urdf_path = load_config(str(robot.hardware)).options.robot_model
                cfg.validate()
            read = inputs(args.input, cfg, cleanup, pose, names, robot.hardware, args.leader, real_robot=isinstance(robot, Device))
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
                if args.input != "mock":
                    time.sleep(max(0, 1 / FPS - (time.monotonic() - started)))
            robot.disable()
            dataset.save_episode()
        pending.rename(args.root)
    print(f"保存: {args.root} ({args.frames}フレーム)")


def replay(args):
    dataset = LeRobotDataset("local/topoarm", root=args.root, video_backend="pyav")
    if dataset.num_episodes != 1 or len(dataset) == 0:
        raise ValueError("1エピソードのデータセットを指定してください")
    error = 0.
    with ExitStack() as cleanup:
        sim = recording_target(args.robot)
        if isinstance(sim, Device) or abs(sim.dt - 1/dataset.fps) > 1e-9:
            raise ValueError("再生は収録周期と一致するMuJoCo専用です")
        if args.viewer:
            sim.sim.show_viewer = True
        cleanup.enter_context(sim)
        sim.enable()
        if any(list(dataset.features[key]["names"]) != list(sim.joints) for key in ("action", "observation.state")):
            raise ValueError("収録とシミュレータの関節順序が異なります")
        for index in range(len(dataset)):
            row = dataset[index]
            state = sim.read().position
            actual = np.array([state[n] for n in sim.joints])
            expected = row["observation.state"].numpy()
            if not np.isfinite(actual).all() or not np.isfinite(expected).all():
                raise ValueError("関節状態に非有限値があります")
            error = max(error, float(np.max(np.abs(actual - expected))))
            for name in CAMERAS:
                if tuple(row["observation.images." + name].shape) != (3, HEIGHT, WIDTH):
                    raise ValueError(f"{name}の画像サイズが不正です")
            sim.command(dict(zip(sim.joints, row["action"].tolist(), strict=True)))
            if args.viewer:
                time.sleep(1 / dataset.fps)
    print(f"再生: {len(dataset)}フレーム、18軸、3カメラ、最大状態誤差 {error:.6g} rad")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    rec = commands.add_parser("record")
    rec.add_argument("root", type=Path)
    rec.add_argument("--input", choices=["mock", "glove", "meta", "quest", "leader"], default="mock")
    rec.add_argument("--robot", type=Path, default=DEFAULT_ROBOT)
    rec.add_argument("--leader", type=Path)
    rec.add_argument("--cameras", type=Path)
    rec.add_argument("--allow-hardware", action="store_true")
    rec.add_argument("--frames", type=int, default=40)
    rec.add_argument("--task", default="Open and close both grippers")
    rec.add_argument("--config", type=Path, default=ROOT / "config/yubi.json")
    play = commands.add_parser("replay")
    play.add_argument("root", type=Path)
    play.add_argument("--robot", type=Path, default=DEFAULT_ROBOT)
    play.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    if args.command == "record" and args.frames < 1:
        parser.error("framesは1以上を指定してください")
    (record if args.command == "record" else replay)(args)
