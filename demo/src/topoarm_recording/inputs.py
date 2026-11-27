"""収録GUIとCLIで共用するリーダー・Quest・グローブの入力。"""
from types import SimpleNamespace

import draccus

from lerobot_teleoperator_yubi import YubiTeleoperator, YubiTeleoperatorConfig
from lerobot_teleoperator_yubi.gripper import GripperInput
from lerobot_teleoperator_yubi.joints import GRIPPERS, NECK, SIDES
from topoarm_recording import ROOT
from topoarm_recording.config import read_config, settings
from topoarm_sdk import Device


QUEST_DEFAULTS = ROOT.parent / "config/teleop/quest.yaml"


def quest_configuration(source, hardware):
    defaults, _ = read_config(QUEST_DEFAULTS)
    overrides = source if isinstance(source, dict) else read_config(source)[0]
    if "record" in overrides:
        overrides = settings(overrides, "record").get("quest", {})
    options = {**defaults, **overrides}
    pose = options.pop("pose")
    if not isinstance(pose, dict) or not {"neck_pivot_m", "frame_offset_m"} <= pose.keys():
        raise ValueError("record.quest.poseにneck_pivot_mとframe_offset_mを指定してください")

    cfg = calibrated_input(options, hardware)
    for name in ("neck_pivot_m", "frame_offset_m"):
        cfg.validate_transform(pose[name], 3, name)
    return cfg, pose


def calibrated_input(source, hardware):
    options = source.copy() if isinstance(source, dict) else read_config(source)[0]
    arms = options.get("arms", {})
    if not isinstance(arms, dict) or not arms.keys() <= GRIPPERS.keys():
        raise ValueError("armsにはright・leftを指定してください")
    joints = {j.name: j for j in hardware.joints}
    options["arms"] = {}
    for side, name in GRIPPERS.items():
        joint = joints[name]
        closed, opened = (joint.min, joint.max) if side == "right" else (joint.max, joint.min)
        options["arms"][side] = {
            "tcp_link": f"{side[0].upper()}_tcp",
            **arms.get(side, {}),
            "gripper_closed_rad": closed,
            "gripper_open_rad": opened,
        }
    options["urdf_path"] = hardware.options.robot_model
    cfg = draccus.decode(YubiTeleoperatorConfig, options)
    cfg.validate()
    return cfg


class QuestReader:
    def __init__(self, step, receive, names):
        self.step, self.receive, self.names = step, receive, names
        self.tracking = {}

    def __call__(self, index, state, *, world_requested=False):
        observed = {name + ".pos": state[name] for name in self.names}
        raw = self.receive()
        self.tracking = {s: bool(raw.get(f"{s}.tracked", True)) for s in SIDES}
        self.tracking["received"] = raw["received"]
        action = self.step.compute(raw, observed, world_requested=world_requested)
        return {name: action[name + ".pos"] for name in self.names}

    def accepted(self, applied):
        self.step.last = dict(applied)

    def pause(self, applied):
        self.accepted(applied)
        self.step.head_anchor = None
        for side in SIDES:
            self.step.active[side] = False


def inputs(mode, cfg, cleanup, pose, names, hardware, leader_config, *,
           button_event=None, position_first=False, direct_quest_pose=None,
           measure_ik_error=False, ik_period=None, follower_port=None):
    if mode == "leader":
        if not leader_config:
            raise ValueError("--leader に読み取り専用の個体YAMLを指定してください")
        from topoarm_leader_follower import LeaderFollowerTransform
        connection = leader_config if isinstance(leader_config, dict) else {"path": leader_config}
        leader = Device(**connection, role="leader")
        if not leader.read_only:
            raise ValueError("リーダーのroleはleaderにしてください")
        if leader.cfg.port == follower_port:
            raise ValueError("リーダーとフォロワーには別のポートを指定してください")
        neck_mode = "fixed" if set(NECK) <= set(names) else None
        transform = LeaderFollowerTransform({k: k for k in names}, neck_mode=neck_mode)
        if not set(transform.source_joints) <= set(leader.joints):
            raise ValueError("リーダーの関節設定がフォロワーと一致しません")
        cleanup.enter_context(leader)
        return lambda index, state: transform.targets(leader.read().position)
    if mode in ("meta", "quest"):
        from lerobot_teleoperator_yubi.processor import YubiActionStep
        robot = SimpleNamespace(type="topoarm", joints=names, config_file=str(hardware))
        step = YubiActionStep(cfg, robot, position_first=position_first, direct_quest_pose=direct_quest_pose,
                              measure_ik_error=measure_ik_error, position_first_dt=ik_period)
        if mode == "meta":
            if not cfg.hardware_verified:
                raise ValueError("Metaの取付け・座標変換を確認してください")
            from lerobot_teleoperator_yubi.quest import QuestInput
            device = QuestInput(cfg, allow_untracked=True, button_event=button_event,
                                include_world=direct_quest_pose is not None)
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

    if mode != "glove":
        raise ValueError(f"未対応の入力です: {mode}")
    if not cfg.hardware_verified:
        raise ValueError("左右のポートと開閉端を校正し、Yubi設定のhardware_verifiedを有効にしてください")
    ports = {}
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
            fraction = ports[side].snapshot()["opening"]
            target[side[0].upper() + "_gripper_joint"] = (
                arm.gripper_closed_rad + fraction * (arm.gripper_open_rad - arm.gripper_closed_rad))
        return target
    return read_gloves
