"""収録GUIとCLIで共用するリーダー・Quest・グローブの入力。"""
from types import SimpleNamespace

from lerobot_teleoperator_yubi import YubiTeleoperator
from lerobot_teleoperator_yubi.gripper import GripperInput
from lerobot_teleoperator_yubi.joints import SIDES
from topoarm_sdk import Device


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
            self.step.active[side] = False


def inputs(mode, cfg, cleanup, pose, names, hardware, leader_config, *,
           button_event=None, position_first=False, direct_quest_pose=None,
           measure_ik_error=False, ik_period=None):
    if mode == "leader":
        if not leader_config:
            raise ValueError("--leader に読み取り専用の個体YAMLを指定してください")
        from topoarm_leader_follower import LeaderFollowerTransform
        leader = Device(leader_config)
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
        robot = SimpleNamespace(type="topoarm", joints=JOINTS, config_file=str(hardware))
        step = YubiActionStep(cfg, robot, position_first=position_first, direct_quest_pose=direct_quest_pose,
                              measure_ik_error=measure_ik_error, position_first_dt=ik_period)
        if mode == "meta":
            if not cfg.hardware_verified:
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
