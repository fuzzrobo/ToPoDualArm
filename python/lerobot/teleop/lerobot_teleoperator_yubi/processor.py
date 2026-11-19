"""左右の生入力と同一周期の実測値から、18軸の目標をまとめて作る。"""
import math
import time
import numpy as np
from topoarm_sdk import load_config
from lerobot.processor import RobotProcessorPipeline, ProcessorStep
from lerobot.processor.converters import robot_action_observation_to_transition, transition_to_robot_action
from lerobot.lerobot_types import TransitionKey
from .config import SIDES, ARMS, GRIPPERS, NECK, JOINTS
from .kinematics import ArmKinematics, transform


class YubiActionStep(ProcessorStep):
    def __init__(self, config, robot_config):
        config.validate()
        if robot_config.type not in ("topoarm", "topoarm_mujoco") or robot_config.joints != JOINTS:
            raise ValueError("YUBIは両腕・両グリッパー・パンチルトの18軸設定を使用してください")
        self.config = config
        data = load_config(robot_config.config_file or robot_config.actuator_config, JOINTS)
        self.limits = {joint.name: (joint.min, joint.max) for joint in data.joints}
        for lo, hi in self.limits.values():
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError("個体設定の可動域が不正です")
        self.ik, self.mount = {}, {}
        for side in SIDES:
            arm = config.arms[side]
            for value in (arm.gripper_closed_rad, arm.gripper_open_rad):
                self.check_joint(GRIPPERS[side], value)
            self.ik[side] = ArmKinematics(config.urdf_path, config.base_link, arm.tcp_link, self.limits, side=side)
            self.mount[side] = transform(arm.controller_to_tcp_xyz, arm.controller_to_tcp_xyzw)
        self.fixed_neck = [math.radians(config.neck_pan_deg), math.radians(config.neck_tilt_deg)]
        if config.neck_mode == "fixed":
            for name, value in zip(NECK, self.fixed_neck, strict=True):
                self.check_joint(name, value)
        self.world = transform([0, 0, 0], config.quest_to_robot_xyzw)
        self.last = self.head_anchor = None
        self.anchor = dict.fromkeys(SIDES)
        self.active = dict.fromkeys(SIDES, False)
        self.released = dict.fromkeys(SIDES, False)
        self.release_count = dict.fromkeys(SIDES)
        self.failed = False
        self.last_call = None

    def check_joint(self, name, value):
        lo, hi = self.limits[name]
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{name}が可動域外です")
        return value

    def input_pose(self, raw, prefix):
        value = transform([raw[f"{prefix}.p{i}"] for i in range(3)],
                          [raw[f"{prefix}.q{i}"] for i in range(4)])
        return self.world @ value @ self.mount.get(prefix, np.eye(4))

    def check_age(self, raw):
        now = time.monotonic()
        for key in ["received", *[f"{s}.opening_received" if f"{s}.opening_received" in raw
                                 else f"{s}.encoder_received" for s in SIDES]]:
            age = now-float(raw[key])
            if not 0 <= age <= self.config.input_timeout_s:
                raise TimeoutError("Quest/YUBIの入力が古いため停止します")
        return now

    def compute(self, raw, observation):
        if self.failed:
            raise RuntimeError("入力処理は停止済みです。原因を確認して起動し直してください")
        try:
            return self._compute(raw, observation)
        except Exception:
            self.failed = True
            raise

    def _compute(self, raw, observation):
        now = self.check_age(raw)
        actual = {name: self.check_joint(name, float(observation[f"{name}.pos"])) for name in JOINTS}
        hands, openings = {}, {}
        for side in SIDES:
            openings[side] = float(raw[f"{side}.opening"])
            if not math.isfinite(openings[side]) or not 0 <= openings[side] <= 1:
                raise ValueError(f"{side}のYUBI開口値が不正です")
            if (type(raw[f"{side}.clutch"]) is not bool or type(raw[f"{side}.releases"]) is not int
                    or raw[f"{side}.releases"] < 0):
                raise ValueError(f"{side}のクラッチ情報が不正です")
            hands[side] = self.input_pose(raw, side)
        head = self.input_pose(raw, "head") if self.config.neck_mode == "follow" else None
        if self.last is None:
            self.last = actual.copy()
        was_active = any(self.active.values())
        paused = self.last_call is not None and now-self.last_call > self.config.input_timeout_s
        if paused:
            self.head_anchor = None
        candidate = self.last.copy()
        for side in SIDES:
            count, clutch = raw[f"{side}.releases"], raw[f"{side}.clutch"]
            if paused or (self.release_count[side] is not None and count != self.release_count[side]):
                # 読み飛ばした解除も検出し、該当する手の押し直しを要求する。
                self.active[side] = self.released[side] = False
            self.release_count[side] = count
            if not clutch:
                self.active[side], self.released[side] = False, True
            elif self.released[side]:
                if not self.active[side]:
                    self.anchor[side] = (hands[side].copy(), self.ik[side].fk([actual[n] for n in ARMS[side]]))
                    # 押し直した側だけ実測値を基準にし、反対の腕の目標を保持する。
                    candidate.update({n: actual[n] for n in ARMS[side] + [GRIPPERS[side]]})
                    self.active[side] = True
                input_origin, tcp_origin = self.anchor[side]
                target = tcp_origin.copy()
                target[:3, 3] += self.config.position_scale*(hands[side][:3, 3]-input_origin[:3, 3])
                target[:3, :3] = hands[side][:3, :3] @ input_origin[:3, :3].T @ tcp_origin[:3, :3]
                q = self.ik[side].solve(target, [candidate[n] for n in ARMS[side]], self.config)
                candidate.update(zip(ARMS[side], map(float, q), strict=True))
                arm = self.config.arms[side]
                candidate[GRIPPERS[side]] = arm.gripper_closed_rad + openings[side]*(arm.gripper_open_rad-arm.gripper_closed_rad)
        if any(self.active.values()):
            if not was_active or self.head_anchor is None:
                self.head_anchor = (head, [actual[n] for n in NECK])
            neck = self.fixed_neck
            if head is not None:
                relative = head[:3, :3] @ self.head_anchor[0][:3, :3].T
                yaw = math.atan2(relative[1, 0], relative[0, 0])
                pitch = math.atan2(-relative[2, 0], math.hypot(relative[0, 0], relative[1, 0]))
                neck = [self.head_anchor[1][0]+self.config.neck_pan_sign*yaw,
                        self.head_anchor[1][1]+self.config.neck_tilt_sign*pitch]
            candidate.update(zip(NECK, neck, strict=True))
        else:
            self.head_anchor = None
        for name, value in candidate.items():
            self.check_joint(name, value)
        self.check_age(raw)  # 左右のIK完了後に期限を確認し、片側だけの指令を返さない。
        self.last, self.last_call = candidate, now
        return {f"{name}.pos": self.last[name] for name in JOINTS}

    def __call__(self, transition):
        result = transition.copy()
        result[TransitionKey.ACTION] = self.compute(transition[TransitionKey.ACTION], transition[TransitionKey.OBSERVATION])
        return result

    def transform_features(self, features):
        # 記録形式はrobot.action_featuresの18軸を使う。
        return features


def make_yubi_processor(config, robot_config):
    return RobotProcessorPipeline(steps=[YubiActionStep(config, robot_config)],
        to_transition=robot_action_observation_to_transition, to_output=transition_to_robot_action)
