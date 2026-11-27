"""左右の生入力と同一周期の実測値から、構成に応じた関節目標をまとめて作る。"""
import math
import time
from xml.etree import ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from topoarm_sdk import load_config
from lerobot.processor import RobotProcessorPipeline, ProcessorStep
from lerobot.processor.converters import robot_action_observation_to_transition, transition_to_robot_action
from lerobot.lerobot_types import TransitionKey
from .joints import SIDES, ARMS, GRIPPERS, NECK, ARM_JOINTS, JOINTS
from .kinematics import ArmKinematics, IKTargetUnreachable, POSITION_FIRST_REFERENCE_DT, transform


class YubiActionStep(ProcessorStep):
    def __init__(self, config, robot_config, *, position_first=False, direct_quest_pose=None,
                 measure_ik_error=False, position_first_dt=None):
        config.validate()
        self.joints = tuple(robot_config.joints)
        if (robot_config.type != "topoarm" or len(set(self.joints)) != len(self.joints)
                or set(self.joints) not in (set(ARM_JOINTS), set(JOINTS))):
            raise ValueError("YUBIには両腕・両グリッパーの16軸、またはパンチルト付きの18軸を指定してください")
        self.has_pan_tilt = len(self.joints) == len(JOINTS)
        self.config = config
        self.position_first = position_first
        data = load_config(robot_config.config_file or robot_config.actuator_config, list(self.joints))
        self.clip_targets = data.options.clip_to_limits
        self.limits = {joint.name: (joint.min, joint.max) for joint in data.joints}
        for lo, hi in self.limits.values():
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError("個体設定の可動域が不正です")
        self.ik, self.mount = {}, {}
        urdf_root = None
        if config.urdf_path == data.options.robot_model and data.options.robot_model_fragment:
            urdf_root = ET.fromstring(data.options.robot_urdf())
        for side in SIDES:
            arm = config.arms[side]
            for value in (arm.gripper_closed_rad, arm.gripper_open_rad):
                self.check_joint(GRIPPERS[side], value)
            self.ik[side] = ArmKinematics(config.urdf_path, config.base_link, arm.tcp_link, self.limits,
                                         side=side, urdf_root=urdf_root)
            self.mount[side] = transform(arm.controller_to_tcp_xyz, arm.controller_to_tcp_xyzw)
        self.quest_origin = None
        if direct_quest_pose is not None:
            neck_pivot = direct_quest_pose["neck_pivot_m"]
            frame_offset = direct_quest_pose["frame_offset_m"]
            config.validate_transform(neck_pivot, 3, "neck_pivot_m")
            config.validate_transform(frame_offset, 3, "frame_offset_m")
            shoulders = [arm.shoulder_origin() for arm in self.ik.values()]
            self.quest_origin = np.mean(shoulders, axis=0) + frame_offset
            # 旧OpenArmのWebXR→ロボット変換。受信済みのHMD相対姿勢はロボット軸に変換済み。
            webxr_to_robot = Rotation.from_matrix([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
            self.quest_neck_pivot = webxr_to_robot.apply(neck_pivot)
            controller_to_tcp = webxr_to_robot * Rotation.from_euler("z", 90, degrees=True)
            convention = transform([0, 0, 0], controller_to_tcp.as_quat())
            for side in SIDES:
                self.mount[side] = convention @ self.mount[side]
        self.fixed_neck = [math.radians(config.neck_pan_deg), math.radians(config.neck_tilt_deg)]
        if self.has_pan_tilt and config.neck_mode == "fixed":
            for name, value in zip(NECK, self.fixed_neck, strict=True):
                self.check_joint(name, value)
        self.world = transform([0, 0, 0], config.quest_to_robot_xyzw)
        self.calibrated_head_from_world = None
        self.waiting_for_world = False
        self.resume_after_switch = dict.fromkeys(SIDES, False)
        self.last = self.head_anchor = None
        self.anchor = dict.fromkeys(SIDES)
        self.active = dict.fromkeys(SIDES, False)
        self.release_count = dict.fromkeys(SIDES)
        self.losses = dict.fromkeys(SIDES, 0)
        self.ik_hold = dict.fromkeys(SIDES, False)
        self.measure_ik_error = measure_ik_error
        self.ik_error = {}
        self.ik_positions = {}
        self.position_first_dt = POSITION_FIRST_REFERENCE_DT if position_first_dt is None else position_first_dt
        self.previous_ik_interval = self.position_first_dt
        self.stalls = dict.fromkeys(SIDES, 0)
        self.ik_stall_started = dict.fromkeys(SIDES)
        self.previous_seed = dict.fromkeys(SIDES)
        self.failed = False
        self.last_call = None

    def check_joint(self, name, value):
        lo, hi = self.limits[name]
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{name}が可動域外です: {math.degrees(value):.2f}°（{math.degrees(lo):.2f}〜{math.degrees(hi):.2f}°）")
        return value

    def command_angle(self, name, value):
        if self.clip_targets and math.isfinite(value):
            lo, hi = self.limits[name]
            return max(lo, min(hi, value))
        return self.check_joint(name, value)

    @staticmethod
    def tracked_pose(raw, prefix):
        return transform([raw[f"{prefix}.p{i}"] for i in range(3)],
                         [raw[f"{prefix}.q{i}"] for i in range(4)])

    @property
    def world_mode(self):
        return self.calibrated_head_from_world is not None

    def input_pose(self, raw, prefix):
        direct = self.quest_origin is not None and prefix in self.mount
        if direct and self.world_mode:
            value = self.calibrated_head_from_world @ self.tracked_pose(raw, f"{prefix}.world")
        else:
            value = self.tracked_pose(raw, prefix)
        if direct:
            value[:3, 3] = self.config.position_scale * (value[:3, 3] - self.quest_neck_pivot)
        value = self.world @ value @ self.mount.get(prefix, np.eye(4))
        if direct:
            value[:3, 3] += self.quest_origin
        return value

    def check_age(self, raw):
        now = time.monotonic()
        for key in ["received", *[f"{s}.opening_received" if f"{s}.opening_received" in raw
                                 else f"{s}.encoder_received" for s in SIDES if raw.get(f"{s}.tracked", True)]]:
            age = now-float(raw[key])
            if not 0 <= age <= self.config.input_timeout_s:
                raise TimeoutError("Quest/YUBIの入力が古いため停止します")
        return now

    def compute(self, raw, observation, *, world_requested=False):
        if self.failed:
            raise RuntimeError("入力処理は停止済みです。原因を確認して起動し直してください")
        try:
            return self._compute(raw, observation, world_requested=world_requested)
        except Exception:
            self.failed = True
            raise

    def _compute(self, raw, observation, *, world_requested):
        now = self.check_age(raw)
        self.ik_error.clear()
        self.ik_positions.clear()
        actual = {}
        for name in self.joints:
            value = float(observation[f"{name}.pos"])
            if not math.isfinite(value):
                raise ValueError(f"{name}の実測が非有限値です")
            actual[name] = value
        if self.last is None:
            self.last = {n: self.command_angle(n, v) for n, v in actual.items()}
        if self.quest_origin is not None:
            if self.last_call is None:
                self.waiting_for_world = world_requested
            if world_requested != self.world_mode and all(raw.get(f"{side}.clutch") is False for side in SIDES):
                # 現在のHMD基準を部屋に固定し、同じ位置合わせを左右で共用する。
                self.calibrated_head_from_world = (
                    np.linalg.inv(self.tracked_pose(raw, "head")) if world_requested else None
                )
                self.resume_after_switch = dict.fromkeys(SIDES, True)
            self.waiting_for_world = self.waiting_for_world and world_requested and not self.world_mode
            if self.waiting_for_world:
                # ワールドを選択した状態での再接続時は、位置合わせまで現在指令を保持する。
                self.last_call = now
                return {f"{name}.pos": self.last[name] for name in self.joints}
        hands, openings = {}, {}
        for side in SIDES:
            if raw.get(f"{side}.tracked") is False:
                continue
            openings[side] = float(raw[f"{side}.opening"])
            if not math.isfinite(openings[side]) or not 0 <= openings[side] <= 1:
                raise ValueError(f"{side}のYUBI開口値が不正です")
            if (type(raw[f"{side}.clutch"]) is not bool or type(raw[f"{side}.releases"]) is not int
                    or raw[f"{side}.releases"] < 0):
                raise ValueError(f"{side}のクラッチ情報が不正です")
            hands[side] = self.input_pose(raw, side)
        head = None
        if self.has_pan_tilt and self.config.neck_mode == "follow" and raw.get("head.tracked", True):
            head = self.input_pose(raw, "head")
        was_active = any(self.active.values())
        elapsed = now - self.last_call if self.last_call is not None else self.position_first_dt
        paused = elapsed > self.config.input_timeout_s
        # 遅れた分を一度に取り戻す指令は出さない。
        ik_dt = min(elapsed, POSITION_FIRST_REFERENCE_DT)
        if paused:
            self.head_anchor = None
        candidate = self.last.copy()
        for side in SIDES:
            previously_active = self.active[side]
            if not self.active[side] or paused:
                self.stalls[side] = 0
            self.ik_hold[side] = False
            if side not in hands:
                self.active[side] = False
                continue
            losses = raw.get(f"{side}.losses", 0)
            recovered = losses != self.losses[side]
            self.losses[side] = losses
            count, clutch = raw[f"{side}.releases"], raw[f"{side}.clutch"]
            if recovered or paused or count != self.release_count[side]:
                # 途切れた側のIK履歴と相対操作の起点だけを取り直す。
                self.active[side] = False
            self.release_count[side] = count
            if not clutch:
                self.active[side] = False
                if previously_active:
                    self.resume_after_switch[side] = False
            else:
                target = hands[side]
                if self.quest_origin is None:
                    if not self.active[side]:
                        origin = candidate if recovered else actual
                        self.anchor[side] = (hands[side].copy(), self.ik[side].fk([origin[n] for n in ARMS[side]]))
                        # 押し直した側だけ実測値を基準にし、反対の腕の目標を保持する。
                        candidate.update({n: self.command_angle(n, origin[n]) for n in ARMS[side] + [GRIPPERS[side]]})
                    input_origin, tcp_origin = self.anchor[side]
                    target = tcp_origin.copy()
                    target[:3, 3] += self.config.position_scale*(hands[side][:3, 3]-input_origin[:3, 3])
                    target[:3, :3] = hands[side][:3, :3] @ input_origin[:3, :3].T @ tcp_origin[:3, :3]
                if not self.active[side]:
                    self.previous_seed[side] = None
                    self.stalls[side] = 0
                    self.ik_stall_started[side] = None
                self.active[side] = True
                kin = self.ik[side]
                if self.measure_ik_error:
                    tcp = kin.fk([actual[n] for n in ARMS[side]])
                    position_mm = 1000 * np.linalg.norm(target[:3, 3] - tcp[:3, 3])
                    rotation = Rotation.from_matrix(target[:3, :3] @ tcp[:3, :3].T)
                    self.ik_error[side] = (position_mm, math.degrees(rotation.magnitude()))
                    self.ik_positions[side] = (*target[:3, 3], *tcp[:3, 3])
                try:
                    seed = [candidate[n] for n in ARMS[side]]
                    if self.position_first or self.resume_after_switch[side]:
                        seed = np.asarray(seed)
                        prev = self.previous_seed[side]
                        self.previous_seed[side] = seed
                        previous_step = None if prev is None else seed-prev
                        interval = self.position_first_dt if prev is None else ik_dt
                        velocity = None if previous_step is None else previous_step / self.previous_ik_interval
                        q = kin.solve_position_first(target, seed, self.config, dt=interval,
                                                     previous_velocity=velocity)
                    else:
                        q = kin.solve(target, seed, self.config)
                except IKTargetUnreachable:
                    # 単発の停滞でも指令は直ちに保持。表示と解除閾値は持続した停滞にのみ適用。
                    self.stalls[side] += 1
                    if self.ik_stall_started[side] is None:
                        self.ik_stall_started[side] = now
                    self.ik_hold[side] = (not self.position_first or
                                         now - self.ik_stall_started[side] >= 2 * POSITION_FIRST_REFERENCE_DT)
                    candidate.update({name: self.last[name] for name in ARMS[side]})
                    candidate[GRIPPERS[side]] = self.last[GRIPPERS[side]]
                    continue
                else:
                    self.stalls[side] = 0
                    self.ik_stall_started[side] = None
                    candidate.update(zip(ARMS[side], map(float, q), strict=True))
                arm = self.config.arms[side]
                candidate[GRIPPERS[side]] = arm.gripper_closed_rad + openings[side]*(arm.gripper_open_rad-arm.gripper_closed_rad)
        if self.has_pan_tilt and any(self.active.values()):
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
            candidate[name] = self.command_angle(name, value)
        self.check_age(raw)  # 左右のIK完了後に期限を確認し、片側だけの指令を返さない。
        self.previous_ik_interval = elapsed
        self.last, self.last_call = candidate, now
        return {f"{name}.pos": self.last[name] for name in self.joints}

    def __call__(self, transition):
        result = transition.copy()
        result[TransitionKey.ACTION] = self.compute(transition[TransitionKey.ACTION], transition[TransitionKey.OBSERVATION])
        return result

    def transform_features(self, features):
        # 収録範囲は機体設定の関節一覧に従う。
        return features


def make_yubi_processor(config, robot_config):
    return RobotProcessorPipeline(steps=[YubiActionStep(config, robot_config)],
        to_transition=robot_action_observation_to_transition, to_output=transition_to_robot_action)
