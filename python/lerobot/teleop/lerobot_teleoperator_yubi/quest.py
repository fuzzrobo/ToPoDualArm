"""YubiQuestAppのTCP JSON行形式を受信する（左右コントローラ）。"""
import json
import math
import socket
import numpy as np
from scipy.spatial.transform import Rotation
from .transport import LineInput
from .config import SIDES


def pose(raw, position_key, rotation_key):
    p = [raw[position_key][k] for k in ("x", "y", "z")]
    q = [raw[rotation_key][k] for k in ("x", "y", "z", "w")]
    if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in p+q):
        raise ValueError("Quest姿勢に非数値があります")
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-6:
        raise ValueError("Quest姿勢のQuaternionがゼロです")
    # Unity左手Y-up → 右手Z-up。公式プロトコルと同じ軸変換。
    p = np.array([p[2], -p[0], p[1]])
    q = [q[2]/norm, -q[0]/norm, q[1]/norm, -q[3]/norm]
    return p, Rotation.from_quat(q)


def pose_fields(prefix, position, rotation):
    return {**{f"{prefix}.p{i}": v for i, v in enumerate(position)},
            **{f"{prefix}.q{i}": v for i, v in enumerate(rotation.as_quat())}}


class QuestParser:
    def __init__(self, config, *, allow_untracked=False, button_event=None):
        self.config = config
        self.button_event = button_event
        self.allow_untracked = allow_untracked
        self.tracked = dict.fromkeys(SIDES, False)
        self.losses = dict.fromkeys(SIDES, 0)
        self.buttons = {}
        self.previous_stamp = None
        self.initial = None
        self.previous_clutch = dict.fromkeys(SIDES)
        self.releases = dict.fromkeys(SIDES, 0)

    def __call__(self, line, received):
        raw = json.loads(line)
        result = {}
        head_position = head_rotation = head_inverse = None
        if raw.get("hmdTracked", True) is True and raw.get("hmdValid", True) is True:
            try:
                head_position, head_rotation = pose(raw, "hmdPosition", "hmdRotation")
            except (KeyError, TypeError, ValueError) as exc:
                if not self.allow_untracked:
                    raise ValueError("HMDの姿勢を取得できません") from exc
            else:
                head_inverse = head_rotation.inv()
        if head_inverse is None and not self.allow_untracked:
            raise ValueError("HMDの追跡が無効です")
        for side in SIDES:
            if (head_inverse is None or raw.get(f"{side}Tracked") is not True
                    or raw.get(f"{side}Valid") is not True):
                if self.allow_untracked:
                    if self.tracked[side]:
                        self.losses[side] += 1
                    self.tracked[side] = False
                    result[f"{side}.tracked"] = False
                    continue
                raise ValueError(f"{side}コントローラの追跡が無効です")
            if self.allow_untracked:
                self.tracked[side] = True
                result[f"{side}.tracked"] = True
                result[f"{side}.losses"] = self.losses[side]
            clutch = raw[f"{side}GripPressed"]
            if type(clutch) is not bool:
                raise ValueError(f"{side}のクラッチ値が不正です")
            if not clutch and self.previous_clutch[side] is not False:
                self.releases[side] += 1
            self.previous_clutch[side] = clutch
            position, rotation = pose(raw, f"{side}ControllerPosition", f"{side}ControllerRotation")
            # コントローラーの位置と向きを、同一フレームのHMD座標で表す。
            result.update(pose_fields(side, head_inverse.apply(position - head_position),
                                      head_inverse * rotation))
            result[f"{side}.clutch"] = clutch
            if f"{side}TriggerPressed" in raw:
                trigger = raw[f"{side}TriggerPressed"]
                if type(trigger) is not bool:
                    raise ValueError("トリガー値が不正です")
                result[f"{side}.trigger"] = trigger
            result[f"{side}.releases"] = self.releases[side]
        stamp = raw["ovrTimeNs"]
        if type(stamp) is not int or stamp <= 0:
            raise ValueError("Questの時刻が不正です")
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            raise ValueError("Questの時刻が逆行・重複しました")
        if self.initial is None:
            self.initial = (received, stamp)
        lag = (received - self.initial[0]) - (stamp - self.initial[1]) / 1e9
        if abs(lag) > self.config.input_timeout_s:
            raise ValueError("Quest時刻に遅延または不連続があります")
        self.previous_stamp = stamp
        if self.config.neck_mode == "follow":
            # 首は従来どおりQuestの固定座標系でHMDの向きの変化を見る。
            if head_rotation is None:
                result["head.tracked"] = False
            else:
                result.update(pose_fields("head", head_position, head_rotation))
        if self.button_event:
            for button in ("A", "B"):
                pressed = raw.get(f"button{button}Pressed")
                if type(pressed) is bool:
                    if pressed and self.buttons.get(button) is False:
                        self.button_event(received, button)
                    self.buttons[button] = pressed
        return {**result, "received": received, "device_time_ns": stamp}


class QuestInput(LineInput):
    def __init__(self, config, *, allow_untracked=False, button_event=None):
        self.button_event = button_event
        self.allow_untracked = allow_untracked
        super().__init__(QuestParser(config, allow_untracked=allow_untracked), config.input_timeout_s)
        self.config = config

    def connect(self):
        if not self.config.quest_host:
            raise ValueError("quest_hostを設定してください")
        self.parser = QuestParser(self.config, allow_untracked=self.allow_untracked, button_event=self.button_event)
        sock = socket.create_connection((self.config.quest_host, self.config.quest_port),
                                        timeout=self.config.connect_timeout_s)
        sock.settimeout(0.05)

        def read():
            try:
                return sock.recv(8192)
            except socket.timeout:
                return None
        self.start(read, sock.close, self.config.connect_timeout_s)
