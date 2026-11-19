"""YubiQuestAppのTCP JSON行形式を受信する（左右コントローラ）。"""
import json
import math
import socket
from .transport import LineInput
from .config import SIDES


def pose(raw, prefix, position_key, rotation_key):
    p = [raw[position_key][k] for k in ("x", "y", "z")]
    q = [raw[rotation_key][k] for k in ("x", "y", "z", "w")]
    if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in p+q):
        raise ValueError("Quest姿勢に非数値があります")
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-6:
        raise ValueError("Quest姿勢のQuaternionがゼロです")
    # Unity左手Y-up → 右手Z-up。公式プロトコルと同じ軸変換。
    p = [p[2], -p[0], p[1]]
    q = [q[2]/norm, -q[0]/norm, q[1]/norm, -q[3]/norm]
    return {**{f"{prefix}.p{i}": v for i, v in enumerate(p)},
            **{f"{prefix}.q{i}": v for i, v in enumerate(q)}}


class QuestParser:
    def __init__(self, config):
        self.config = config
        self.previous_stamp = None
        self.initial = None
        self.previous_clutch = dict.fromkeys(SIDES)
        self.releases = dict.fromkeys(SIDES, 0)

    def __call__(self, line, received):
        raw = json.loads(line)
        result = {}
        for side in SIDES:
            if raw.get(f"{side}Tracked") is not True or raw.get(f"{side}Valid") is not True:
                raise ValueError(f"{side}コントローラの追跡が無効です")
            clutch = raw[f"{side}GripPressed"]
            if type(clutch) is not bool:
                raise ValueError(f"{side}のクラッチ値が不正です")
            if not clutch and self.previous_clutch[side] is not False:
                self.releases[side] += 1
            self.previous_clutch[side] = clutch
            result.update(pose(raw, side, f"{side}ControllerPosition", f"{side}ControllerRotation"))
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
            # 現行形式にはHMD専用validフラグがない。存在する版ではfalseを拒否する。
            if raw.get("hmdTracked", True) is not True or raw.get("hmdValid", True) is not True:
                raise ValueError("HMDの追跡が無効です")
            result.update(pose(raw, "head", "hmdPosition", "hmdRotation"))
        return {**result, "received": received, "device_time_ns": stamp}


class QuestInput(LineInput):
    def __init__(self, config):
        super().__init__(QuestParser(config), config.input_timeout_s)
        self.config = config

    def connect(self):
        if not self.config.quest_host:
            raise ValueError("quest_hostを設定してください")
        self.parser = QuestParser(self.config)
        sock = socket.create_connection((self.config.quest_host, self.config.quest_port),
                                        timeout=self.config.connect_timeout_s)
        sock.settimeout(0.05)

        def read():
            try:
                return sock.recv(8192)
            except socket.timeout:
                return None
        self.start(read, sock.close, self.config.connect_timeout_s)
