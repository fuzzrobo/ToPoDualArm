"""V-SidoのUDP状態をLeRobotで使う関節順・単位へ変換する。"""
from dataclasses import dataclass
import math
from pathlib import Path
import socket
import time

from topoarm_recording.config import read_config
from topoarm_sdk import decode_vsido_status


@dataclass(frozen=True)
class VsidoSample:
    received: float
    position: dict
    goal: dict
    leader_rad: dict
    raw: str


def decode_status(payload, received, follower_ids, leader_ids):
    raw = payload.decode("ascii")
    axes = decode_vsido_status(payload)
    missing = sorted(set(follower_ids.values()) - axes.keys())
    if missing:
        raise ValueError(f"UDP statusのフォロワーID欠落 {missing}")
    position = {name: axes[i][0] for name, i in follower_ids.items()}
    goal = {name: axes[i][1] for name, i in follower_ids.items()}
    leader_rad = {name: axes[i][0] for name, i in leader_ids.items() if i in axes}
    return VsidoSample(received, position, goal, leader_rad, raw)


class VsidoReceiver:
    def __init__(self, config, follower_ids, leader_ids):
        cfg = {} if config is None else config
        if isinstance(cfg, (Path, str)):
            cfg, _ = read_config(cfg)
        if not isinstance(cfg, dict):
            raise ValueError("V-Sidoには設定マップを指定してください")
        self.follower_ids = self._ids(cfg.get("follower_ids", follower_ids), follower_ids)
        self.leader_ids = self._ids(cfg.get("leader_ids", leader_ids), leader_ids)
        if set(self.follower_ids.values()) & set(self.leader_ids.values()):
            raise ValueError("V-SidoのリーダーとフォロワーのIDが重複しています")
        self.port = cfg.get("port", 8886)
        self.fps = cfg.get("fps", 100)
        self.sender = cfg.get("sender")
        self.bind = cfg.get("bind", "0.0.0.0")
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("V-Sidoの受信ポートが不正です")
        if type(self.fps) is not int or not 1 <= self.fps <= 100:
            raise ValueError("V-Sidoの収録fpsは1〜100で指定してください")
        self.error = ""
        self.last_received = -math.inf
        self.last_leader_received = -math.inf
        self.socket = None

    @staticmethod
    def _ids(mapping, known):
        if not isinstance(mapping, dict) or not mapping or not mapping.keys() <= known.keys():
            raise ValueError("V-SidoのID表には関節名を指定してください")
        ids = tuple(mapping.values())
        if any(type(i) is not int or not 1 <= i <= 253 for i in ids) or len(set(ids)) != len(ids):
            raise ValueError("V-SidoのID表に無効または重複したIDがあります")
        return {name: mapping[name] for name in known if name in mapping}

    def __enter__(self):
        with_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            with_socket.bind((self.bind, self.port))
            with_socket.setblocking(False)
        except BaseException:
            with_socket.close()
            raise
        self.socket = with_socket
        return self

    def __exit__(self, *exc):
        self.socket.close()
        self.socket = None

    def poll(self):
        latest = None
        for _ in range(32):
            try:
                payload, (sender, _) = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            received = time.monotonic()
            if self.sender and sender != self.sender:
                continue
            try:
                latest = decode_status(payload, received, self.follower_ids, self.leader_ids)
            except ValueError as exc:
                self.error = str(exc)
            else:
                self.last_received = received
                if latest.leader_rad:
                    self.last_leader_received = received
                self.error = ""
        return latest
