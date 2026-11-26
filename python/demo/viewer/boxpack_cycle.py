"""両手で箱を運ぶACTの切替に使う、実測姿勢からの完了判定。"""
import json
from pathlib import Path
from typing import NamedTuple

from lerobot_teleoperator_yubi.kinematics import ARMS, ArmKinematics


class Hand(NamedTuple):
    opening: float
    x: float
    y: float
    z: float


class BoxpackHands:
    def __init__(self, config_path, hardware):
        cfg = json.loads(Path(config_path).read_text())
        if set(cfg["arms"]) != set(ARMS):
            raise ValueError("交互推論には左右の手先とグリッパー校正が必要です")
        limits = {j.name: (j.min, j.max) for j in hardware.joints}
        self.kinematics = {
            side: ArmKinematics(hardware.options.robot_model, cfg["base_link"],
                                cfg["arms"][side]["tcp_link"], limits, side=side)
            for side in ARMS
        }
        self.calibration = {
            side: (arm["gripper_closed_rad"], arm["gripper_open_rad"])
            for side, arm in cfg["arms"].items()
        }

    def read(self, state, side):
        closed, opened = self.calibration[side]
        opening = (state[side[0].upper() + "_gripper_joint"] - closed) / (opened - closed)
        x, y, z = self.kinematics[side].fk([state[n] for n in ARMS[side]])[:3, 3]
        return Hand(opening, x, y, z)


class TransferProgress:
    """外側で掴む・中央で受け取る・外側へ置く、を順に確認する。"""
    def __init__(self, source, now):
        self.source = source
        self.destination = "right" if source == "left" else "left"
        self.destination_sign = 1 if self.destination == "left" else -1
        self.started = now
        self.phase = "source"
        self.grip_since = None
        self.release_since = None
        self.drop_m = 0.
        self.release_drop_m = 0.
        self.outward_m = 0.

    @property
    def stage(self):
        return {"source": "送り手の把持待ち", "transfer": "中央での受け取り待ち",
                "place": "移動先での解放待ち"}[self.phase]

    def gripped(self, now, opening, in_zone):
        # 箱を挟んだときは閉じ切らない。開閉途中に範囲を横切るだけの場合を除く。
        if not in_zone or not .12 <= opening <= .39:
            self.grip_since = None
            return False
        if self.grip_since is None:
            self.grip_since = now
            return False
        if now - self.grip_since < .09:
            return False
        self.grip_since = None
        return True

    def update(self, now, source, destination):
        if now - self.started >= 16.:
            return "timeout"
        signed_y = self.destination_sign * destination.y
        if self.phase == "source":
            if self.gripped(now, source.opening, -self.destination_sign * source.y >= .17):
                self.phase = "transfer"
        elif self.phase == "transfer":
            if self.gripped(now, destination.opening, source.opening >= .44 and abs(destination.y) <= .115):
                self.picked_y = signed_y
                self.peak_z = destination.z
                self.min_opening = destination.opening
                self.phase = "place"
        else:
            self.peak_z = max(self.peak_z, destination.z)
            self.min_opening = min(self.min_opening, destination.opening)
            self.drop_m = self.peak_z - destination.z
            self.outward_m = signed_y - self.picked_y
            # 収録では送り先のTCPは外側20 cm以上、解放前後の下げ幅は中央値で約1 cm。
            open_on_destination = (signed_y >= .19 and self.outward_m >= .12 and destination.opening >= .44
                                   and self.min_opening >= .12 and destination.opening - self.min_opening >= .09)
            if not open_on_destination:
                self.release_since = None
            elif self.release_since is None:
                if .24 <= destination.x <= .40 and destination.z <= .075 and self.drop_m >= .0025:
                    self.release_since = now
                    self.release_drop_m = self.drop_m
            elif now - self.release_since >= .30:
                return "placed"
        return "ongoing"
