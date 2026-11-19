"""制御周期の姿勢を描画側へ渡す。収録中のサンプルは順序を保つ。"""
import time
from copy import copy
from dataclasses import dataclass
from queue import Empty, SimpleQueue

import mujoco


@dataclass
class Frame:
    stamp: float
    qpos: object
    sim_time: float
    state: dict
    action: dict
    images: dict | None = None


class SimFrames:
    def __init__(self, sim, episodes):
        # GLコンテキスト・表示姿勢・モデルはメインスレッド専用。
        self.view = copy(sim)
        self.view.model = copy(sim.model)
        self.view.data = mujoco.MjData(self.view.model)
        self.view.cameras = ("front", "left", "right")
        self.view.fisheyes = dict(sim.fisheyes)
        self.view.camera_calibration = dict(sim.camera_calibration)
        self.episodes = episodes
        self.latest = None
        self.queue = SimpleQueue()
        self.task = None
        self.frames = 0

    @property
    def accepted(self):
        return self.episodes.accepted

    def button(self, key, text):
        if self.task is None:
            if key == "A" and text.strip():
                self.task = text.strip()
                self.frames = 0
                self.queue.put(("A", self.task))
            return False
        if key == "A" and self.frames == 0:
            return False
        self.task = None
        self.queue.put((key, text))
        return True

    def publish(self, frame):
        if self.task is not None:
            self.queue.put(frame)
            self.frames += 1
        self.latest = frame

    def pose(self, frame):
        data = self.view.data
        data.qpos[:] = frame.qpos
        data.time = frame.sim_time
        mujoco.mj_forward(self.view.model, data)

    def record_pending(self, deadline):
        while time.monotonic() < deadline:
            try:
                item = self.queue.get_nowait()
            except Empty:
                return
            if isinstance(item, Frame):
                if item.images is None:
                    self.pose(item)
                    item.images = self.view.images()
                self.episodes.frame(item.images, item.state, item.action)
            else:
                self.episodes.button(*item)

    def close(self):
        try:
            self.record_pending(float("inf"))
        finally:
            self.view.close()
