"""採用した試行だけを同じLeRobotデータセットに追加する。"""
import numpy as np
from lerobot.datasets import LeRobotDataset


class Episodes:
    def __init__(self, root, names, fps=20, *, bam=False, hardware_timing=False):
        self.root = root
        self.names = list(names)
        features = {key: dict(dtype="float32", shape=(len(names),), names=self.names)
                    for key in ("action", "observation.state")}
        self.hardware_timing = hardware_timing
        if hardware_timing:
            features["observation.host_time_s"] = dict(dtype="float64", shape=(5,),
                names=["state", "command", "left", "front", "right"])
        self.bam_fields = ("velocity", "current", "pwm", "voltage", "temperature", "torque_enabled",
                           "hardware_error", "realtime_tick", "goal", "position_trajectory",
                           "velocity_trajectory") if bam else ()
        for field in self.bam_fields:
            features[f"observation.bam.{field}"] = features["observation.state"].copy()
        features.update({f"observation.images.{side}": dict(dtype="image", shape=(240, 320, 3),
                         names=["height", "width", "channels"]) for side in ("front", "left", "right")})
        self.data = LeRobotDataset.create("local/topoarm", fps, features, root=root,
                                          robot_type="topoarm", use_videos=False, image_writer_threads=3)
        self.task = None
        self.frames = 0
        self.accepted = 0

    def button(self, key, task):
        if self.task is None:
            if key == "A" and task.strip():
                if self.data is None:
                    self.data = LeRobotDataset.resume("local/topoarm", root=self.root, image_writer_threads=3)
                self.task = task.strip()
                self.frames = 0
            return False
        if key == "A":
            if not self.frames:
                return False
            self.data.save_episode()
            # 採用時点でParquetの末尾とエピソード索引を確定する。
            self.data.finalize()
            self.data = None
            self.accepted += 1
        else:
            self.data.clear_episode_buffer()
        self.task = None
        return True

    def frame(self, images, state, applied, servo_states=(), host_times=()):
        if self.task is None:
            return
        row = {key: np.array([values[n] for n in self.names], dtype=np.float32)
               for key, values in (("action", applied), ("observation.state", state))}
        row.update({f"observation.images.{k}": v for k, v in images.items()})
        if self.hardware_timing:
            row["observation.host_time_s"] = np.asarray(host_times, dtype=np.float64)
        if self.bam_fields:
            joints = {joint.name: joint for joint in servo_states}
            for field in self.bam_fields:
                row[f"observation.bam.{field}"] = np.array(
                    [getattr(joints[name], field) for name in self.names], dtype=np.float32)
        self.data.add_frame({**row, "task": self.task})
        self.frames += 1

    def close(self):
        if self.data is not None:
            if self.task is not None:
                self.data.clear_episode_buffer()
            self.data.finalize()
