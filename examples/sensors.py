"""3台の実カメラはSDKとは別に明示したV4L2デバイスから取得する。"""
from pathlib import Path
import yaml
from topoarm_sdk import Device


def rgb_reader(robot, cleanup, config):
    if not isinstance(robot, Device):
        return robot.sim.images
    if config is None:
        raise ValueError("実機収録には --cameras で3台のV4L2デバイスを指定してください")
    import av
    devices = yaml.safe_load(Path(config).read_text())
    if set(devices) != {"front", "left", "right"} or len(set(devices.values())) != 3:
        raise ValueError("front/left/rightには別々のカメラを指定してください")
    frames = {}
    for k, path in devices.items():
        container = av.open(path, format="v4l2")
        cleanup.callback(container.close)
        frames[k] = container.decode(video=0)
    def read():
        return {k: next(it).reformat(width=320, height=240, format="rgb24").to_ndarray() for k,it in frames.items()}
    return read
