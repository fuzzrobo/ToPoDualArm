"""3台の実カメラはSDKとは別に明示したV4L2デバイスから取得する。"""


def rgb_reader(cleanup, devices):
    import av
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
