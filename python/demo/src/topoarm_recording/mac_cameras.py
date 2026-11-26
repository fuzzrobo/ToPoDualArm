"""左右カメラを継続取得する。時刻は露光時刻でなくホスト受信完了時刻。"""
from collections import deque
import ctypes
from fractions import Fraction
import math
from pathlib import Path
import threading
import time
import sys

import cv2


RECONNECT_INTERVAL = .5
WARMUP_FRAMES = 10
PREVIEW_MAX_AGE_S = .15
DARK_TRANSIENT_S = .25


def device_key(value):
    return str(Path(f'/dev/video{value}' if isinstance(value, int) else value).resolve())


def _mac_frame_rate(unique_id, fps):
    """OpenCVが対応範囲の最大FPSを選ぶため、AVFoundationへ正確な周期を設定する。"""
    from AVFoundation import AVCaptureDevice
    from CoreMedia import CMTimeMake, CMTimeCompare, CMVideoFormatDescriptionGetDimensions
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('カメラFPSは正の有限値で指定してください')
    device = AVCaptureDevice.deviceWithUniqueID_(unique_id)
    if device is None:
        raise RuntimeError(f'カメラが接続されていません: {unique_id}')
    current = device.activeFormat()
    size = CMVideoFormatDescriptionGetDimensions(current.formatDescription())
    selected = match = None
    # 同じ640×480でも120fps専用の420vと30fps専用のyuvsが別形式として列挙される。
    formats = [current, *[f for f in device.formats() if f != current]]
    for fmt in formats:
        dims = CMVideoFormatDescriptionGetDimensions(fmt.formatDescription())
        if (dims.width, dims.height) != (size.width, size.height):
            continue
        match = next((r for r in fmt.videoSupportedFrameRateRanges()
                      if r.minFrameRate()-1e-3 <= fps <= r.maxFrameRate()+1e-3), None)
        if match is not None:
            selected = fmt
            break
    if selected is None:
        raise RuntimeError(f'カメラが{fps}fpsに対応していません')
    # RealSenseの公称30fpsは30.00003fps。離散値はデバイスが返した周期をそのまま使う。
    if fps < match.minFrameRate():
        interval = match.maxFrameDuration()
    elif fps > match.maxFrameRate():
        interval = match.minFrameDuration()
    else:
        rate = Fraction(fps).limit_denominator(100000)
        interval = CMTimeMake(rate.denominator, rate.numerator)
    locked, error = device.lockForConfiguration_(None)
    if not locked:
        raise RuntimeError('カメラFPS設定のロックに失敗しました')
    try:
        if device.activeFormat() != selected:
            device.setActiveFormat_(selected)
        # 設定途中でも最小周期≦最大周期を保つ。
        setters = [device.setActiveVideoMinFrameDuration_, device.setActiveVideoMaxFrameDuration_]
        if CMTimeCompare(interval, device.activeVideoMaxFrameDuration()) >= 0:
            setters.reverse()
        for setter in setters:
            setter(interval)
        for value in (device.activeVideoMinFrameDuration(), device.activeVideoMaxFrameDuration()):
            if value.value <= 0 or value.timescale <= 0 or not math.isclose(value.timescale/value.value, fps, rel_tol=1e-4):
                raise RuntimeError('カメラFPSの読戻しが一致しません')
    finally:
        device.unlockForConfiguration()


def mac_camera_index(unique_id, *, fps=None):
    """OpenCVと同じAVFoundationの一覧・順序からUSBカメラを引く。"""
    ctypes.CDLL('/System/Library/Frameworks/AVFoundation.framework/AVFoundation')
    objc = ctypes.CDLL('/usr/lib/libobjc.A.dylib')
    ptr, string = ctypes.c_void_p, ctypes.c_char_p
    objc.objc_getClass.argtypes, objc.objc_getClass.restype = [string], ptr
    objc.sel_registerName.argtypes, objc.sel_registerName.restype = [string], ptr

    def send(receiver, selector, result=ptr, argument_type=None, argument=None):
        types = () if argument_type is None else (argument_type,)
        args = () if argument_type is None else (argument,)
        invoke = ctypes.CFUNCTYPE(result, ptr, ptr, *types)(('objc_msgSend', objc))
        return invoke(receiver, objc.sel_registerName(selector), *args)

    pool = send(send(objc.objc_getClass(b'NSAutoreleasePool'), b'alloc'), b'init')
    try:
        ids = []
        for media in (b'vide', b'muxx'):
            kind = send(objc.objc_getClass(b'NSString'), b'stringWithUTF8String:', ptr, string, media)
            devices = send(objc.objc_getClass(b'AVCaptureDevice'), b'devicesWithMediaType:', ptr, ptr, kind)
            for index in range(send(devices, b'count', ctypes.c_size_t)):
                device = send(devices, b'objectAtIndex:', ptr, ctypes.c_size_t, index)
                identifier = send(send(device, b'uniqueID'), b'UTF8String', string).decode()
                ids.append(identifier)
        if ids.count(unique_id) != 1:
            raise RuntimeError(f'指定されたカメラを一意に識別できません: {unique_id}')
        if fps is not None:
            _mac_frame_rate(unique_id, fps)
        return sorted(ids).index(unique_id)
    finally:
        send(pool, b'drain', None)


class CameraStreams:
    def __init__(self, devices, *, width=640, height=480, fps=30, reject_dark_frames=False):
        if not devices or len({device_key(value) for value in devices.values()}) != len(devices):
            raise ValueError('カメラごとに異なるデバイスを指定してください')
        self.threads, self.frames, self.errors = [], {}, {}
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.open_lock = threading.Lock()
        self.preview_frames = {side: deque(maxlen=10) for side in devices}
        self.size = (height, width, 3)
        self.fps = fps
        self.reject_dark_frames = reject_dark_frames
        self.devices = dict(devices)
        self.reopen = {name: threading.Event() for name in devices}
        self.reopen_requested = dict.fromkeys(devices, 0.)
        self.diagnostic = {name: dict(opens=0, raw_frames=0, published=0, dark_drops=0,
                                     last_raw_at=None, last_publish_at=None, state='opening')
                           for name in devices}
        try:
            for side, device in devices.items():
                thread = threading.Thread(target=self._capture, args=(side, device, None), daemon=True)
                thread.start()
                self.threads.append(thread)
        except BaseException:
            self.close()
            raise

    def _open(self, side, device):
        with self.lock:
            self.diagnostic[side]['opens'] += 1
            self.diagnostic[side]['state'] = 'opening'
            self.diagnostic[side]['last_open_at'] = time.monotonic()
        backend = cv2.CAP_AVFOUNDATION if sys.platform == 'darwin' else cv2.CAP_ANY
        by_id = backend == cv2.CAP_AVFOUNDATION and isinstance(device, str)
        index = mac_camera_index(device) if by_id else device
        cap = cv2.VideoCapture(index, backend)
        try:
            if not cap.isOpened():
                raise RuntimeError(f'{side}カメラを開けません: {device}')
            height, width, _ = self.size
            applied = {}
            for key, prop, value in (
                    ('mjpg', cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                    ('width', cv2.CAP_PROP_FRAME_WIDTH, width),
                    ('height', cv2.CAP_PROP_FRAME_HEIGHT, height),
                    ('fps', cv2.CAP_PROP_FPS, self.fps),
                    ('buffer', cv2.CAP_PROP_BUFFERSIZE, 1)):
                applied[key] = bool(cap.set(prop, value))
            before_fps = cap.get(cv2.CAP_PROP_FPS) if hasattr(cap, 'get') else None
            if by_id and mac_camera_index(device, fps=self.fps) != index:
                raise RuntimeError(f'{side}カメラを開く途中で機器一覧が変わりました')
            with self.lock:
                self.diagnostic[side]['settings_accepted'] = applied
                self.diagnostic[side]['fps_after_backend_set'] = before_fps
                self.diagnostic[side]['native_fps_fixed'] = by_id
                if hasattr(cap, 'get'):
                    self.diagnostic[side]['reported_fps'] = cap.get(cv2.CAP_PROP_FPS)
                    self.diagnostic[side]['reported_size'] = [cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)]
            return cap
        except BaseException:
            cap.release()
            raise

    def _capture(self, side, device, cap):
        sequence = 0
        period = 1 / self.fps if sys.platform == 'darwin' and isinstance(device, str) else 0.
        while not self.stop.is_set():
            try:
                if cap is None:
                    with self.open_lock:
                        if self.stop.is_set():
                            return
                        cap = self._open(side, device)
                with self.lock:
                    self.diagnostic[side]['state'] = 'warmup'
                warmup = WARMUP_FRAMES
                next_frame = 0.
                reference = dark_since = None
                while not self.stop.is_set():
                    ok, bgr = cap.read()
                    timestamp = time.monotonic()
                    if self.reopen[side].is_set():
                        self.reopen[side].clear()
                        raise RuntimeError(f'{side}カメラの更新停止により再接続')
                    with self.lock:
                        if ok and bgr is not None:
                            self.diagnostic[side]['last_raw_at'] = timestamp
                            self.diagnostic[side]['raw_frames'] += 1
                            self.diagnostic[side]['shape'] = list(bgr.shape)
                    if not ok or bgr is None or bgr.shape != self.size:
                        raise RuntimeError(f'{side}カメラの欠測または解像度不一致: '
                                           f'ok={ok}, shape={None if bgr is None else bgr.shape}')
                    if warmup:
                        warmup -= 1
                        continue
                    # AVFoundationが要求より速く返す場合も、推論に渡す画像は指定周期に揃える。
                    if timestamp < next_frame - .002:
                        continue
                    pixels = bgr[::16, ::16]
                    brightness = float(pixels.mean())
                    with self.lock:
                        self.diagnostic[side]['brightness'] = brightness
                    if self.reject_dark_frames:
                        # ほぼ黒一色のコマは観測に使わない。急な暗転は短時間だけ除外する。
                        if brightness < 10 and pixels.max() < 20:
                            with self.lock:
                                self.diagnostic[side]['dark_drops'] += 1
                                self.diagnostic[side]['state'] = 'dark_filtered'
                            continue
                        if reference is not None and brightness < .65 * reference and reference - brightness >= 12:
                            if dark_since is None:
                                dark_since = timestamp
                            if timestamp - dark_since < DARK_TRANSIENT_S:
                                with self.lock:
                                    self.diagnostic[side]['dark_drops'] += 1
                                    self.diagnostic[side]['state'] = 'dark_filtered'
                                continue
                            reference = brightness
                        dark_since = None
                        reference = brightness if reference is None else .9 * reference + .1 * brightness
                    next_frame = (timestamp if timestamp > next_frame + period else next_frame) + period
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    sequence += 1
                    with self.lock:
                        self.frames[side] = (rgb, timestamp)
                        self.preview_frames[side].append((rgb, timestamp, brightness))
                        self.errors.pop(side, None)
                        self.diagnostic[side].update(state='streaming', last_publish_at=timestamp, published=sequence)
                return
            except Exception as error:
                with self.lock:
                    self.errors[side] = str(error)
                    self.frames.pop(side, None)
                    self.preview_frames[side].clear()
                    self.diagnostic[side].update(state='retry_wait', last_error=str(error))
            finally:
                if cap is not None:
                    cap.release()
                    cap = None
            if self.stop.wait(RECONNECT_INTERVAL):
                return

    def frame_rates(self):
        cutoff = time.monotonic() - PREVIEW_MAX_AGE_S
        with self.lock:
            return {side: (len(frames) - 1) / (frames[-1][1] - frames[0][1])
                    for side, frames in self.preview_frames.items()
                    if len(frames) > 1 and frames[-1][1] >= cutoff and frames[-1][1] > frames[0][1]}

    def diagnostics(self):
        now = time.monotonic()
        with self.lock:
            result = {n: dict(v) for n, v in self.diagnostic.items()}
        for values in result.values():
            for key in ('last_raw_at', 'last_publish_at', 'last_open_at'):
                stamp = values.pop(key, None)
                values[key.replace('_at', '_age_s')] = None if stamp is None else round(now-stamp, 3)
        return result

    def reconnect_stale(self, max_age=1.):
        """保持用の鮮度制限とは別に、1秒止まったカメラだけ取得スレッドで開き直す。"""
        now = time.monotonic()
        with self.lock:
            for name, values in self.diagnostic.items():
                published = values['last_publish_at']
                raw = values['last_raw_at']
                # 暗転除外中でも生画像は届いている場合、USB再接続を繰り返さない。
                dark = values['state'] == 'dark_filtered' and raw is not None and now-raw <= max_age
                warming = (values['state'] in ('opening', 'warmup') and
                           now-values.get('last_open_at', now) < 2.)
                if (name not in self.errors and not dark and not warming and (published is None or now-published > max_age)
                        and now-self.reopen_requested[name] >= 1.5):
                    self.reopen_requested[name] = now
                    self.reopen[name].set()

    def latest_per_camera(self, *, max_age=.15, max_skew=.06):
        """受信済みの最新画像と元の受信時刻、カメラごとの同期判定を返す。"""
        with self.lock:
            frames = self.frames.copy()
            errors = set(self.errors)
        stamps = {name: stamp for name, (_, stamp) in frames.items()}
        now = time.monotonic()
        newest = max(stamps.values(), default=now)
        valid = {name: name in stamps and name not in errors and
                 now - stamps[name] <= max_age and newest - stamps[name] <= max_skew
                 for name in self.devices}
        return {name: image for name, (image, _) in frames.items()}, stamps, valid

    def preview(self):
        cutoff = time.monotonic() - PREVIEW_MAX_AGE_S
        with self.lock:
            # 直近の明るいフレームを使い、期限切れの映像は表示しない。
            return {side: max(recent, key=lambda frame: frame[2])[0]
                    for side, frames in self.preview_frames.items()
                    if (recent := [frame for frame in frames if frame[1] >= cutoff])}

    def close(self):
        self.stop.set()
        # read中のVideoCaptureを別スレッドからreleaseしない。
        for thread in self.threads:
            thread.join(timeout=.5)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
