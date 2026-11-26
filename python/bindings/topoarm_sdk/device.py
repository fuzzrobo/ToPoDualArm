"""実機に直接接続する。保護と送信はC++の同じセッションで行う。"""
import time
from pathlib import Path
import yaml
from ._native import Driver, load_config
from ._session import PositionSession


class Device(PositionSession):
    def __init__(self, path, *, read_profile=None):
        path = Path(path).resolve()
        profile = yaml.safe_load(path.read_text())
        if profile["backend"] != "direct":
            raise ValueError("Deviceには実機直結用の個体プロファイルを指定してください")
        self.hardware = (path.parent / profile["hardware"]).resolve()
        self.cfg = load_config(str(self.hardware))
        if read_profile is not None:
            if read_profile not in ("teleop", "bam"):
                raise ValueError("read_profileにはteleopまたはbamを指定してください")
            self.cfg.options.read_profile = read_profile
        self.last_states = ()
        super().__init__([j.name for j in self.cfg.joints], profile["role"], profile.get("state_timeout_s", .25))
        self.device = None

    def _connect(self):
        c = self.cfg
        if not c.hardware_verified:
            raise ValueError("未校正の実機設定には接続できません")
        o = c.options
        if not self.read_only and not o.floor_enabled:
            raise ValueError("実機フォロワーには床保護の設定が必要です")
        o.verified, o.torque, o.configure_devices = True, not self.read_only, False
        self.device = Driver(c.port, c.baud_rate, c.latency_timer, c.joints, o)
        if self.read_only and o.apply_startup_configuration:
            self.device.connect()
        else:
            self.device.open_passive()

    def _read(self):
        states = self.device.read()
        stamp = time.monotonic_ns()
        if any(j.valid != 1 or (self.read_only and j.torque_enabled != 0) for j in states):
            raise RuntimeError("関節状態が無効、またはリーダーのTorqueがONです")
        self.last_states = states
        return {j.name: j.position for j in states}, stamp

    def _enable(self):
        self.device.enable()

    def _disable(self):
        self.device.disable()

    def _command(self, q):
        return dict(zip(q, self.device.command(list(q.values())), strict=True))

    def _close(self):
        if self.device is not None:
            self.device.close()
            self.device = None
