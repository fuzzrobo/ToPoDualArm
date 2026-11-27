"""位置セッションの寿命と実測の鮮度を管理する。関節・床の補正はC++側。"""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class State:
    position: dict[str, float]
    monotonic_ns: int
    valid: bool


class PositionSession:
    def __init__(self, names, role, timeout):
        if role not in ("leader", "follower"):
            raise ValueError("roleはleaderまたはfollowerです")
        self.joints, self.read_only, self.timeout = tuple(names), role == "leader", float(timeout)
        if not math.isfinite(self.timeout) or not 0 < self.timeout <= 2:
            raise ValueError("状態の期限は0秒より大きく2秒以下です")
        self._open = self._enabled = False
        self._stamp = None

    def __enter__(self):
        if self._open:
            raise RuntimeError("すでに接続済みです")
        try:
            self._connect()
        except BaseException:
            self._close()
            raise
        self._open = True
        return self

    def _connected(self):
        if not self._open:
            raise RuntimeError("先にwithで接続してください")

    def read(self):
        self._connected()
        self._stamp = None
        q, stamp = self._read()
        if set(q) != set(self.joints) or not all(math.isfinite(v) for v in q.values()):
            raise RuntimeError("有効な全軸の状態がありません")
        if not 0 <= time.monotonic_ns() - stamp <= self.timeout * 1e9:
            raise TimeoutError("実測状態が古くなっています")
        self._stamp = stamp
        return State(q, stamp, True)

    def enable(self):
        self._connected()
        if self.read_only or self._enabled:
            raise RuntimeError("読み取り専用または既に有効です")
        self._enable()
        self._enabled = True
        self._stamp = None

    def disable(self):
        if self._enabled:
            self._enabled = False
            self._stamp = None
            self._disable()

    def command(self, positions):
        self._connected()
        if not self._enabled:
            raise RuntimeError("指令にはenable()が必要です")
        stamp, self._stamp = self._stamp, None
        if stamp is None or not 0 <= time.monotonic_ns()-stamp <= self.timeout*1e9:
            raise TimeoutError("各指令の直前にread()が必要です")
        if set(positions) != set(self.joints):
            raise ValueError("指令は設定された全関節を指定してください")
        q = {k: float(positions[k]) for k in self.joints}
        if not all(math.isfinite(v) for v in q.values()):
            raise ValueError("指令が非有限値です")
        return self._command(q)

    def __exit__(self, *exc):
        try:
            self.disable()
        finally:
            self._open = False
            self._stamp = None
            self._close()
