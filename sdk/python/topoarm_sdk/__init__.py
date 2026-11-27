"""ToPoDualArm実機1台のPythonバインディング。"""
from ._native import OfflineLimiter, load_config
from .device import Device, VsidoDevice

__all__ = ["Device", "OfflineLimiter", "VsidoDevice", "decode_vsido_status", "load_config"]


def decode_vsido_status(payload):
    """V-Sidoの状態をIDごとの位置・目標・速度・電流に変換する。"""
    from ._native import decode_vsido_status as decode

    return decode(payload)
