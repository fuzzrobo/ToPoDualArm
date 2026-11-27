"""YUBIの開口エンコーダを読み取る。角度の単位はrad。"""
import math
from .transport import LineInput


def opening(line, config, *, strict=True):
    device, text = line.strip().split(",")
    if device != config.encoder_id:
        raise ValueError("YUBIエンコーダIDが設定と異なります")
    angle = float(text)
    fraction = (angle-config.encoder_closed_rad)/(config.encoder_open_rad-config.encoder_closed_rad)
    if not math.isfinite(fraction) or (strict and not 0 <= fraction <= 1):
        raise ValueError("YUBIの開口が校正範囲外です")
    return angle, fraction


class GripperInput(LineInput):
    def __init__(self, config, *, input_timeout_s=0.25, connect_timeout_s=3.0, strict=True):
        def parse(line, received):
            angle, fraction = opening(line, config, strict=strict)
            return {"received": received, "encoder_rad": angle, "opening": fraction,
                    "within_calibration": 0 <= fraction <= 1}
        super().__init__(parse, input_timeout_s, max_line=128)
        self.connect_timeout_s = connect_timeout_s
        self.config = config

    def connect(self):
        import serial
        if not self.config.encoder_port:
            raise ValueError("encoder_portを設定してください")
        port = serial.Serial(self.config.encoder_port, 115200, timeout=0.05, exclusive=True)
        self.start(lambda: port.read(max(1, min(port.in_waiting, 4096))) or None,
                   port.close, self.connect_timeout_s)
