"""Quest＋YUBI入力の設定。実機確認前は接続による制御を許可しない。"""
from dataclasses import dataclass, field
import math
from lerobot.teleoperators import TeleoperatorConfig

from .joints import SIDES, ARMS, GRIPPERS, NECK, JOINTS


@dataclass
class YubiArmConfig:
    encoder_port: str = ""
    encoder_id: str = ""
    encoder_closed_rad: float = 0.0
    encoder_open_rad: float = 1.0
    gripper_closed_rad: float = 0.0
    gripper_open_rad: float = math.radians(70)
    controller_to_tcp_xyz: list[float] = field(default_factory=lambda: [0., 0., 0.])
    controller_to_tcp_xyzw: list[float] = field(default_factory=lambda: [0., 0., 0., 1.])
    tcp_link: str = ""


def default_arms():
    return {"right": YubiArmConfig(encoder_id="R003", tcp_link="R_tcp"),
            "left": YubiArmConfig(encoder_id="L003", tcp_link="L_tcp", gripper_open_rad=-math.radians(70))}


@TeleoperatorConfig.register_subclass("yubi")
@dataclass
class YubiTeleoperatorConfig(TeleoperatorConfig):
    quest_host: str = ""
    quest_port: int = 65432
    arms: dict[str, YubiArmConfig] = field(default_factory=default_arms)
    hardware_verified: bool = False
    # Unity→右手Z-up変換後の基準回転。順序はxyzw。
    quest_to_robot_xyzw: list[float] = field(default_factory=lambda: [0., 0., 0., 1.])
    position_scale: float = 1.0
    neck_mode: str = "fixed"
    neck_pan_deg: float = 0.0
    neck_tilt_deg: float = -45.0
    neck_pan_sign: float = 1.0
    neck_tilt_sign: float = 1.0
    urdf_path: str = "models/300mm/robot.urdf"
    base_link: str = "base_link"
    input_timeout_s: float = 0.25
    connect_timeout_s: float = 3.0
    ik_position_tolerance_m: float = 0.001
    ik_rotation_tolerance_rad: float = 0.01
    ik_max_nfev: int = 80

    def make_action_processor(self, robot_config):
        from .processor import make_yubi_processor
        return make_yubi_processor(self, robot_config)

    def validate(self):
        if self.neck_mode not in ("fixed", "follow"):
            raise ValueError("neck_modeはfixed/followを指定してください")
        if set(self.arms) != set(SIDES):
            raise ValueError("armsにはrightとleftの両方が必要です")
        ports = [arm.encoder_port for arm in self.arms.values() if arm.encoder_port]
        if len(ports) != len(set(ports)):
            raise ValueError("左右のエンコーダには別々のポートを指定してください")
        for side, arm in self.arms.items():
            if arm.tcp_link != {"right": "R_tcp", "left": "L_tcp"}[side]:
                raise ValueError(f"{side}のTCPリンクが左右と一致しません")
            for key in ("encoder_closed_rad", "encoder_open_rad", "gripper_closed_rad", "gripper_open_rad"):
                if not math.isfinite(getattr(arm, key)):
                    raise ValueError(f"{side}.{key}は有限値にしてください")
            if abs(arm.encoder_open_rad - arm.encoder_closed_rad) < 0.01:
                raise ValueError(f"{side}のYUBI閉/開校正値が近すぎます")
            if arm.gripper_closed_rad == arm.gripper_open_rad:
                raise ValueError(f"{side}のグリッパー閉/開角度が同一です")
            self.validate_transform(arm.controller_to_tcp_xyz, 3, f"{side}.controller_to_tcp_xyz")
            self.validate_transform(arm.controller_to_tcp_xyzw, 4, f"{side}.controller_to_tcp_xyzw")
        for key in ("neck_pan_deg", "neck_tilt_deg"):
            if not math.isfinite(getattr(self, key)):
                raise ValueError(f"{key}は有限値にしてください")
        for key in ("position_scale", "input_timeout_s", "connect_timeout_s",
                    "ik_position_tolerance_m", "ik_rotation_tolerance_rad"):
            if not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f"{key}は正の有限値にしてください")
        if self.neck_pan_sign not in (-1, 1) or self.neck_tilt_sign not in (-1, 1):
            raise ValueError("首の方向は1または-1にしてください")
        if not 1 <= self.quest_port <= 65535 or self.ik_max_nfev < 1:
            raise ValueError("ポートまたはIK反復数が不正です")
        self.validate_transform(self.quest_to_robot_xyzw, 4, "quest_to_robot_xyzw")

    @staticmethod
    def validate_transform(values, size, key):
        if len(values) != size or not all(math.isfinite(v) for v in values):
            raise ValueError(f"{key}が不正です")
        if size == 4 and sum(v*v for v in values) < 1e-12:
            raise ValueError(f"{key}のQuaternionがゼロです")
