"""校正済みのリーダー角からフォロワー目標を作る。角度はrad。"""
import math


NECK_JOINTS = ("neck_pan_joint", "neck_tilt_joint")
DEFAULT_NECK_PAN_RAD = 0.0
DEFAULT_NECK_TILT_RAD = -0.9


def scale_negative_angle(angle: float, scale: float) -> float:
    """0以上は等倍、0未満だけ倍率を適用する。角度単位は入出力で同じ。"""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("下側の角度倍率は有限の正数が必要です")
    if not math.isfinite(angle):
        raise ValueError("変換する角度は有限値が必要です")
    result = angle * scale if angle < 0 else angle
    if not math.isfinite(result):
        raise ValueError("変換後の角度が非有限です")
    return result


class LeaderFollowerTransform:
    def __init__(self, joint_map, neck_mode="fixed", *, neck_pan_rad=DEFAULT_NECK_PAN_RAD,
                 neck_tilt_rad=DEFAULT_NECK_TILT_RAD):
        targets = set(joint_map.values())
        if not joint_map or len(targets) != len(joint_map):
            raise ValueError("joint_mapは空・出力先重複にできません")
        if neck_mode not in (None, "fixed", "follow"):
            raise ValueError("neck_modeはfixed/follow、首なしの場合のみnullを指定してください")
        neck = set(NECK_JOINTS)
        if neck_mode == "follow" and not neck <= targets:
            raise ValueError("followにはjoint_mapで首pan/tilt両方の対応が必要です")
        if neck_mode is None and neck & targets:
            raise ValueError("首の対応を指定する場合はneck_modeをfixed/followにしてください")

        self._neck_mode = neck_mode
        self._joint_map = dict(joint_map)
        self._fixed = {}
        if neck_mode == "fixed":
            fixed_angles = (neck_pan_rad, neck_tilt_rad)
            if not all(math.isfinite(angle) for angle in fixed_angles):
                raise ValueError("固定首角度には有限値を指定してください")
            self._joint_map = {source: target for source, target in joint_map.items() if target not in neck}
            self._fixed = dict(zip(NECK_JOINTS, fixed_angles))
        if not self._joint_map:
            raise ValueError("leaderから読み取る腕の関節対応が必要です")
        self.source_joints = tuple(self._joint_map)
        self.target_joints = (*self._joint_map.values(), *self._fixed)

    def targets(self, positions, *, neck_tilt_negative_scale=1.0):
        goals = {target: float(positions[source]) for source, target in self._joint_map.items()}
        if not all(math.isfinite(value) for value in goals.values()):
            raise ValueError("リーダー角度は有限値が必要です")
        goals.update(self._fixed)
        if self._neck_mode == "follow":
            tilt = NECK_JOINTS[1]
            goals[tilt] = scale_negative_angle(goals[tilt], neck_tilt_negative_scale)
        return goals
