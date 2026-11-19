"""現行URDFの指定した腕のチェーンを使うFK/IK。ROS・描画・メッシュ読込みは不要。"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from .config import ARMS


def transform(xyz, xyzw):
    xyz, xyzw = np.asarray(xyz, float), np.asarray(xyzw, float)
    if xyz.shape != (3,) or xyzw.shape != (4,) or not np.isfinite(np.r_[xyz, xyzw]).all():
        raise ValueError("姿勢の形式が不正です")
    if np.linalg.norm(xyzw) < 1e-6:
        raise ValueError("Quaternionがゼロです")
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(xyzw).as_matrix()
    out[:3, 3] = xyz
    return out


class IKTargetUnreachable(RuntimeError):
    """許容誤差内の手先姿勢が得られなかった。"""


class ArmKinematics:
    def __init__(self, path, base_link, tcp_link, limits, *, side):
        self.joints = ARMS[side]
        root = ET.parse(path).getroot()
        by_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
        chain, seen, link = [], set(), tcp_link
        while link != base_link:
            if link in seen or link not in by_child:
                raise ValueError("URDFのbaseからTCPまでのチェーンが不正です")
            seen.add(link)
            joint = by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        self.chain, active, lower, upper = [], [], [], []
        for joint in reversed(chain):
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            if rpy.shape != (3,) or not np.isfinite(rpy).all():
                raise ValueError("URDFのoriginが不正です")
            fixed = transform(xyz, Rotation.from_euler("xyz", rpy).as_quat())
            name, kind = joint.get("name"), joint.get("type")
            axis, index = None, None
            if kind != "fixed":
                if kind != "revolute" or joint.find("mimic") is not None:
                    raise ValueError("IKチェーンは独立したrevolute関節だけに対応します")
                active.append(name)
                index = len(active)-1
                axis_element = joint.find("axis")
                axis = np.fromstring(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0", sep=" ")
                if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
                    raise ValueError("URDFのaxisが不正です")
                axis /= np.linalg.norm(axis)
                limit = joint.find("limit")
                lower.append(max(float(limit.get("lower")), limits[name][0]))
                upper.append(min(float(limit.get("upper")), limits[name][1]))
            self.chain.append((fixed, axis, index))
        if active != self.joints:
            raise ValueError("IKチェーンが指定側の7関節と一致しません")
        self.lower, self.upper = np.array(lower), np.array(upper)
        if not np.isfinite(np.r_[self.lower, self.upper]).all() or np.any(self.lower >= self.upper):
            raise ValueError("URDFと個体設定の可動域に共通範囲がありません")

    def _forward(self, q, axes=None, origins=None):
        q = np.asarray(q, float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("腕7関節の有限値が必要です")
        out = np.eye(4)
        for fixed, axis, index in self.chain:
            out = out @ fixed
            if index is not None:
                if axes is not None:
                    axes[index] = out[:3, :3] @ axis
                    origins[index] = out[:3, 3]
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis*q[index]).as_matrix()
                out = out @ rotation
        return out

    def fk(self, q):
        return self._forward(q)

    def shoulder_origin(self):
        """ベース座標での第1関節の中心。"""
        frame = np.eye(4)
        for fixed, _, index in self.chain:
            frame = frame @ fixed
            if index == 0:
                return frame[:3, 3]

    def solve_position_first(self, target, seed, config):
        """関節の残り可動域の中で肩も使い、必要なら手先姿勢を緩める。"""
        start = np.clip(seed, self.lower, self.upper)
        # 反復回数にかかわらず、1周期で別の腕姿勢に飛び移らないようにする。
        lower = np.maximum(self.lower, start - 0.12)
        upper = np.minimum(self.upper, start + 0.12)
        axes, origins = np.empty((7, 3)), np.empty((7, 3))
        position_tol = config.ik_position_tolerance_m
        rotation_tol = config.ik_rotation_tolerance_rad
        rotation_slack = np.deg2rad(10.)
        best = None
        best_score = (np.inf, np.inf, np.inf)
        iterations = min(8, config.ik_max_nfev)
        # まず元の姿勢を狙い、位置が合わなければ向きの重みを下げて再探索する。
        for weight in (0.2, 0.02):
            q = start.copy()
            for iteration in range(iterations + 1):
                actual = self._forward(q, axes, origins)
                dp = target[:3, 3] - actual[:3, 3]
                dr = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
                pos, rot = np.linalg.norm(dp), np.linalg.norm(dr)
                if pos <= position_tol and rot <= rotation_tol:
                    return q
                # 急な回転入力でも、許容帯に戻る方向の中間解は選べるようにする。
                score = (max(0., rot - rotation_slack), max(position_tol, pos), rot)
                if best is None or score < best_score:
                    best, best_score = q.copy(), score
                if iteration == iterations:
                    break
                jac = np.vstack((np.cross(axes, actual[:3, 3] - origins).T,
                                 weight * axes.T, 0.004 * np.eye(7)))
                dq = lsq_linear(jac, np.r_[dp, weight * dr, np.zeros(7)],
                                bounds=(lower - q, upper - q), method="bvls").x
                if np.max(np.abs(dq)) < 1e-6:
                    break
                q = np.clip(q + dq, lower, upper)
        if best is None or np.allclose(best, start, rtol=0., atol=1e-7):
            raise IKTargetUnreachable("手先の位置・姿勢を改善できません")
        return best

    def solve(self, target, seed, config):
        # 世界座標の関節軸から幾何ヤコビアンを作り、前周期の解から減衰付きIKを解く。
        q = np.clip(seed, self.lower, self.upper)
        axes = np.empty((7, 3))
        origins = np.empty((7, 3))
        damping = np.eye(6) * 1e-4
        for iteration in range(min(8, config.ik_max_nfev)):
            actual = self._forward(q, axes, origins)
            dp = target[:3, 3] - actual[:3, 3]
            dr = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            if (np.linalg.norm(dp) <= config.ik_position_tolerance_m and
                    np.linalg.norm(dr) <= config.ik_rotation_tolerance_rad):
                return q
            linear = np.cross(axes, actual[:3, 3] - origins)
            jac = np.vstack((linear.T, 0.2 * axes.T))
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping, np.r_[dp, 0.2 * dr])
            q = np.clip(q + dq / max(1., np.max(np.abs(dq)) / 0.12), self.lower, self.upper)
        raise IKTargetUnreachable("IKが収束しません。目標が可動範囲外の可能性があります")
