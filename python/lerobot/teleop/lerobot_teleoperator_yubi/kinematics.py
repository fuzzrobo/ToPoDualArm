"""現行URDFの指定した腕のチェーンを使うFK/IK。ROS・描画・メッシュ読込みは不要。"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
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

    def fk(self, q):
        q = np.asarray(q, float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("腕7関節の有限値が必要です")
        out = np.eye(4)
        for fixed, axis, index in self.chain:
            out = out @ fixed
            if index is not None:
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis*q[index]).as_matrix()
                out = out @ rotation
        return out

    def solve(self, target, seed, config):
        seed = np.asarray(seed, float)
        if np.any(seed < self.lower) or np.any(seed > self.upper):
            raise ValueError("IK初期姿勢がモデル可動域外です")

        def error(q):
            actual = self.fk(q)
            rotation = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            return np.r_[target[:3, 3]-actual[:3, 3], 0.2*rotation]

        result = least_squares(error, seed, bounds=(self.lower, self.upper),
                               max_nfev=config.ik_max_nfev, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        residual = error(result.x)
        if (not np.isfinite(result.x).all()
                or np.linalg.norm(residual[:3]) > config.ik_position_tolerance_m
                or np.linalg.norm(residual[3:])/0.2 > config.ik_rotation_tolerance_rad):
            raise RuntimeError("IKが収束しません。目標が可動範囲外の可能性があります")
        return result.x
