"""現行URDFの指定した腕のチェーンを使うFK/IK。ROS・描画・メッシュ読込みは不要。"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from .config import ARMS

POSITION_FIRST_REFERENCE_DT = 0.05
POSITION_FIRST_JOINT_SPEED = 1.2
POSITION_FIRST_JOINT_ACCELERATION = 8.0


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

    def solve_position_first(self, target, seed, config, *, dt=POSITION_FIRST_REFERENCE_DT,
                             previous_velocity=None):
        """位置を重くした同一コストで、前回の受理角から次の1歩だけ追従する。"""
        start = np.clip(seed, self.lower, self.upper)
        axes, origins = np.empty((7, 3)), np.empty((7, 3))
        current = self._forward(start, axes, origins)
        # 通常は向き1 radを位置9 cm相当とする。大きな姿勢誤差は滑らかに緩め、
        # 手首の向きだけが難しい目標で、到達可能な位置まで捨てない（pseudo-Huber）。
        weight = .09
        slack = .15

        def error(frame):
            dp = target[:3, 3] - frame[:3, 3]
            dr = Rotation.from_matrix(target[:3, :3] @ frame[:3, :3].T).as_rotvec()
            return dp, dr

        dp, dr = error(current)
        if np.linalg.norm(dp) <= config.ik_position_tolerance_m and np.linalg.norm(dr) <= config.ik_rotation_tolerance_rad:
            return start
        def cost_norm(p, r):
            r2 = r @ r
            return np.sqrt(p @ p + weight**2 * 2*r2 / (1 + np.sqrt(1 + r2 / slack**2)))

        angular = weight / (1 + (dr @ dr) / slack**2)**.25
        # 20 Hzでの25 mm / 0.10 radを速度に換算し、実周期に合わせる。
        # 停滞判定はこの局所目標ではなく、元のQuest目標に対して行う。
        residual = np.r_[dp * min(1., .5 * dt / max(1e-12, np.linalg.norm(dp))),
                         angular * dr * min(1., 2. * dt / max(1e-12, np.linalg.norm(dr)))]
        cost = cost_norm(dp, dr)
        # 特異姿勢でも肘などが飛ばないよう減衰を加える。
        damping = .06
        jac = np.vstack((np.cross(axes, current[:3, 3] - origins).T, angular * axes.T, damping * np.eye(7)))
        # 停止方向は制限せず、発進・反転時の最初の1歩だけ小さくする。
        # 前周期の提案値ではなく、床・関節制限を通った実際の指令差を基準にする。
        previous = np.zeros(7) if previous_velocity is None else previous_velocity
        speed_increment = POSITION_FIRST_JOINT_ACCELERATION * dt
        down = dt * np.minimum(POSITION_FIRST_JOINT_SPEED, speed_increment + np.maximum(0., -previous))
        up = dt * np.minimum(POSITION_FIRST_JOINT_SPEED, speed_increment + np.maximum(0., previous))
        dq = lsq_linear(jac, np.r_[residual, np.zeros(7)],
                        bounds=(np.maximum(self.lower - start, -down), np.minimum(self.upper - start, up)),
                        method="bvls").x
        # 保持中も同じ改善幅で判定し、解ける目標に戻ればその周期で追従する。
        min_gain = min(.00003, .02 * cost**2) * (dt / POSITION_FIRST_REFERENCE_DT)
        for _ in range(min(6, config.ik_max_nfev)):
            q = np.clip(start + dq, self.lower, self.upper)
            p, r = error(self.fk(q))
            if cost - cost_norm(p, r) > min_gain:
                return q
            dq *= .5
        raise IKTargetUnreachable("現在の可動域で手先目標に有意な改善がありません")

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
