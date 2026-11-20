"""Quest＋MuJoCo用の、実測関節角を基準とする差分IK。"""
import mink
import numpy as np
from scipy.spatial.transform import Rotation

from .kinematics import IKTargetUnreachable


class MinkArm:
    def __init__(self, model, kin, base, tcp, dt):
        self.kin, self.tcp, self.dt = kin, tcp, dt
        self.cfg = mink.Configuration(model)
        self.q = self.cfg.q.copy()
        self.adr = [int(model.joint(n).qposadr[0]) for n in kin.joints]
        self.dof = [int(model.joint(n).dofadr[0]) for n in kin.joints]
        mask = np.eye(model.nv)[self.dof]
        self.G = np.vstack((mask, -mask))
        self.task = mink.FrameTask(tcp, "body", position_cost=1., orientation_cost=.09)
        self.base = self.cfg.get_transform_frame_to_world(base, "body").as_matrix()
        tcp_zero = self.cfg.get_transform_frame_to_world(tcp, "body").as_matrix()
        if not np.allclose(tcp_zero, self.base @ kin.fk(self.q[self.adr]), atol=1e-5):
            raise ValueError(f"{tcp}: MuJoCoとURDFの手先座標が一致しません")
        self.stall_target = None
        self.posture = mink.PostureTask(model, cost=0.)

    def compute_qp_inequalities(self, configuration, dt):
        # Minkの制約は速度ではなく、この周期の変位 G Δq ≤ h。
        return mink.Constraint(self.G, self.bounds)

    @staticmethod
    def error(target, pose):
        return target[:3, 3] - pose[:3, 3], Rotation.from_matrix(target[:3,:3] @ pose[:3,:3].T).as_rotvec()

    @staticmethod
    def cost(dp, dr):
        r2 = dr @ dr
        return np.sqrt(dp @ dp + .09**2 * 2*r2 / (1 + np.sqrt(1 + r2/.15**2)))

    def solve(self, target, seed, config, *, actual, previous_step, holding):
        q = np.asarray(actual)
        # クラッチを入れた手先姿勢の近傍に戻る場合にだけ、元の肘配置を弱く優先。
        if previous_step is None:
            self.ref_frame = self.kin.fk(q)
            ref = self.q.copy()
            ref[self.adr] = q
            self.posture.set_target(ref)
        if not holding or previous_step is None:
            self.stall_target = None
        # 停滞中に実測の微振動を積分しない。解除距離は停止点からの累積なので
        # ゆっくりとした操作でも復帰する。クラッチ再投入時は履歴を引き継がない。
        if self.stall_target is not None:
            p, r = self.error(target, self.stall_target)
            if np.linalg.norm(p) < .004 and np.linalg.norm(r) < .035:
                raise IKTargetUnreachable("停止点の近傍を保持")
            self.stall_target = None
        self.q[self.adr] = q
        self.cfg.update(self.q)
        frame = self.cfg.get_transform_frame_to_world(self.tcp, "body").as_matrix()
        goal = self.base @ target
        dp, dr = self.error(goal, frame)
        if np.linalg.norm(dp) <= config.ik_position_tolerance_m and np.linalg.norm(dr) <= config.ik_rotation_tolerance_rad:
            return seed
        cost = self.cost(dp, dr)
        local = frame.copy()
        local[:3, 3] += dp * min(1., .5*self.dt/max(1e-12,np.linalg.norm(dp)))
        rot = dr * min(1., 2.*self.dt/max(1e-12,np.linalg.norm(dr)))
        local[:3,:3] = Rotation.from_rotvec(rot).as_matrix() @ frame[:3,:3]
        prev = np.zeros(7) if previous_step is None else previous_step
        vmax, dv = 1.2*self.dt, 8.*self.dt**2
        lo = np.maximum(self.kin.lower-seed, -np.minimum(vmax, dv+np.maximum(0., -prev)))
        hi = np.minimum(self.kin.upper-seed, np.minimum(vmax, dv+np.maximum(0., prev)))
        # 指令が実測を先行し過ぎる側に積み増さない。制限を越えても戻り方向は許す。
        lo = np.maximum(lo, np.minimum(0., q-seed-.10))
        hi = np.minimum(hi, np.maximum(0., q-seed+.10))
        self.bounds = np.r_[hi, -lo]
        self.task.set_target(mink.SE3.from_matrix(local))
        self.task.set_orientation_cost(.09 / (1 + dr@dr/.15**2)**.25)
        p, r = self.error(target, self.ref_frame)
        # 開始時の手先から15 cmまたは0.8 rad離れると肘の重みは1/e以下。
        # 目標に収束するほど二次タスクを弱め、位置の定常偏差を作らない。
        self.posture.cost[self.dof] = .035 * np.exp(-(p@p)/.15**2 - (r@r)/.8**2) * min(1., cost/.025)
        try:
            dq = mink.solve_ik(self.cfg, [self.task, self.posture], dt=self.dt, solver="daqp",
                               damping=.06**2, limits=[self])[self.dof]*self.dt
        except mink.NoSolutionFound as exc:
            raise IKTargetUnreachable("Minkの制約内に解がありません") from exc
        if not np.isfinite(dq).all():
            raise ValueError("Minkの出力が非有限値です")
        dq = np.clip(dq, lo, hi)
        for _ in range(6):
            if cost - self.cost(*self.error(target, self.kin.fk(q+dq))) > min(.00035 if holding else .00003, .02*cost**2):
                return seed + dq
            dq *= .5
        self.stall_target = target.copy()
        raise IKTargetUnreachable("Minkの局所目標への進捗なし")
