"""以前のIsaacと同じYUBI魚眼式・取付座標による再投影。"""
from dataclasses import dataclass

import cv2
import mujoco
import numpy as np

CV_TO_MJ = np.diag([1., -1., -1.])
HAND_ROTATION = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])


@dataclass
class Fisheye:
    f: float
    c: np.ndarray
    k1: float
    k2: float
    theta_max: float
    size: tuple

    def __post_init__(self):
        theta = np.linspace(0, self.theta_max, 500)
        values = [self.f, *self.c, self.k1, self.k2, self.theta_max, *self.size]
        if (not np.isfinite(values).all() or self.f <= 0 or not 0 < self.theta_max < np.pi
                or min(self.size) < 1 or np.any(1 + 3*self.k1*theta**2 + 5*self.k2*theta**4 <= 0)):
            raise ValueError('魚眼校正値が不正、または信頼範囲内で非単調です')

    @classmethod
    def from_config(cls, lens, native_size, size):
        scale = np.asarray(size, dtype=float) / native_size
        if not np.allclose(scale, scale[0]):
            raise ValueError('魚眼校正は縦横同率のresizeが必要です')
        return cls(lens['f'] * scale[0], np.asarray(lens['c']) * scale,
                   lens['k1'], lens['k2'], lens['theta_max'], tuple(size))

    def radius(self, theta):
        return self.f * theta * (1 + self.k1*theta**2 + self.k2*theta**4)

    def project(self, points):
        points = np.asarray(points, dtype=float)
        length = np.linalg.norm(points[..., :2], axis=-1)
        theta = np.arctan2(length, points[..., 2])
        scale = np.divide(self.radius(theta), length, out=np.zeros_like(length), where=length > 1e-12)
        return self.c + points[..., :2]*scale[..., None], (theta <= self.theta_max) & (np.linalg.norm(points, axis=-1) > 1e-12)

    def unproject(self, pixels):
        offset = np.asarray(pixels, dtype=float) - self.c
        radius = np.linalg.norm(offset, axis=-1)
        lo, hi = np.zeros_like(radius), np.full_like(radius, self.theta_max)
        for _ in range(45):
            mid = (lo + hi) / 2
            lower = self.radius(mid) < radius
            lo, hi = np.where(lower, mid, lo), np.where(lower, hi, mid)
        theta = (lo + hi) / 2
        scale = np.divide(np.sin(theta), radius, out=np.zeros_like(radius), where=radius > 1e-12)
        return np.concatenate([offset*scale[..., None], np.cos(theta)[..., None]], axis=-1), radius <= self.radius(self.theta_max)


def quat_matrix(xyzw):
    quat = np.asarray(xyzw, dtype=float)[[3, 0, 1, 2]]
    if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-8:
        raise ValueError('カメラのQuaternionが不正です')
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, quat / np.linalg.norm(quat))
    return result.reshape(3, 3)


def mount_from_yubi(gripper_body, config, side):
    """元のX_optical=R(q)(X_camera_link-t)を姿勢用に逆変換する。"""
    prefix = side[0].upper()
    # 左右の指ピボットを、公式hand_rootの中点へ対応付ける。
    pivots = [np.fromstring(gripper_body.find(f"./body[@name='{prefix}_finger_{s}']").get('pos', '0 0 0'), sep=' ')
              for s in ('left', 'right')]
    alignment = np.eye(4)
    alignment[:3, :3] = HAND_ROTATION
    alignment[:3, 3] = np.mean(pivots, axis=0) - HAND_ROTATION @ config['finger_pivot_midpoint_m']
    camera_link = np.eye(4)
    camera_link[:3, 3] = config['camera_link_m']
    values = config['wrist'][side]
    optical = np.eye(4)
    optical[:3, :3] = quat_matrix(values['mount']['q_xyzw']).T
    optical[:3, 3] = values['mount']['t']
    correction = np.eye(4)
    delta = values.get('correction', {})
    correction[:3, 3] = delta.get('xyz_m', [0, 0, 0])
    quat = np.empty(4)
    # SciPyのfrom_euler('xyz')は固定軸。MuJoCoでは大文字が対応する。
    mujoco.mju_euler2Quat(quat, np.deg2rad(delta.get('rpy_deg', [0, 0, 0])), 'XYZ')
    correction[:3, :3] = quat_matrix(quat[[1, 2, 3, 0]])
    return alignment @ camera_link @ optical @ correction


def cube_rotations():
    result = []
    for direction in ([0, 0, 1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, -1]):
        z = np.asarray(direction, dtype=float)
        up = np.array([0., -1., 0.]) if not z[1] else np.array([0., 0., 1.])
        x = np.cross(z, up)
        result.append(np.column_stack([x, np.cross(z, x), z]))
    return result


class FisheyeProjection:
    """固定の逆投影マップを作り、必要な面だけ描画して1回のremapで合成する。"""
    fovy = 92.

    def __init__(self, lens, face_size=320, method='cubemap'):
        self.lens, self.face_size = lens, face_size
        self.method = method
        if method == 'wide_pair':
            self.fovy = 132.
            # 校正済みの各出力光線を覆う広角2方向。投影式・歪み係数は変えない。
            self.rotations = []
            for angle in np.deg2rad([-35., 35.]):
                c, s = np.cos(angle), np.sin(angle)
                self.rotations.append(np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]))
        elif method == 'cubemap':
            self.rotations = cube_rotations()
        else:
            raise ValueError('魚眼描画方式はcubemap/wide_pairで指定してください')
        x, y = np.meshgrid(np.arange(lens.size[0]), np.arange(lens.size[1]))
        rays, self.valid = lens.unproject(np.stack([x, y], axis=-1))
        local = np.stack([rays @ r for r in self.rotations])
        face = np.argmax(local[..., 2], axis=0)
        self.used = [i for i in range(len(self.rotations)) if np.any(self.valid & (face == i))]
        focal = face_size / (2*np.tan(np.deg2rad(self.fovy / 2)))
        u, v = np.full(x.shape, -100., dtype=np.float32), np.full(y.shape, -100., dtype=np.float32)
        for slot, i in enumerate(self.used):
            mask = self.valid & (face == i)
            points = local[i][mask]
            if np.any(points[:, 2] <= 0) or np.max(np.abs(points[:, :2]/points[:, 2, None])) >= np.tan(np.deg2rad(self.fovy/2)):
                raise ValueError('魚眼視野を覆えません。cubemap方式を使用してください')
            uv = focal*points[:, :2]/points[:, 2, None] + face_size/2 - .5
            u[mask], v[mask] = uv[:, 0] + slot*face_size, uv[:, 1]
        self.maps = cv2.convertMaps(u, v, cv2.CV_16SC2)

    def render(self, render_face):
        atlas = np.concatenate([render_face(i) for i in self.used], axis=1)
        return cv2.remap(atlas, *self.maps, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
