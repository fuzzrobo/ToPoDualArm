"""共通モデルとSDKの目標制限を使うMuJoCo実行環境。単位はrad/Nm。"""
from pathlib import Path
import copy
import math
import os
import secrets
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml
# SSHなど表示先のないLinuxでは、画像取得にEGLを使用する。
if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
from topoarm_sdk import load_config
from topoarm_sdk._native import OfflineLimiter
from .settings import ROOT, DEFAULT_CONFIG, load_settings
from .bam import ARM_NAMES

CAMERAS = ("front", "left", "right")


class Simulation:
    def __init__(self, config_file=ROOT/'config/hardware/follower.yaml', *, width=640, height=480,
                 cameras=CAMERAS, viewer=False, floor_enabled=None, floor_z_m=None,
                 floor_margin_m=None, neck_envelope=None, clip_to_limits=None, scene_config=None, bam_config=None,
                 simulation_config=DEFAULT_CONFIG):
        self.config = load_config(str(Path(config_file).resolve()))
        for key, value in {"floor_enabled": floor_enabled, "floor_z_m": floor_z_m,
                           "floor_margin_m": floor_margin_m, "neck_envelope": neck_envelope,
                           "clip_to_limits": clip_to_limits}.items():
            if value is not None:
                setattr(self.config.options, key, value)
        self.names = [joint.name for joint in self.config.joints]
        self.joints = {joint.name: joint for joint in self.config.joints}
        if len(self.names) != 18:
            raise ValueError("シミュレータには両腕・指・首の18軸設定が必要です")
        if width <= 0 or height <= 0 or len(set(cameras)) != len(cameras) or set(cameras) - set(CAMERAS):
            raise ValueError("カメラはfront/left/right、画像サイズは正の整数で指定してください")
        self.width, self.height = width, height
        self.cameras = tuple(cameras)
        self.show_viewer = viewer
        self.viewer = self.renderer = None
        self.fisheye_renderer = None
        self.fisheyes = {}
        self.camera_calibration = {}
        self.model = self.data = None
        self.bam = None
        self.simulation_config = Path(simulation_config).resolve()
        self.scene_config = Path(scene_config).resolve() if scene_config else self.simulation_config
        self.scene = load_settings(self.simulation_config, scene_config)
        from .bam import BamSpec
        self.bam_spec = (BamSpec(bam_config, self.config.joints) if bam_config is not None else
                         BamSpec(self.simulation_config, self.config.joints, settings=self.scene['actuators']))
        self.limiter = OfflineLimiter(self.config)
        self.target = None

    @property
    def is_connected(self):
        return self.data is not None

    def connect(self):
        if self.is_connected:
            raise RuntimeError("シミュレータは起動済みです")
        if self.show_viewer and sys.platform.startswith("linux") and not (
                os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("表示先がありません。--headlessまたはsim_viewer=falseを指定してください")
        model_file = Path(self.config.options.robot_model).with_suffix(".xml")
        xml = ET.parse(model_file).getroot()
        for mesh in xml.findall("./asset/mesh"):
            mesh.set("file", str((model_file.parent / mesh.get("file")).resolve()))
        floor = xml.find("./worldbody/geom[@name='floor']")
        if floor is None:
            raise ValueError("モデルにfloorがありません")
        floor.set("pos", f"0 0 {self.config.options.floor_z_m}")
        asset = xml.find("asset")
        ET.SubElement(asset, "texture", name="sim_sky", type="skybox", builtin="gradient",
                      rgb1=".45 .55 .65", rgb2=".85 .9 .95", width="256", height="1536")
        ET.SubElement(asset, "texture", name="sim_floor", type="2d", builtin="checker",
                      rgb1=".28 .31 .34", rgb2=".45 .48 .51", width="256", height="256")
        ET.SubElement(asset, "material", name="sim_floor", texture="sim_floor", texrepeat="10 10", texuniform="true")
        floor.set("material", "sim_floor")
        floor.set("rgba", "1 1 1 1")
        scene = self.scene
        physics = scene.get('physics', {})
        if physics.get('cone', 'pyramidal') not in ('pyramidal', 'elliptic'):
            raise ValueError('接触摩擦モデルはpyramidal/ellipticで指定してください')
        xml.find('option').set('cone', physics.get('cone', 'pyramidal'))
        impratio = float(physics.get('impratio', 1))
        if not math.isfinite(impratio) or impratio <= 0:
            raise ValueError('接触のimpratioは正数で指定してください')
        xml.find('option').set('impratio', str(impratio))
        render = scene.get('rendering', {})
        if render.get('visual_meshes', True):
            visuals = {}
            for original in list(asset.findall('mesh')):
                low = model_file.parent / 'visual' / (Path(original.get('file')).stem + '.msh')
                if not low.exists():
                    raise ValueError(f'描画用メッシュがありません: {low}。dev/models/build_visual_meshes.pyで生成してください')
                mesh = copy.deepcopy(original)
                mesh.set('name', original.get('name') + '_visual')
                mesh.set('file', str(low))
                mesh.set('inertia', 'convex')
                asset.append(mesh)
                visuals[original.get('name')] = mesh.get('name')
            for geom in xml.findall('.//geom'):
                if geom.get('group') == '2' and geom.get('mesh') in visuals:
                    geom.set('mesh', visuals[geom.get('mesh')])
        # 腕はBAMのmotorへ変更する。未同定の指・首だけを物理単位の位置サーボにする。
        for actuator in xml.findall("./actuator/position"):
            name = actuator.get("joint")
            if name not in self.joints:
                raise ValueError(f"設定にないアクチュエータです: {name}")
            joint = self.joints[name]
            actuator.set("ctrlrange", f"{joint.min} {joint.max}")
            if name not in ARM_NAMES:
                params = scene.get('gripper_actuator' if name in ('R_gripper_joint','L_gripper_joint') else 'neck_actuator', {})
                for key, default in (('kp', 100), ('kv', 10)):
                    value = float(params.get(key, default))
                    if not math.isfinite(value) or value <= 0:
                        raise ValueError('グリッパーのシミュレーションゲインは正数で指定してください')
                    actuator.set(key, str(value))
                force = float(params.get('force_limit_nm', 20))
                if not math.isfinite(force) or force <= 0:
                    raise ValueError('グリッパーのシミュレーショントルク上限は正数で指定してください')
                actuator.set('forcerange', f'{-force} {force}')
        for joint in xml.findall(".//joint"):
            if joint.get("name") in self.joints:
                cfg = self.joints[joint.get("name")]
                joint.set("range", f"{cfg.min} {cfg.max}")
        if self.bam_spec:
            self.bam_spec.prepare_xml(xml)
        for body in xml.findall(".//body"):
            body.set("gravcomp", "0" if self.bam_spec else "1")
        def vector(value, size):
            data = np.asarray(value, dtype=float)
            if data.shape != (size,) or not np.isfinite(data).all():
                raise ValueError('シーンの座標・向きが不正です')
            return ' '.join(map(str, data))
        for name, mount in scene['cameras'].items():
            body_name = mount['parent']
            body = xml.find(f".//body[@name='{body_name}']")
            if body is None:
                raise ValueError(f"カメラ取付リンクがありません: {body_name}")
            if name not in CAMERAS:
                raise ValueError('仮想カメラの名前が不正です')
            if 'lens_file' in mount:
                from .camera import Fisheye, FisheyeProjection, mount_from_yubi, CV_TO_MJ
                calibration = yaml.safe_load((self.scene_config.parent / mount['lens_file']).read_text())
                self.camera_calibration[name] = calibration
                lens = Fisheye.from_config(calibration['wrist'][name], calibration['resolution'], [self.width, self.height])
                face_size = render.get('fisheye_face_size', 320)
                if type(face_size) is not int or not 64 <= face_size <= 2048:
                    raise ValueError('fisheye_face_sizeは64〜2048の整数で指定してください')
                projection = self.fisheyes[name] = FisheyeProjection(lens, face_size, render.get('fisheye_method', 'cubemap'))
                optical = mount_from_yubi(body, calibration, name)
                for i in projection.used:
                    quat = np.empty(4)
                    rotation = optical[:3, :3] @ projection.rotations[i] @ CV_TO_MJ
                    mujoco.mju_mat2Quat(quat, rotation.reshape(-1))
                    ET.SubElement(body, 'camera', name=f'{name}_face_{i}', pos=vector(optical[:3, 3], 3),
                                  quat=vector(quat, 4), fovy=str(projection.fovy))
            else:
                fovy = float(mount['fovy_deg'])
                if not 0 < fovy < 180:
                    raise ValueError('仮想カメラの画角が不正です')
                ET.SubElement(body, 'camera', name=name, pos=vector(mount['position_m'], 3),
                              quat=vector(mount['quat_wxyz'], 4), fovy=str(fovy))
        if set(scene['cameras']) != set(CAMERAS):
            raise ValueError('front/left/rightの仮想カメラが必要です')
        world = xml.find('worldbody')
        table = scene.get('scene', {}).get('table')
        if table:
            ET.SubElement(world, 'geom', name='work_table', type='box', pos=vector(table['position_m'], 3),
                          size=vector(table['half_size_m'], 3), rgba='.65 .55 .4 1')
        for item in scene.get('scene', {}).get('objects', []):
            body = ET.SubElement(world, 'body', name=item['name'], pos=vector(item['position_m'], 3))
            ET.SubElement(body, 'freejoint', name=item['name'] + '_free')
            size = np.asarray(item['half_size_m'], dtype=float)
            if np.any(size <= 0):
                raise ValueError('物体サイズは正数で指定してください')
            inertia = .1 / 3 * np.array([size[1]**2 + size[2]**2, size[0]**2 + size[2]**2, size[0]**2 + size[1]**2])
            ET.SubElement(body, 'inertial', pos='0 0 0', mass='.1', diaginertia=vector(inertia, 3))
            ET.SubElement(body, 'geom', type='box', size=vector(item['half_size_m'], 3),
                          rgba=vector(item['rgba'], 4), mass='.1', friction='1 .005 .0001')
        ET.SubElement(xml.find("worldbody"), "light", pos="0 -1 2", dir="0 0 -1", directional="true")
        visual = xml.find("visual")
        if visual is None:
            visual = ET.SubElement(xml, "visual")
        self.shadows = bool(render.get('shadows', False))
        samples = render.get('samples', 0)
        if type(samples) is not int or samples not in (0, 2, 4, 8):
            raise ValueError('rendering.samplesは0/2/4/8で指定してください')
        ET.SubElement(visual, 'quality', offsamples=str(samples), shadowsize='1024')
        ET.SubElement(visual, "headlight", ambient=".4 .4 .4", diffuse=".6 .6 .6")
        face_size = max((p.face_size for p in self.fisheyes.values()), default=0)
        ET.SubElement(visual, "global", offwidth=str(max(640, self.width, face_size)), offheight=str(max(480, self.height, face_size)))
        collision = scene.get('hand_collision', {})
        reference = None
        if collision.get('simplify', False):
            from .hand_collision import simplify_hands
            reference = simplify_hands(xml, collision)
        try:
            self.model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
            if reference is not None:
                for key in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat'):
                    if not np.allclose(getattr(reference, key), getattr(self.model, key), rtol=1e-12, atol=1e-14):
                        raise ValueError('指の当たり判定変更で質量・慣性が変化しました: ' + key)
            self.data = mujoco.MjData(self.model)
            self.qpos = np.array([self.model.joint(n).qposadr[0] for n in self.names])
            self.dofs = np.array([self.model.joint(n).dofadr[0] for n in self.names])
            self.actuators = np.array([self.model.actuator(n).id for n in self.names])
            if self.bam_spec:
                self.bam = self.bam_spec.bind(self.model, self.data)
            self.reset()
            if self.show_viewer:
                from mujoco import viewer
                self.viewer = viewer.launch_passive(self.model, self.data)
                self.viewer.cam.distance = 1.8
                self.viewer.cam.lookat[:] = [0, 0, .45]
            # 初回GPU準備を記録時間へ含めない。描画も接続時に検証する。
            self.images()
        except BaseException:
            self.close()
            raise

    def _require_connected(self):
        if not self.is_connected:
            raise RuntimeError("シミュレータは未起動です")

    def reset(self):
        self._require_connected()
        mujoco.mj_resetData(self.model, self.data)
        self.limiter.reset()
        initial = np.array([min(j.max, max(j.min, 0.)) for j in self.config.joints])
        self.data.qpos[self.qpos] = initial
        self.target = dict(zip(self.names, map(float, initial)))
        if self.bam:
            self.bam.reset()
        self._apply_targets(self.target)
        mujoco.mj_forward(self.model, self.data)

    def reset_from_state(self, positions, velocities):
        """比較開始時だけ実測状態へ合わせる。比較中の実測角による上書きは行わない。"""
        self._require_connected()
        if set(positions) != set(self.names) or set(velocities) != set(self.names):
            raise ValueError('初期状態には18軸の位置と速度が必要です')
        q, dq = ([float(values[n]) for n in self.names] for values in (positions, velocities))
        if not np.isfinite([q, dq]).all():
            raise ValueError('初期状態に非有限値があります')
        self.reset()
        self.data.qpos[self.qpos], self.data.qvel[self.dofs] = q, dq
        for side in ('R', 'L'):
            self.data.joint(side+'_gripper_mimic').qpos[:] = -positions[side+'_gripper_joint']
            self.data.joint(side+'_gripper_mimic').qvel[:] = -velocities[side+'_gripper_joint']
        if self.bam:
            self.bam.reset()
        self._apply_targets(positions)
        self.limiter.reset()
        mujoco.mj_forward(self.model, self.data)

    def _apply_targets(self, positions):
        self.target = {n: float(positions[n]) for n in self.names}
        for n, i in zip(self.names, self.actuators):
            if self.bam is None or n not in self.bam.names:
                self.data.ctrl[i] = self.target[n]
        if self.bam:
            self.bam.command(self.target)

    def object_poses(self):
        """箱の実際の姿勢を返す。座標とQuaternionはワールド基準。"""
        self._require_connected()
        result = {}
        for item in self.scene.get('scene', {}).get('objects', []):
            body = self.data.body(item['name'])
            quat = body.xquat.copy()
            w, x, y, z = quat
            result[item['name']] = dict(position_m=body.xpos.tolist(), quat_wxyz=quat.tolist(),
                                       yaw_deg=math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
        return result

    def reset_episode(self, positions, *, seed=None):
        """収録外で腕・指・箱を初期化し、物理計算で箱を天板へ落ち着かせる。"""
        self._require_connected()
        if set(positions) != set(self.names):
            raise ValueError('初期姿勢には18軸すべてを指定してください')
        values = np.array([positions[n] for n in self.names], dtype=float)
        if not np.isfinite(values).all() or any(not self.joints[n].min <= v <= self.joints[n].max
                                               for n, v in zip(self.names, values, strict=True)):
            raise ValueError('初期姿勢が非有限または可動域外です')
        scene = self.scene.get('scene', {})
        settings = scene.get('randomization', {})
        extent = np.asarray(settings.get('xy_range_m', [0, 0]), dtype=float)
        yaw = np.asarray(settings.get('yaw_range_deg', [0, 0]), dtype=float)
        distance = float(settings.get('min_center_distance_m', 0))
        settle = float(settings.get('settle_s', .5))
        if (extent.shape != (2,) or yaw.shape != (2,) or not np.isfinite(extent).all()
                or not np.isfinite(yaw).all() or np.any(extent < 0) or yaw[0] > yaw[1]
                or not math.isfinite(distance) or distance < 0 or not 0 < settle <= 1):
            raise ValueError('箱のランダム配置範囲・距離・静止待ち時間が不正です')
        seed = secrets.randbits(63) if seed is None else seed
        rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self.limiter.reset()
        self.data.qpos[self.qpos] = values
        self.target = dict(zip(self.names, map(float, values)))
        if self.bam:
            self.bam.reset()
        self._apply_targets(self.target)
        for prefix in ('R', 'L'):
            self.data.joint(prefix + '_gripper_mimic').qpos[:] = -positions[prefix + '_gripper_joint']
        items = scene.get('objects', [])
        table = scene.get('table')
        object_ids = {self.model.body(item['name']).id for item in items}
        for _ in range(1000):
            chosen = []
            for item in items:
                center = np.asarray(item['position_m'], dtype=float).copy()
                center[:2] += rng.uniform(-extent, extent)
                angle = math.radians(rng.uniform(*yaw))
                half = np.asarray(item['half_size_m'])
                if table:
                    center[2] = table['position_m'][2] + table['half_size_m'][2] + half[2]
                    bounds = np.array([abs(math.cos(angle))*half[0] + abs(math.sin(angle))*half[1],
                                       abs(math.sin(angle))*half[0] + abs(math.cos(angle))*half[1]])
                    if np.any(np.abs(center[:2] - np.asarray(table['position_m'])[:2]) + bounds
                              > np.asarray(table['half_size_m'])[:2]):
                        break
                # 外接円同士の非交差も確認し、大きな箱へ変更した場合も重ねない。
                radius = float(np.linalg.norm(half[:2]))
                if any(np.linalg.norm(center[:2]-p[:2]) < max(distance, radius+r) for p, r in chosen):
                    break
                chosen.append((center, radius))
                self.data.joint(item['name'] + '_free').qpos[:] = [*center, math.cos(angle/2), 0, 0, math.sin(angle/2)]
            else:
                mujoco.mj_forward(self.model, self.data)
                overlap = any(c.dist < -1e-5 and
                              ((self.model.geom_bodyid[c.geom1] in object_ids and self.model.geom_bodyid[c.geom2] != 0)
                               or (self.model.geom_bodyid[c.geom2] in object_ids and self.model.geom_bodyid[c.geom1] != 0))
                              for c in self.data.contact)
                if not overlap:
                    break
        else:
            raise ValueError('指定範囲に重ならない箱配置を作れません')
        self.step(settle)
        for item in items:
            velocity = self.data.joint(item['name'] + '_free').qvel
            if np.linalg.norm(velocity[:3]) > .01 or np.linalg.norm(velocity[3:]) > .1:
                raise RuntimeError('再配置後の箱が静止していません')
        self.episode_initial = dict(seed=int(seed), objects=self.object_poses())
        return copy.deepcopy(self.episode_initial)

    def command(self, positions):
        self._require_connected()
        if set(positions) != set(self.names):
            raise ValueError("目標の関節名が18軸設定と一致しません")
        target = self.limiter.prepare(self.data.qpos[self.qpos].tolist(), [float(positions[n]) for n in self.names])
        self.limiter.commit()
        self._apply_targets(dict(zip(self.names, target)))
        return dict(self.target)

    def step(self, duration):
        self._require_connected()
        if not math.isfinite(duration) or not 0 < duration <= 1:
            raise ValueError("シミュレーション時間は0より大きく1秒以下です")
        dt = float(self.model.opt.timestep)
        # 30Hzなど非整数周期でも端数を失わず、シミュレーション時刻を進める。
        remaining = duration
        try:
            while remaining > 1e-12:
                self.model.opt.timestep = min(dt, remaining)
                if self.bam:
                    self.bam.update()
                mujoco.mj_step(self.model, self.data)
                remaining -= float(self.model.opt.timestep)
        finally:
            self.model.opt.timestep = dt
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("MuJoCoの関節状態が非有限値です")
        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()

    def state(self):
        self._require_connected()
        return {name: {"position": float(self.data.qpos[q]), "velocity": float(self.data.qvel[d]),
                       "effort": float(self.data.qfrc_actuator[d])}
                for name, q, d in zip(self.names, self.qpos, self.dofs, strict=True)}

    def images(self):
        self._require_connected()
        if not self.cameras:
            return {}
        if self.renderer is None:
            from mujoco.rendering.classic.renderer import Renderer
            self.renderer = Renderer(self.model, height=self.height, width=self.width)
            if self.fisheyes:
                size = next(iter(self.fisheyes.values())).face_size
                self.fisheye_renderer = Renderer(self.model, height=size, width=size)
        images = {}
        for name in self.cameras:
            images[name] = self.render_image(name)
        return images

    def render_image(self, name):
        self._require_connected()
        if name in self.fisheyes:
            def face(i):
                self.fisheye_renderer.update_scene(self.data, camera=f'{name}_face_{i}')
                self.fisheye_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = self.shadows
                return self.fisheye_renderer.render().copy()
            return self.fisheyes[name].render(face)
        self.renderer.update_scene(self.data, camera=name)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = self.shadows
        return self.renderer.render().copy()

    def close(self):
        try:
            if self.viewer is not None:
                self.viewer.close()
        finally:
            if self.renderer is not None:
                self.renderer.close()
            if self.fisheye_renderer is not None:
                self.fisheye_renderer.close()
            self.fisheye_renderer = None
            self.fisheyes.clear()
            self.camera_calibration.clear()
            self.viewer = self.renderer = self.data = self.model = None
            self.bam = None
