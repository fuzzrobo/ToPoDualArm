"""mode 3のXMモデルを公式BAMのMuJoCoコントローラへ接続する。実機I/Oは行わない。"""
from pathlib import Path
import hashlib
import importlib.metadata
import json
import math

import numpy as np
import yaml

ARM_NAMES = tuple(f'{s}_joint{i}' for s in ('R', 'L') for i in range(1, 8))


class TimeProfile:
    """時間指定の台形軌道近似。再目標設定時は現在の軌道位置・速度を引き継ぐ。"""
    def __init__(self, duration, acceleration):
        self.duration = np.asarray(duration, dtype=float)
        self.acceleration = np.asarray(acceleration, dtype=float)
        if (not np.isfinite(self.duration).all() or not np.isfinite(self.acceleration).all()
                or np.any(self.duration < 0) or np.any(self.acceleration < 0)
                or np.any(2*self.acceleration > self.duration)):
            raise ValueError('時間プロファイルは0≦加速時間≦全体時間/2です')

    def reset(self, positions, velocities=None, now=0.):
        self.start = np.asarray(positions, dtype=float).copy()
        self.goal = self.start.copy()
        self.velocity = np.zeros_like(self.start) if velocities is None else np.asarray(velocities, dtype=float).copy()
        self.started = np.full_like(self.start, now)

    def sample(self, now):
        t = np.clip(now-self.started, 0, self.duration)
        T, a = self.duration, self.acceleration
        distance = self.goal-self.start
        peak = np.divide(distance-.5*self.velocity*a, T-a, out=np.zeros_like(distance), where=T > 0)
        rate = np.divide(peak-self.velocity, a, out=np.zeros_like(peak), where=a > 0)
        braking = np.divide(peak, a, out=np.zeros_like(peak), where=a > 0)
        position = self.start + .5*(self.velocity+peak)*a + peak*(t-a)
        speed = peak.copy()
        first = (a > 0) & (t < a)
        last = (a > 0) & (t > T-a)
        position = np.where(first, self.start+self.velocity*t+.5*rate*t*t, position)
        speed = np.where(first, self.velocity+rate*t, speed)
        position = np.where(last, self.goal-.5*braking*(T-t)**2, position)
        speed = np.where(last, braking*(T-t), speed)
        finished = (T == 0) | (now-self.started >= T)
        return np.where(finished, self.goal, position), np.where(finished, 0., speed)

    def command(self, goals, now):
        goals = np.asarray(goals, dtype=float)
        position, velocity = self.sample(now)
        changed = goals != self.goal
        self.start[changed], self.velocity[changed] = position[changed], velocity[changed]
        self.started[changed], self.goal[changed] = now, goals[changed]


class BamSpec:
    def __init__(self, path, joints, *, settings=None):
        self.path = Path(path).resolve()
        raw = yaml.safe_load(self.path.read_text()) if settings is None else settings
        self.settings = raw.get('actuators', raw)
        if self.settings.get('operating_mode') != 3:
            raise ValueError('BAM比較はmode 3専用です')
        self.joints = {j.name: j for j in joints}
        if not set(ARM_NAMES) <= self.joints.keys():
            raise ValueError('両腕14軸の設定が必要です')
        self.models, self.groups, assigned = {}, {}, set()
        self.provenance = {'bam_version': importlib.metadata.version('better-actuator-models'),
                           'config': str(self.path), 'models': {}, 'assumptions': [
            '同定時の棒・取付部品の質量は0kg。見かけの慣性を含む暫定モデル。',
            'q_offsetは単体振り子の取付誤差なのでロボット関節の原点には適用しない。',
            '目標角入力のD換算・時間プロファイルは近似。実測PWM入力と分けて評価する。']}
        for model_name, item in self.settings['models'].items():
            path = (self.path.parent/item['parameters']).resolve()
            params = json.loads(path.read_text())
            if (len(set(item['ids'])) != len(item['ids']) or
                    any(not math.isfinite(float(item[k])) or float(item[k]) <= 0 for k in ('voltage_v','error_gain'))):
                raise ValueError('BAMのモーターID・電圧・換算値が不正です')
            if params.get('actuator') != item['actuator'] or params.get('model') != 'm4':
                raise ValueError(f'M4のモデル種別が一致しません: {path}')
            if (any(not math.isfinite(v) for v in params.values() if isinstance(v, (int, float)))
                    or any(params[k] <= 0 for k in ('kt', 'R', 'armature'))):
                raise ValueError('BAMパラメータに非有限値があります')
            names = [n for n in ARM_NAMES if self.joints[n].id in item['ids']]
            if not names or len(names) != len(item['ids']) or assigned.intersection(names):
                raise ValueError('BAMの関節対応が空または重複しています')
            for n in names:
                j = self.joints[n]
                if j.operating_mode != 3 or abs(j.multiplier) != 1:
                    raise ValueError(f'{n}はmode3・倍率±1である必要があります')
                if j.position_ki != 0:
                    raise ValueError('BAMの目標角入力はI=0専用です')
                if j.profile_duration_ms < 0 and (j.profile_velocity or j.profile_acceleration):
                    raise ValueError('BAMでは時間指定またはプロファイルなしを使用してください')
            assigned.update(names)
            self.groups[model_name] = names
            self.models[model_name] = (params, item)
            self.provenance['models'][model_name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                joints=names, source=item.get('source'), parameters=params)
        if assigned != set(ARM_NAMES):
            raise ValueError('BAMのモデル対応は両腕14軸を過不足なく指定してください')
        derivative = float(self.settings['derivative_sample_s'])
        if not math.isfinite(derivative) or derivative <= 0:
            raise ValueError('D換算用の仮定周期は正の有限値です')

    def prepare_xml(self, xml):
        actuators = xml.find('actuator')
        found = set()
        for actuator in actuators:
            if actuator.get('joint') in ARM_NAMES:
                name, joint = actuator.get('name'), actuator.get('joint')
                actuator.tag = 'motor'
                actuator.attrib.clear()
                actuator.attrib.update(name=name, joint=joint, gear='1', ctrllimited='false', forcelimited='false')
                found.add(joint)
        if found != set(ARM_NAMES):
            raise ValueError('MJCFに両腕14軸のアクチュエータがありません')

    def bind(self, model, data):
        return BamDynamics(self, model, data)


class BamDynamics:
    def __init__(self, spec, model, data):
        from bam.actuator import VoltageControlledActuator
        from bam.model import models
        from bam.mujoco import MujocoController
        from bam.testbench import Pendulum

        class XMMode3(VoltageControlledActuator):
            def get_extra_inertia(self):
                return self.model.armature.value

            def compute_control(self, q_target, q, dq, dt):
                if self.input_kind == 'pwm':
                    return self.vin * np.clip(self.pwm, -1, 1)
                qref, vref = self.profile.sample(data.time)
                if self.input_kind == 'trajectory':
                    qref, vref = self.trajectory, self.trajectory_velocity
                # Dの内部サンプリング周期は未同定。仮定値を設定・評価結果に残す。
                duty = ((qref-q)*self.kp*self.error_gain +
                        (vref-dq)*self.kd*self.derivative_gain) / self.sign
                return self.vin*np.clip(duty, -self.max_pwm, self.max_pwm)

            def compute_torque(self, control, torque_enable, q, dq):
                return super().compute_torque(control, torque_enable, q, dq/self.sign) * self.enabled/self.sign

        self.spec, self.model, self.data = spec, model, data
        self.controllers, self.actuators = [], []
        for key, names in spec.groups.items():
            params, settings = spec.models[key]
            fitted = models['m4']()
            act = XMMode3(Pendulum, vin=float(settings['voltage_v']), kp=np.array([spec.joints[n].position_kp for n in names]),
                          error_gain=float(settings['error_gain']))
            fitted.set_actuator(act)
            fitted.actuator_name = params['actuator']
            # armatureはアプリ独自登録で作成してから同定値を読み込む。
            from bam.parameter import Parameter
            fitted.armature = Parameter(params['armature'], params['armature'], params['armature'])
            fitted.load_parameters_from_dict(params)
            act.kd = np.array([spec.joints[n].position_kd for n in names])
            act.derivative_gain = 4096/(2*np.pi*16*885) * float(spec.settings['derivative_sample_s'])
            act.sign = np.array([spec.joints[n].multiplier for n in names])
            act.names, act.input_kind = names, 'targets'
            act.enabled = np.ones(len(names))
            act.pwm = np.zeros(len(names))
            act.profile = TimeProfile([max(0, spec.joints[n].profile_duration_ms)/1000 for n in names],
                                      [max(0, spec.joints[n].profile_acceleration_ms)/1000 for n in names])
            controller = MujocoController(fitted, names, model, data)
            # BAM1.0.2の摩擦拘束はDOF番号。freejointが先行しても正しく除外する。
            # 初期化時の関節→qpos/DOF変換が終わった後、update内の照合番号を補正する。
            controller.joint_indexes = list(controller.dof_indexes)
            self.controllers.append(controller)
            self.actuators.append(act)
        self.names = set(ARM_NAMES)
        self.reset()

    def reset(self):
        for controller, act in zip(self.controllers, self.actuators):
            q = self.data.qpos[controller.qpos_indexes].copy()
            act.profile.reset(q, now=self.data.time)
            act.trajectory, act.trajectory_velocity = q, np.zeros_like(q)
            controller.last_ts = self.data.time
            act.input_kind = 'targets'

    def command(self, positions):
        for controller, act in zip(self.controllers, self.actuators):
            target = np.array([positions[n] for n in act.names])
            act.profile.command(target, self.data.time)
            for n, q in zip(act.names, target):
                controller.set_q_target(n, q)

    def inputs(self, kind, *, pwm=None, voltage=None, trajectory=None, trajectory_velocity=None, enabled=None):
        if kind not in ('targets', 'pwm', 'trajectory'):
            raise ValueError('BAM入力はtargets/pwm/trajectoryです')
        required = {'pwm': (pwm, voltage), 'trajectory': (trajectory, trajectory_velocity), 'targets': ()}[kind]
        if any(value is None for value in required):
            raise ValueError('BAM再生に必要な入力がありません')
        for act in self.actuators:
            act.input_kind = kind
            for attr, values in (('pwm', pwm), ('vin', voltage), ('trajectory', trajectory),
                                 ('trajectory_velocity', trajectory_velocity), ('enabled', enabled)):
                if values is not None:
                    value = np.array([values[n] for n in act.names], dtype=float)
                    if not np.isfinite(value).all() or (attr == 'vin' and np.any(value <= 0)):
                        raise ValueError(f'BAM入力が不正です: {attr}')
                    setattr(act, attr, value)

    def update(self):
        for controller in self.controllers:
            controller.update()

    def currents(self):
        result = {}
        for controller, act in zip(self.controllers, self.actuators):
            current = self.data.qfrc_actuator[controller.dof_indexes] * act.sign / act.model.kt.value
            result.update(zip(act.names, map(float, current)))
        return result
