"""Quest実機に接続せず、viewerと同じ直接姿勢・床保護・MuJoCoを20 Hzで検証する。"""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import draccus
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples"))
from episode import QuestReader
from lerobot_teleoperator_yubi.config import ARMS, JOINTS, SIDES, YubiTeleoperatorConfig
from lerobot_teleoperator_yubi.processor import YubiActionStep
from topoarm_recording.simulation import SimTarget


def check(case, output, seed):
    path = ROOT / "config/robots/mujoco.yaml"
    sim = SimTarget(yaml.safe_load(path.read_text()), path.parent,
                    scene=ROOT/"dev/viewer/stack_boxes.yaml", cameras=())
    cfg = draccus.decode(YubiTeleoperatorConfig, json.loads((ROOT/"config/yubi.json").read_text()))
    cfg.urdf_path = str(ROOT/"models/300mm/robot.urdf")
    model = SimpleNamespace(type="topoarm_mujoco", joints=JOINTS, config_file=str(sim.hardware))
    pos, measured, holds, errors, angles, measured_errors, measured_angles, costs = [], [], [], [], [], [], [], []
    rng = np.random.default_rng(seed)
    with sim:
        sim.enable()
        home = sim.read().position
        step = YubiActionStep(cfg, model, direct_quest_pose=sim.home["pose"], simulation=sim)
        reader = QuestReader(step, lambda: raw, JOINTS)
        origins = {s: step.ik[s].fk([home[n] for n in ARMS[s]]) for s in SIDES}
        bank = {s:[origins[s]] for s in SIDES}
        if case == "workspace":
            for s in SIDES:
                kin = step.ik[s]
                q0 = np.array([home[n] for n in ARMS[s]])
                while len(bank[s]) <= 12:
                    angles_q = np.clip(q0 + rng.uniform(-1.1,1.1,7), kin.lower+.02, kin.upper-.02)
                    goal = kin.fk(angles_q)
                    if goal[2,3] > .08:
                        bank[s].append(goal)
        for i in range(12*120 if case == "workspace" else 360):
            raw = {}
            targets = {}
            for s, sign in zip(SIDES, (1,-1), strict=True):
                # 通常姿勢→手首を捻りながら限界の外→静止時の微小揺れ→作業域へ復帰。
                frac = np.interp(i, [0,20,65,205,250,359], [0,0,1,1,0,0])
                goal = origins[s].copy()
                goal[:3,3] += frac*np.array([.50,.16*sign,.10])
                rv = np.array([.4,1.4*sign,.65])*frac
                if 66 <= i < 205:
                    goal[:3,3] += rng.normal(0,.0008,3)
                    rv += rng.normal(0,.005,3)
                goal[:3,:3] = Rotation.from_rotvec(rv).as_matrix() @ goal[:3,:3]
                if case == "near":
                    # FKで作る既知の到達可能な経路。両腕の各関節を別々に動かす。
                    t = np.interp(i, [0,20,100,180,250,359], [0,0,1,-.6,0,0])
                    qs = [home[n] for n in ARMS[s]] + sign*t*np.array([.18,-.12,.20,-.18,.34,.15,-.25])
                    goal = step.ik[s].fk(qs)
                elif case == "wrist":
                    goal = origins[s].copy()
                    goal[:3,:3] = Rotation.from_rotvec([.2*frac, 0., 2.8*sign*frac]).as_matrix() @ goal[:3,:3]
                    goal[2,3] += .055*frac
                elif case == "workspace":
                    j, off = divmod(i,120)
                    a,b = bank[s][j:j+2]
                    u = min(off/42, 1.)
                    goal = a.copy()
                    goal[:3,3] += u*(b[:3,3]-a[:3,3])
                    rel = Rotation.from_matrix(b[:3,:3] @ a[:3,:3].T).as_rotvec()
                    goal[:3,:3] = Rotation.from_rotvec(u*rel).as_matrix() @ a[:3,:3]
                targets[s] = goal
                h = goal.copy()
                h[:3,3] -= step.quest_origin
                h = np.linalg.inv(step.world) @ h @ np.linalg.inv(step.mount[s])
                h[:3,3] = h[:3,3]/cfg.position_scale + step.quest_neck_pivot
                for k,v in enumerate(h[:3,3]): raw[f"{s}.p{k}"] = v
                for k,v in enumerate(Rotation.from_matrix(h[:3,:3]).as_quat()): raw[f"{s}.q{k}"] = v
                raw.update({f"{s}.tracked":True, f"{s}.clutch":i>0, f"{s}.releases":1,
                            f"{s}.opening":0., f"{s}.losses":0})
                if case == "near" and s == "left":
                    raw[f"{s}.tracked"] = not 44 <= i < 52
                    raw[f"{s}.losses"] = int(i>=44)
                    raw[f"{s}.clutch"] = i>0 and not 145 <= i < 158
                    raw[f"{s}.releases"] += int(i>=145)
            raw["received"] = time.monotonic()
            for s in SIDES: raw[f"{s}.opening_received"] = raw["received"]
            state = sim.read().position
            started = time.monotonic()
            q = reader(i,state)
            costs.append(time.monotonic()-started)
            applied = sim.command(q)
            reader.accepted(applied)
            pos.append([[applied[n] for n in ARMS[s]] for s in SIDES])
            measured.append([[state[n] for n in ARMS[s]] for s in SIDES])
            holds.append([step.ik_hold[s] for s in SIDES])
            errors.append([np.linalg.norm(step.ik[s].fk(pos[-1][j])[:3,3]-targets[s][:3,3]) for j,s in enumerate(SIDES)])
            angles.append([Rotation.from_matrix(targets[s][:3,:3] @ step.ik[s].fk(pos[-1][j])[:3,:3].T).magnitude()
                           for j,s in enumerate(SIDES)])
            measured_fk = [step.ik[s].fk(measured[-1][j]) for j,s in enumerate(SIDES)]
            measured_errors.append([np.linalg.norm(targets[s][:3,3]-v[:3,3]) for s,v in zip(SIDES,measured_fk)])
            measured_angles.append([Rotation.from_matrix(targets[s][:3,:3]@v[:3,:3].T).magnitude() for s,v in zip(SIDES,measured_fk)])
    p, m, h, e = map(np.asarray, (pos, measured, holds, errors))
    assert np.isfinite(p).all() and np.isfinite(m).all()
    for j,s in enumerate(SIDES):
        assert (p[:,j] >= step.ik[s].lower-1e-8).all() and (p[:,j] <= step.ik[s].upper+1e-8).all()
    if output: np.savez(output, command=p, measured=m, hold=h, error=e, angle=angles,
                        measured_error=measured_errors, measured_angle=measured_angles, reader_s=costs)
    if case == "workspace":
        print("両腕処理 p95 ms", round(np.percentile(costs,95)*1000,2),
              "実測角・最大二階差分", round(float(np.abs(np.diff(m,n=2,axis=0)).max()),5))
        for tag, pe, re in (("command",e,np.asarray(angles)),("measured",np.asarray(measured_errors),np.asarray(measured_angles))):
            pe, re = pe[119::120], re[119::120]
            print(tag, "24 targets", "position<=5mm", int((pe<=.005).sum()), "6D<=5mm/5deg",
                  int(((pe<=.005)&(re<=np.deg2rad(5))).sum()), "median_mm",round(float(np.median(pe)*1000),2))
            if seed == 285 and tag == "measured":
                assert np.median(pe) < .002 and (pe<=.005).sum() >= 22
                assert ((pe<=.005)&(re<=np.deg2rad(5))).sum() >= 18
        return
    window = slice(130,205)
    def peak(x): return round(float(np.abs(x).max()),5)
    switches = np.count_nonzero(np.diff(h,axis=0),axis=0)
    print(json.dumps(dict(case=case, hold_switches=switches.tolist(),
                         hold_fraction=np.mean(h[window],axis=0).round(3).tolist(),
                         boundary_cmd_dq=peak(np.diff(p[window],axis=0)), boundary_measured_dq=peak(np.diff(m[window],axis=0)),
                         peak_dq=peak(np.diff(p,axis=0)), peak_ddq=peak(np.diff(p,n=2,axis=0)),
                         measured_ddq=peak(np.diff(m,n=2,axis=0)),
                         final_rotation_rad=np.round(angles[-1],4).tolist(),
                         final_error_m=e[-1].round(5).tolist(), boundary_error_m=e[window].mean(axis=0).round(4).tolist(),
                         final_measured_mm=np.round(np.asarray(measured_errors[-1])*1000,2).tolist(),
                         p95_ik_ms=round(np.percentile(costs,95)*1000,2))))
    assert peak(np.diff(p,axis=0)) <= .06001
    if seed == 285:
        assert max(measured_errors[-1]) < .002 and max(measured_angles[-1]) < np.deg2rad(1.)
    if case == "reach":
        assert np.all(switches == 2) and np.all(h[window])
    if case == "wrist":
        assert np.all(switches == 2)
    if case == "near":
        assert np.max(np.abs(np.diff(p[43:52,1],axis=0))) < 1e-7
        assert np.max(np.abs(np.diff(p[144:158,1],axis=0))) < 1e-7
        assert np.ptp(p[:,0],axis=0).max() > .1 and not h.any()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--output")
    p.add_argument("--case", choices=("reach","near","wrist","workspace"), default="reach")
    p.add_argument("--seed",type=int,default=285)
    args = p.parse_args()
    check(args.case,args.output,args.seed)
