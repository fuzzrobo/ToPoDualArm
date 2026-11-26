"""実機ビューアでACT、Diffusion Policy、SmolVLAを実行する。"""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import threading
import traceback
import time
from tkinter import ttk

import cv2
import numpy as np
import torch
import yaml
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from topoarm_recording.mac_cameras import CameraStreams
from topoarm_sdk import Device

from camera_window import CAMERA_TITLES, CameraWindow

WRISTS = ("left", "right")


class InitialPoseMove:
    def __init__(self, target):
        self.target = target
        self.start = None
        self.start_time = self.settled_since = None

    def update(self, state, now):
        if self.start is None:
            self.start, self.start_time = state.copy(), now
            distance = max(abs(self.target[n] - self.start[n]) for n in self.target)
            # 五次補間の最大速度を約20°/s、最大加速度を1 rad/s²に抑える。
            self.duration = max(1., 1.875 * distance / .35, math.sqrt(5.774 * distance))
        elapsed = now - self.start_time
        s = min(elapsed / self.duration, 1.)
        if s < 1.:
            blend = s**3 * (10. + s * (-15. + 6. * s))
            return {n: self.start[n] + blend * (self.target[n] - self.start[n]) for n in self.target}, "moving"
        error = max(abs(self.target[n] - state[n]) for n in self.target)
        if error <= math.radians(5.):
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= .15:
                return self.target, "ready"
        else:
            self.settled_since = None
        if elapsed > self.duration + 2.:
            return state, "timeout"
        return self.target, "moving"


def configured_initial_pose(cfg, robot, *, folder=None):
    pose = cfg.get("initial_pose_rad")
    if "initial_pose_file" in cfg:
        if pose is not None:
            raise ValueError("initial_pose_radとinitial_pose_fileはどちらか一方を指定してください")
        if folder is None:
            raise ValueError("初期姿勢ファイルの基準ディレクトリがありません")
        file = folder / cfg["initial_pose_file"]
        pose = yaml.safe_load(file.read_text())
        if not isinstance(pose, dict):
            raise ValueError(f"初期姿勢ファイルには関節名と角度を指定してください: {file}")
    if pose is None:
        return None
    if not isinstance(pose, dict) or set(pose) != set(robot.joints):
        raise ValueError("initial_pose_radには全関節の角度をradで指定してください")
    pose = {n: float(pose[n]) for n in robot.joints}
    for joint in robot.cfg.joints:
        if not joint.min <= pose[joint.name] <= joint.max:
            raise ValueError(f"初期姿勢が関節可動域外です: {joint.name}")
    return pose


class BoxpackPolicy:
    def __init__(self, path, device, joints, backbone=None):
        schema = json.loads((path/"topoarm_schema.json").read_text())
        self.hz = int(schema["fps"])
        self.state_names = schema["features"]["observation.state"]["names"]
        self.action_names = schema["features"]["action"]["names"]
        if any(len(names) != len(joints) or set(names) != set(joints)
               for names in (self.state_names, self.action_names)):
            raise ValueError("モデルの関節と実機の関節が一致しません")
        cfg = PreTrainedConfig.from_pretrained(str(path), local_files_only=True)
        if cfg.type not in ("act", "diffusion", "smolvla") or cfg.n_obs_steps != 1:
            raise ValueError("観測1ステップのACT、Diffusion Policy、SmolVLAを指定してください")
        self.kind = cfg.type
        self.label = {"act": "ACT", "diffusion": "DP", "smolvla": "SmolVLA"}[self.kind]
        self.views = tuple(key.removeprefix("observation.images.") for key in cfg.image_features)
        if set(self.views) not in (set(WRISTS), set(CAMERA_TITLES)):
            raise ValueError(f"モデルのカメラ構成が未対応です: {self.views}")
        expected = {"observation.state": [len(joints)]}
        expected.update({"observation.images."+s: [3,240,320] for s in self.views})
        if {k:list(v.shape) for k,v in cfg.input_features.items()} != expected:
            raise ValueError("推論には各視点の320×240画像と実機関節状態が必要です")
        cfg.device = device
        preprocessor_overrides = {"device_processor": {"device": device}}
        if self.kind == "smolvla":
            if backbone is None or not (backbone / "config.json").is_file():
                raise ValueError("SmolVLAにはローカルのVLM設定とトークナイザーを指定してください")
            # VLMを含む学習済みの全重みを、このチェックポイントから復元する。
            cfg.load_vlm_weights = False
            cfg.vlm_model_name = str(backbone)
            preprocessor_overrides["tokenizer_processor"] = {"tokenizer_name": str(backbone)}
        else:
            cfg.pretrained_backbone_weights = None
        self.policy = get_policy_class(self.kind).from_pretrained(str(path), config=cfg, strict=True, local_files_only=True)
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=str(path),
            preprocessor_overrides=preprocessor_overrides)
        self.chunk_size = cfg.n_action_steps if self.kind == "diffusion" else cfg.chunk_size

    @torch.inference_mode()
    def predict(self, images, state, *, task=None):
        started = time.monotonic()
        obs = {"observation.state":torch.tensor([state[n] for n in self.state_names], dtype=torch.float32)}
        for s in self.views:
            obs["observation.images."+s] = torch.from_numpy(images[s]).permute(2,0,1).float().div_(255.)
        if self.kind == "smolvla":
            if not isinstance(task, str) or not task.strip():
                raise ValueError("SmolVLAの指示文が空です")
            obs["task"] = task
        batch = self.pre(obs)
        if self.kind == "diffusion":
            # DPは状態に明示的な観測時刻の次元を必要とする。
            batch["observation.state"] = batch["observation.state"].unsqueeze(1)
        chunk = self.post(self.policy.predict_action_chunk(batch)).cpu().numpy()
        if chunk.shape != (1,self.chunk_size,len(self.action_names)) or not np.isfinite(chunk).all():
            raise ValueError("モデルの出力形状または数値が不正です")
        return chunk[0], (time.monotonic()-started)*1000


def run_inference(config_path):
    config_path = Path(config_path).resolve()
    cfg = yaml.safe_load(config_path.read_text())
    folder = config_path.parent
    robot = Device(folder/cfg["robot"], read_profile="teleop")
    if robot.read_only:
        raise ValueError("フォロワーを指定してください")
    initial_pose = configured_initial_pose(cfg, robot, folder=folder)
    if initial_pose is not None:
        print(f"初期姿勢を設定（{len(initial_pose)}軸、推論開始時に移動）", flush=True)
    configured_cameras = yaml.safe_load((folder/cfg["cameras"]).read_text())
    if not isinstance(configured_cameras, dict) or set(configured_cameras) not in (set(WRISTS), set(CAMERA_TITLES)):
        raise ValueError("実カメラ設定にはleft/rightが必要です。frontは省略できます")
    checkpoint = (folder / cfg["checkpoint"]).resolve()
    backbone = (folder / cfg["backbone"]).resolve() if cfg.get("backbone") else None
    model = BoxpackPolicy(checkpoint, cfg["device"], robot.joints, backbone)
    if missing := set(model.views) - configured_cameras.keys():
        raise ValueError(f"モデルが必要とするカメラがありません: {', '.join(sorted(missing))}")
    devices = {side: configured_cameras[side] for side in CAMERA_TITLES if side in model.views}
    rate = float(cfg["replan_hz"])
    if not math.isfinite(rate) or not 0 < rate <= model.hz or rate*model.chunk_size < model.hz:
        raise ValueError("再推論の周期は指令周期以上、チャンク長以下にしてください")

    models, initial_poses, tasks = [model], [initial_pose], [cfg["task"]]
    model_label = f"{checkpoint.parent.name}/{checkpoint.name}"
    print(f"{model.label}モデル {model_label}　入力視点 {model.views}", flush=True)
    hands = None
    cycle, one_shot = cfg.get("alternation"), cfg.get("one_shot")
    if cycle and one_shot:
        raise ValueError("交互推論と単方向デモは同時に指定できません")
    if transfer_cfg := cycle or one_shot:
        from boxpack_cycle import BoxpackHands, TransferProgress

        first_source = transfer_cfg["first_source"]
        if first_source not in WRISTS:
            raise ValueError("first_sourceにはleftまたはrightを指定してください")
        sources = (first_source, next(side for side in WRISTS if side != first_source))
        directions = tuple("右→左" if side == "right" else "左→右" for side in sources)
        hands = BoxpackHands(folder / transfer_cfg["kinematics"], robot.cfg)
    if cycle:
        peer_cfg = (folder / cycle["reverse_config"]).resolve()
        peer = yaml.safe_load(peer_cfg.read_text())
        if model.kind == "smolvla":
            prompts = json.loads((folder / cycle["prompts"]).read_text())
            tasks = [prompts[f"{side}_to_{sources[1-i]}"] for i, side in enumerate(sources)]
            if any(not isinstance(task, str) or not task.strip() for task in tasks):
                raise ValueError("交互推論には2方向の指示文が必要です")
            models.append(model)
            print(f"SmolVLA交互推論（{model.hz} Hz） {directions[0]} / {directions[1]}", flush=True)
        else:
            peer_path = (peer_cfg.parent / peer["checkpoint"]).resolve()
            reverse = BoxpackPolicy(peer_path, cfg["device"], robot.joints)
            if (model.kind != "act" or reverse.kind != "act"
                    or (reverse.hz, reverse.chunk_size, reverse.views) != (model.hz, model.chunk_size, model.views)):
                raise ValueError("交互推論には同じ観測・指令周期のACTモデルを2本指定してください")
            models.append(reverse)
            print(f"ACT交互推論 {directions[0]}: {checkpoint.parent.name}、{directions[1]}: {peer_path.parent.name}", flush=True)
        reverse_pose = configured_initial_pose(peer, robot, folder=peer_cfg.parent)
        if initial_pose is None or reverse_pose is None:
            raise ValueError("交互推論には両方向の初期姿勢が必要です")
        initial_poses.append(reverse_pose)
        model_label = "交互 " + " / ".join(directions)
    elif one_shot:
        model_label = f"単方向デモ {directions[0]}"
        print(f"{model.label}単方向デモ {directions[0]}", flush=True)
    view = CameraWindow(f"実機・{model.label}推論 | {model_label}", devices)
    controls = SimpleQueue()
    quit_event = threading.Event()
    ttk.Label(view.controls,text=cfg["task"]).pack(side="left")
    start_button = ttk.Button(view.controls,text="推論開始",state="disabled",command=lambda:controls.put("start"))
    start_button.pack(side="left")
    ttk.Button(view.controls,text="停止・姿勢保持",command=lambda:controls.put("stop")).pack(side="left")
    status = ("接続中",False)
    display_images = {}
    connected = threading.Event()

    def control(streams):
        nonlocal status, display_images
        index, completed = 0, 0
        model = models[index]
        transfer = None
        active = enabled = warmed = False
        warmup_index = 0
        generation = 0
        pending = chunk = None
        next_infer = chunk_stamp = 0.
        infer_ms = None
        last_stamps, resized = {}, {}
        reported_cameras = False
        hold_after_gap = camera_hold = False
        move = None
        notice = ""
        with ExitStack() as resources:
            while not quit_event.is_set():
                try:
                    resources.enter_context(robot)
                except RuntimeError as exc:
                    waiting = (f"フォロワー接続待ち　{exc}", False)
                    if waiting != status:
                        print(waiting[0], flush=True)
                    status = waiting
                    if quit_event.wait(3):
                        return
                else:
                    connected.set()
                    break
            if quit_event.is_set():
                return
            executor = resources.enter_context(ThreadPoolExecutor(max_workers=1))
            try:
                deadline = time.monotonic()
                while not quit_event.is_set():
                    streams.reconnect_stale()
                    try:
                        state = robot.read().position
                    except RuntimeError as exc:
                        if "読出し周期が欠落" not in str(exc):
                            raise
                        hold_after_gap = active or hold_after_gap
                        if move is not None or (active and hands is not None):
                            move = None
                            transfer = None
                            active = False
                            notice = "関節角の欠落で実機指令を中断・姿勢保持"
                        generation += 1
                        chunk = None
                        next_infer = 0.
                        status = ("関節角の再受信中・指令保留", False)
                        if quit_event.wait(.02):
                            return
                        continue
                    raw, stamps, _ = streams.latest_per_camera()
                    now = time.monotonic()
                    for s, frame in raw.items():
                        if stamps[s] != last_stamps.get(s):
                            resized[s] = cv2.resize(frame,(320,240),interpolation=cv2.INTER_AREA)
                    last_stamps = stamps
                    display_images = {s:resized[s] for s in raw if now-stamps[s] <= .15}
                    # 各カメラの最新画像を独立に使う。1秒は取得スレッドの再接続基準と同じ。
                    missing = [s for s in model.views if s not in raw or now - stamps[s] > 1.]
                    fresh = not missing
                    if not reported_cameras and len(display_images) == len(devices):
                        camera_names = "・".join(view.titles.values())
                        print(f"{camera_names}の実画像、全関節の実測を受信", flush=True)
                        reported_cameras = True
                    requested = active
                    while True:
                        try:
                            event = controls.get_nowait()
                        except Empty:
                            break
                        requested = event == "start" and fresh and warmed
                        notice = ""
                        transfer = None
                        generation += 1
                        chunk = None
                        next_infer = 0.
                    if requested and not active:
                        pose = initial_poses[index]
                        if not enabled:
                            try:
                                robot.enable()
                            except RuntimeError as exc:
                                if "読出し周期が欠落" not in str(exc):
                                    raise
                                status = ("関節角の再受信中・実機指令OFF", False)
                                continue
                            active = enabled = True
                            move = None if pose is None else InitialPoseMove(pose)
                            continue  # 有効化後の実測と画像を取り直す。
                        if pose is not None:
                            move = InitialPoseMove(pose)
                    if active and not fresh and (move is not None or hands is not None):
                        requested = False
                        generation += 1
                        chunk = None
                        next_infer = 0.
                        transfer = None
                        notice = "映像の欠落で実機指令を中断・姿勢保持"
                    if (active and not requested) or hold_after_gap:
                        robot.command(state)
                        hold_after_gap = False
                        if not requested:
                            move = None
                        active = requested
                        continue  # 保持指令を送った周期は、次の実測を待つ。
                    active = requested
                    if not fresh and active and not camera_hold:
                        robot.command(robot.read().position)
                        camera_hold = True
                        generation += 1
                        chunk = None
                        next_infer = 0.
                    if fresh or not active:
                        camera_hold = False
                    if move is not None and now >= next_infer:
                        target, outcome = move.update(state, time.monotonic())
                        robot.command(target)
                        if outcome == "ready":
                            print(f"初期姿勢に到達・{model.label}推論開始", flush=True)
                            move = None
                            notice = ""
                        elif outcome == "timeout":
                            notice = "基準姿勢に届かなかったため停止・姿勢保持"
                            print(notice, flush=True)
                            active = False
                            move = None
                    if hands is not None and move is None and active and fresh:
                        if transfer is None:
                            transfer = TransferProgress(sources[index], now)
                        outcome = transfer.update(now, hands.read(state, transfer.source),
                                                  hands.read(state, transfer.destination))
                        if outcome != "ongoing":
                            robot.command(state)
                            generation += 1
                            chunk = None
                            if outcome == "placed":
                                completed += 1
                                print(f"{directions[index]} 完了判定 #{completed}、"
                                      f"横移動 {transfer.outward_m*1000:.1f} mm、下降 {transfer.release_drop_m*1000:.1f} mm",
                                      flush=True)
                                if cycle:
                                    index = 1 - index
                                    model = models[index]
                                    move = InitialPoseMove(initial_poses[index])
                                    next_infer = now + .30  # 箱を離した姿勢で保持してから次の初期姿勢へ戻る。
                                    notice = f"{directions[index]}の初期姿勢へ移動"
                                else:
                                    active = False
                                    next_infer = 0.
                                    notice = "デモ完了・姿勢保持"
                            else:
                                active = False
                                next_infer = 0.
                                notice = f"{directions[index]}で完了条件に達せず停止（{transfer.stage}）"
                                print(notice, flush=True)
                            transfer = None
                            status = (notice, False)
                            continue
                    if move is None and pending is not None and pending.done():
                        result, infer_ms = pending.result()
                        pending = None
                        if not warmed:
                            warmup_index += 1
                            warmed = warmup_index == len(models)
                            if warmed:
                                scope = "両方向の" if cycle else ""
                                print(f"実画像の{scope}{model.label}推論成功 {result.shape}・開始待ち", flush=True)
                            else:
                                next_infer = 0.
                        if active and fresh and issued_generation == generation:
                            # 姿勢保持から再開する最初のSmolVLA出力は先頭から使う。
                            chunk_stamp = time.monotonic() if model.kind == "smolvla" and chunk is None else issued_stamp
                            chunk = result
                            notice = ""
                    now = time.monotonic()
                    if move is None and fresh and pending is None and now >= next_infer:
                        issued_generation, issued_stamp = generation, now
                        policy_index = index if warmed else warmup_index
                        predictor = models[policy_index]
                        language = {"task": tasks[policy_index]} if model.kind == "smolvla" else {}
                        pending = executor.submit(predictor.predict, {s:resized[s] for s in model.views}, state, **language)
                        next_infer = now + (1/rate if active else .5)
                    if move is None and active and chunk is not None:
                        action_index = int((now-chunk_stamp)*model.hz)
                        if action_index >= len(chunk):
                            robot.command(robot.read().position)
                            chunk = None
                        else:
                            by_name = dict(zip(model.action_names,chunk[action_index],strict=True))
                            robot.command({n:by_name[n] for n in robot.joints})
                    if missing:
                        camera_names = "・".join(view.titles[s] for s in missing)
                        msg = f"{camera_names}の映像再接続中・" + ("姿勢保持" if active else "実機指令OFF")
                    elif move is not None:
                        msg = f"{notice}中" if notice else "基準姿勢へ移動中"
                    elif active:
                        msg = "推論中・実機指令ON" if chunk is not None else notice or "推論計算中・姿勢保持"
                    else:
                        msg = notice or "プレビュー推論中・実機指令OFF"
                    if hands is not None:
                        if transfer is not None:
                            msg += f"・{transfer.stage}"
                        msg = f"{directions[index]}　完了 {completed}　{msg}"
                    if infer_ms is not None:
                        msg += f"　{cfg['device']} {infer_ms:.1f} ms"
                    status = (msg, fresh and warmed and not active)
                    deadline = max(deadline+1/model.hz, time.monotonic())
                    quit_event.wait(max(0.,deadline-time.monotonic()))
            finally:
                if active and enabled:
                    robot.command(robot.read().position)

    def worker(streams):
        nonlocal status
        try:
            control(streams)
        except Exception as exc:
            status = (f"停止　{exc}",False)
            traceback.print_exc()

    with CameraStreams(devices) as streams:
        thread = threading.Thread(target=worker,args=(streams,),name="act-control")
        thread.start()
        ready_reported = False
        def refresh():
            nonlocal ready_reported
            if connected.is_set():
                view.display(display_images)
            else:
                streams.reconnect_stale()
                frames = streams.preview()
                view.display({side: cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                              for side, frame in frames.items()})
            rates = streams.frame_rates()
            for s,title in view.titles.items():
                hz = rates.get(s)
                view.panels[s].master.configure(text=f"{title}　{hz:.1f} fps" if hz else f"{title}　LOST")
            label, ready = status
            view.status.set(label)
            start_button.configure(state="normal" if ready else "disabled")
            if ready and not ready_reported and not start_button.instate(["disabled"]):
                print("画面の「推論開始」ボタンが有効", flush=True)
                ready_reported = True
            view.root.after(50,refresh)
        try:
            view.root.after(1,refresh)
            view.root.mainloop()
        finally:
            quit_event.set()
            thread.join()
