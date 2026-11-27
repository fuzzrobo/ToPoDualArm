"""実機ビューアでACT、Diffusion Policy、SmolVLAを実行する。"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import json
import math
from queue import Empty, SimpleQueue
import threading
import traceback
import time
from tkinter import BooleanVar, ttk

import cv2
import numpy as np
import torch
import yaml
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from topoarm_recording.config import inference_config
from topoarm_recording.mac_cameras import CameraStreams
from topoarm_sdk import Device, VsidoDevice

from camera_window import CAMERA_TITLES, CameraWindow

WRISTS = ("left", "right")
MODEL_BLEND_S = .2


def action_at(chunks, now, hz, *, smooth):
    while chunks and (now - chunks[0][0]) * hz >= len(chunks[0][1]):
        chunks.popleft()
    if not chunks:
        return None
    if not smooth or len(chunks) == 1:
        stamp, chunk = chunks[-1]
        return chunk[int((now - stamp) * hz)]
    # チャンクごとの観測時刻から同じ指令時刻の行を選び、新しい予測を重くする。
    weights = tuple(.5**age for age in range(len(chunks) - 1, -1, -1))
    return sum(weight * chunk[int((now - stamp) * hz)]
               for weight, (stamp, chunk) in zip(weights, chunks, strict=True)) / sum(weights)


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
            raise ValueError(
                f"モデル（状態{len(self.state_names)}軸／指令{len(self.action_names)}軸）と"
                f"機体設定（{len(joints)}軸）の関節一覧が一致しません: {path}"
            )
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
    cfg, folder = inference_config(config_path)
    if cfg["backend"] == "vsido":
        robot = VsidoDevice(**cfg["robot"])
        connection_name = "V-Sido"
    else:
        robot = Device(**cfg["robot"], role="follower", read_profile="teleop")
        connection_name = "U2D2"
    if robot.read_only:
        raise ValueError("フォロワーを指定してください")
    configured_cameras = cfg["cameras"]
    choices = cfg.get("models")
    if choices is not None:
        if (not isinstance(choices, dict) or not choices
                or any(not isinstance(name, str) or not name.strip() or not isinstance(profile, dict)
                       for name, profile in choices.items())):
            raise ValueError("modelsには表示名とモデル設定を指定してください")
    profiles = choices or {None: cfg}
    models, initial_poses, tasks = [], [], []
    for name, profile in profiles.items():
        initial_pose = configured_initial_pose(profile, robot, folder=folder)
        checkpoint = (folder / profile["checkpoint"]).resolve()
        backbone = (folder / profile["backbone"]).resolve() if profile.get("backbone") else None
        policy = BoxpackPolicy(checkpoint, cfg["device"], robot.joints, backbone)
        if choices and (policy.kind != "act" or (models and
                        (policy.hz, policy.chunk_size, policy.views) !=
                        (models[0].hz, models[0].chunk_size, models[0].views))):
            raise ValueError("モデル選択には同じ視点・指令周期・チャンク長のACTを指定してください")
        models.append(policy)
        initial_poses.append(initial_pose)
        tasks.append(profile.get("task"))
        model_label = name or f"{checkpoint.parent.name}/{checkpoint.name}"
        print(f"{policy.label}モデル {model_label}　入力視点 {policy.views}", flush=True)
        if initial_pose is not None:
            print(f"初期姿勢を設定（{len(initial_pose)}軸、推論開始時に移動）", flush=True)
    model = models[0]
    if missing := set(model.views) - configured_cameras.keys():
        raise ValueError(f"モデルが必要とするカメラがありません: {', '.join(sorted(missing))}")
    devices = {side: configured_cameras[side] for side in CAMERA_TITLES if side in model.views}
    rate = float(cfg["replan_hz"])
    if not math.isfinite(rate) or not 0 < rate <= model.hz or rate*model.chunk_size < model.hz:
        raise ValueError("再推論の周期は指令周期以上、チャンク長以下にしてください")
    ensemble = cfg.get("act_temporal_ensemble", True)
    if not isinstance(ensemble, bool):
        raise ValueError("act_temporal_ensembleにはtrueまたはfalseを指定してください")
    is_act = model.kind == "act"
    if is_act:
        print(f"ACTチャンク平滑化 {'ON' if ensemble else 'OFF'}", flush=True)

    title = "ACT推論" if choices else f"実機・{model.label}推論 | {model_label}"
    view = CameraWindow(title, devices)
    controls = SimpleQueue()
    quit_event = threading.Event()
    if choices:
        names = tuple(choices)
        ttk.Label(view.controls, text="モデル名").pack(side="left")
        picker = ttk.Combobox(view.controls, values=names, state="readonly", width=max(map(len, names)))
        picker.current(0)
        picker.pack(side="left")
        picker.bind("<<ComboboxSelected>>", lambda _: controls.put(picker.current()))
    else:
        ttk.Label(view.controls, text=cfg["task"]).pack(side="left")
    start_button = ttk.Button(view.controls,text="推論開始",state="disabled",command=lambda:controls.put("start"))
    start_button.pack(side="left")
    ttk.Button(view.controls, text="停止" if choices else "停止・姿勢保持",
               command=lambda: controls.put("stop")).pack(side="left")
    if is_act:
        smooth_check = BooleanVar(master=view.root, value=ensemble)
        ttk.Checkbutton(view.controls, text="チャンク平滑化", variable=smooth_check,
                        command=lambda: controls.put(smooth_check.get())).pack(side="left")
    status = (f"{connection_name}接続中", False)
    display_images = {}
    connected = threading.Event()

    def control(streams):
        nonlocal status, display_images
        index = 0
        selected = index
        model = models[index]
        active = enabled = warmed = False
        warmup_index = 0
        generation = 0
        pending = switching = blend = last_command = None
        smooth = is_act and ensemble
        chunks = deque(maxlen=3 if is_act else 1)
        next_infer = next_switch = 0.
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
                    waiting = (f"{connection_name}接続待ち　{exc}", False)
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
            executor = resources.enter_context(ThreadPoolExecutor(max_workers=2 if choices else 1))
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
                        if active and (move is not None or choices):
                            move = None
                            active = False
                            notice = "関節角の欠落で実機指令を中断・姿勢保持"
                        generation += 1
                        chunks.clear()
                        blend = None
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
                    stop_requested = False
                    while True:
                        try:
                            event = controls.get_nowait()
                        except Empty:
                            break
                        if isinstance(event, bool):
                            smooth = event
                            print(f"ACTチャンク平滑化 {'ON' if smooth else 'OFF'}", flush=True)
                            continue
                        if isinstance(event, int):
                            selected = event
                            next_switch = 0.
                            if not requested and selected != index:
                                index = selected
                                model = models[index]
                                infer_ms = None
                                next_infer = 0.
                                if warmed:
                                    generation += 1
                            continue
                        if event == "start" and (requested or stop_requested):
                            continue
                        stop_requested = event == "stop"
                        requested = event == "start" and fresh and warmed
                        notice = ""
                        generation += 1
                        chunks.clear()
                        blend = None
                        next_infer = 0.
                        if choices:
                            index = selected
                            model = models[index]
                            infer_ms = None
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
                    if active and not fresh and (move is not None or choices):
                        requested = False
                        generation += 1
                        chunks.clear()
                        blend = None
                        next_infer = 0.
                        notice = "映像の欠落で実機指令を中断・姿勢保持"
                    if (active and not requested) or hold_after_gap:
                        last_command = robot.command(state)
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
                        chunks.clear()
                        blend = None
                        next_infer = 0.
                    if fresh or not active:
                        camera_hold = False
                    if move is not None and now >= next_infer:
                        target, outcome = move.update(state, time.monotonic())
                        last_command = robot.command(target)
                        if outcome == "ready":
                            print(f"初期姿勢に到達・{model.label}推論開始", flush=True)
                            move = None
                            notice = ""
                        elif outcome == "timeout":
                            notice = "基準姿勢に届かなかったため停止・姿勢保持"
                            print(notice, flush=True)
                            active = False
                            move = None
                    if move is None and pending is not None and pending.done():
                        prediction = pending
                        pending = None
                        if not warmed:
                            result, infer_ms = prediction.result()
                            if choices:
                                print(f"実画像のACT推論成功 {names[warmup_index]} {result.shape}", flush=True)
                            warmup_index += 1
                            warmed = warmup_index == len(models)
                            if warmed:
                                print(f"実画像の{model.label}推論成功 {result.shape}・開始待ち", flush=True)
                            else:
                                next_infer = 0.
                        elif issued_generation == generation:
                            result, infer_ms = prediction.result()
                            if active and fresh:
                                # 姿勢保持から再開する最初のSmolVLA出力は先頭から使う。
                                stamp = time.monotonic() if model.kind == "smolvla" and not chunks else issued_stamp
                                chunks.append((stamp, result))
                                notice = ""
                    now = time.monotonic()
                    if switching is not None and switching.done():
                        candidate = switching
                        switching = None
                        if active and fresh and switch_index == selected and switch_generation == generation:
                            try:
                                result, duration_ms = candidate.result()
                            except Exception as exc:
                                print(f"ACTモデル切り替え待ち {names[switch_index]}: {exc}", flush=True)
                                next_switch = now + .5
                            else:
                                if (now - switch_stamp) * model.hz < len(result):
                                    index = switch_index
                                    model = models[index]
                                    generation += 1
                                    chunks.clear()
                                    chunks.append((switch_stamp, result))
                                    # SDKの制限を適用して送信した指令から補間する。
                                    blend = (now, last_command if last_command is not None else state)
                                    infer_ms = duration_ms
                                    next_infer = switch_stamp + 1/rate
                                    print(f"ACTモデル切り替え {names[index]}", flush=True)
                    if move is None and fresh:
                        policy_index = index if warmed else warmup_index
                        images = {s:resized[s] for s in model.views}
                        if (pending is None and now >= next_infer
                                and (switching is None or policy_index != switch_index)):
                            issued_generation, issued_stamp, issued_index = generation, now, policy_index
                            predictor = models[policy_index]
                            language = {"task": tasks[policy_index]} if model.kind == "smolvla" else {}
                            pending = executor.submit(predictor.predict, images, state, **language)
                            next_infer = now + (1/rate if active else .5)
                        if (active and selected != index and switching is None and now >= next_switch
                                and (pending is None or selected != issued_index)):
                            switch_generation, switch_stamp, switch_index = generation, now, selected
                            switching = executor.submit(models[selected].predict, images, state)
                    if move is None and active and chunks:
                        action = action_at(chunks, now, model.hz, smooth=smooth)
                        if action is None:
                            last_command = robot.command(robot.read().position)
                            blend = None
                        else:
                            by_name = dict(zip(model.action_names, action, strict=True))
                            if blend is not None:
                                start_time, origin = blend
                                phase = (now - start_time) / MODEL_BLEND_S
                                if phase >= 1.:
                                    blend = None
                                else:
                                    weight = phase*phase * (3. - 2.*phase)
                                    by_name = {n: origin[n] + weight*(by_name[n] - origin[n]) for n in robot.joints}
                            last_command = robot.command({n:by_name[n] for n in robot.joints})
                    if missing:
                        camera_names = "・".join(view.titles[s] for s in missing)
                        msg = f"{camera_names}の映像再接続中・" + ("姿勢保持" if active else "実機指令OFF")
                    elif move is not None:
                        msg = f"{notice}中" if notice else "基準姿勢へ移動中"
                    elif active:
                        msg = "推論中・実機指令ON" if chunks else notice or "推論計算中・姿勢保持"
                        if choices:
                            msg += f"　{names[index]}"
                            if selected != index:
                                msg += f" → {names[selected]} 切り替え準備中"
                            elif blend is not None:
                                msg += "　切り替え中"
                    else:
                        msg = notice or ("停止・姿勢保持" if enabled else "プレビュー推論中・実機指令OFF")
                    if infer_ms is not None:
                        msg += f"　{cfg['device']} {infer_ms:.1f} ms"
                    status = (msg, fresh and warmed and not active)
                    deadline = max(deadline+1/model.hz, time.monotonic())
                    quit_event.wait(max(0.,deadline-time.monotonic()))
            finally:
                if active and enabled:
                    try:
                        robot.command(robot.read().position)
                    except (RuntimeError, TimeoutError) as exc:
                        print(f"終了時の保持指令を送れません: {exc}", flush=True)

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
