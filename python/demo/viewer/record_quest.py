"""Questで実機を操作し、カメラ映像と関節角を収録する。"""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime
import json
from pathlib import Path
from queue import SimpleQueue
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk

import draccus
import yaml

from lerobot_teleoperator_yubi import YubiTeleoperatorConfig
from topoarm_recording.episodes import Episodes
from topoarm_recording.inputs import inputs
from topoarm_recording import ROOT
from topoarm_sdk import Device
from camera_window import CAMERA_TITLES, CameraWindow

DISPLAY_FPS = 20
POSITION_FIRST_HZ = 50


def run(config, destination, hardware, cameras, read_profile=None, *, position_first=True,
        ik_hz=POSITION_FIRST_HZ, task_name=None):
    ik_period = 1 / ik_hz
    devices = yaml.safe_load(cameras.read_text())
    if not isinstance(devices, dict) or set(devices) not in ({"left", "right"}, set(CAMERA_TITLES)):
        raise ValueError("実カメラ設定にはleft/rightが必要です。frontは省略できます")
    camera_sides = tuple(side for side in CAMERA_TITLES if side in devices)
    robot = Device(hardware, read_profile=read_profile)
    quest_settings = yaml.safe_load((ROOT / "config/teleop/quest.yaml").read_text())
    direct_quest_pose = quest_settings["pose"]
    cfg = draccus.decode(YubiTeleoperatorConfig, json.loads(config.read_text()))
    cfg.urdf_path = robot.cfg.options.robot_model
    view = CameraWindow("実機・Quest・LeRobot収録", camera_sides)
    window, controls, status = view.root, view.controls, view.status
    panels, titles = view.panels, view.titles
    default_task = "Pack the black box into the box"
    task = tk.StringVar(value=task_name or default_task)
    ttk.Entry(controls,textvariable=task,width=44).pack(side="left")
    fps = 100
    events = SimpleQueue()
    # Tkはメインスレッド専用。
    latest_status = ""
    task_text = task.get()
    def update_task(*_):
        nonlocal task_text
        task_text = task.get()
    task.trace_add("write", update_task)
    def set_status(message):
        nonlocal latest_status
        latest_status = message


    with ExitStack() as cleanup:
        import cv2
        from topoarm_recording.mac_cameras import CameraStreams
        print("実機IK: " + (f"位置優先・{ik_hz} Hz設定" if position_first else "位置・姿勢"), flush=True)
        streams = cleanup.enter_context(CameraStreams(devices))
        follower = cleanup.enter_context(ExitStack())
        window.update_idletasks()
        episodes = Episodes(destination, robot.joints, fps=fps, hardware_timing=True,
                            bam=robot.cfg.options.read_profile == "bam",
                            camera_sides=camera_sides)
        cleanup.callback(episodes.close)
        conn = ExitStack()
        cleanup.callback(conn.close)
        def connect_quest():
            return inputs("meta",cfg,conn,None,list(robot.joints),robot.hardware,None,
                          button_event=lambda _, key: events.put(key),
                          position_first=position_first,direct_quest_pose=direct_quest_pose,
                          measure_ik_error=True,
                          ik_period=ik_period if position_first else None)
        read = None
        retry = start = 0.
        next_ik = 0.
        held = {}
        for key,title in (("A","A 開始／採用"),("B","B 棄却")):
            ttk.Button(controls,text=title,command=lambda k=key: events.put(k)).pack(side="left")

        camera_paused = False
        connected = False
        next_probe = 0.
        read_gap_since = None
        last_report = 0.
        sync_misses = {}
        camera_errors = {}
        timing = cleanup.enter_context((destination/"control-timing.csv").open("w", buffering=1))
        timing.write("command_time,camera_gui_ms,prepare_ms,read_ms,quest_ms,write_ms,input_ok,"
                     "left_active,right_active,left_delta_rad,right_delta_rad,recording,left_ik_hold,right_ik_hold,"
                     "quest_updated,left_position_error_mm,left_rotation_error_deg,right_position_error_mm,"
                     "right_rotation_error_deg,left_ik_stalled,right_ik_stalled\n")
        pose_log = csv.writer(cleanup.enter_context((destination / "ik-pose.csv").open("w", buffering=1, newline="")))
        pose_log.writerow(("quest_time_s", "state_time_s", "command_time_s", "side",
                           "target_x_m", "target_y_m", "target_z_m", "actual_x_m", "actual_y_m", "actual_z_m"))
        previous_action = None
        last_result = ""
        cached_stamps = {}
        resized = {}
        def handle_buttons():
            nonlocal start, last_result
            while not events.empty():
                key = events.get()
                idle = episodes.task is None
                frames = episodes.frames
                if not idle and key == "A" and frames:
                    set_status(f"{frames}フレームを保存中")
                if episodes.button(key,task_text):
                    last_result = (f"保存済み {episodes.accepted} 件・直近 {frames} フレーム"
                                   if key == "A" else f"棄却 {frames} フレーム")
                    print(f"収録{key}: {last_result} ({destination})", flush=True)
                    set_status(last_result)
                    # 保存中に届いた次の操作は、新しいカメラ時刻で処理する。
                    return True
                elif idle and episodes.task:
                    start = time.monotonic()
                    print("収録A: 開始", flush=True)
            return False

        def tick():
            nonlocal camera_paused, read, retry, held, connected, next_probe, last_report
            nonlocal previous_action, cached_stamps, resized, next_ik
            nonlocal read_gap_since
            now = time.monotonic()
            try:
                streams.reconnect_stale()
                raw, stamps, valid = streams.latest_per_camera()
                fresh = all(valid.values())
                for side, frame in raw.items():
                    if cached_stamps.get(side) != stamps[side]:
                        resized[side] = cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)
                        cached_stamps[side] = stamps[side]
                for side, is_valid in valid.items():
                    if not is_valid:
                        sync_misses[side] = sync_misses.get(side, 0) + 1
                if now-last_report > 5:
                    print("実カメラHz",streams.frame_rates(),flush=True)
                    if sync_misses:
                        print("カメラ同期条件外の制御周期数", sync_misses, flush=True)
                        sync_misses.clear()
                    for side, diag in streams.diagnostics().items():
                        error = diag.get("last_error")
                        if error and camera_errors.get(side) != (diag["opens"], error):
                            print(f"{side} カメラ状態 {diag['state']}、オープン {diag['opens']} 回"
                                  f"、直近のエラー {error}", flush=True)
                            camera_errors[side] = (diag["opens"], error)
                    last_report = now
                if not connected:
                    handle_buttons()
                    if not fresh:
                        set_status("実カメラ同期待ち")
                    elif not cfg.hardware_verified:
                        set_status("実カメラ表示中・Questの実機座標設定が未確認")
                    elif now >= next_probe:
                        try:
                            follower.enter_context(robot)
                        except RuntimeError as exc:
                            print(exc, flush=True)
                            set_status(f"フォロワー待機: {exc}")
                            next_probe = now+3
                        else:
                            robot.enable()
                            held = robot.read().position
                            connected = True
                            print("実機18軸 接続・駆動開始",flush=True)
                    return .05
                if not fresh and not camera_paused:
                    held = robot.command(robot.read().position)
                    if read:
                        read.pause(held)
                camera_paused = not fresh
                camera_done = time.monotonic()
                if handle_buttons():
                    return 1 / fps
                if read is None and now >= retry and not camera_paused:
                    try:
                        read = connect_quest()
                    except (ValueError, RuntimeError, OSError) as exc:
                        print(f"Quest入力待機: {exc}",flush=True)
                        conn.close()
                        retry = time.monotonic()+1
                read_start = time.monotonic()
                state = robot.read().position
                read_done = time.monotonic()
                recovered = read_gap_since is not None
                if recovered:
                    held = state.copy()
                    if read:
                        read.pause(held)
                    next_ik = 0.
                    read_gap_since = None
                    print("関節角の再受信・現在位置から再開", flush=True)
                quest_updated = False
                if not recovered and read and read_start >= next_ik and not camera_paused:
                    if position_first:
                        if not next_ik:
                            next_ik = read_start
                        # バス周期の揺れで位相をずらさず、間に合わなかった周期はスキップする。
                        next_ik += (int((read_start - next_ik) / ik_period) + 1) * ik_period
                    try:
                        held = read(0,state)
                        quest_updated = True
                    except (ValueError, RuntimeError, OSError) as exc:
                        print(f"Quest入力待機: {exc}",flush=True)
                        conn.close()
                        read = None
                        retry = now+1
                        held = state.copy()
                solve_done = time.monotonic()
                applied = robot.command(held)
                sent = time.monotonic()
                deltas = [max((abs(applied[n]-previous_action[n]) for n in applied
                               if n.startswith(prefix)), default=0.) if previous_action else 0.
                          for prefix in ("L_joint","R_joint")]
                stamps = (now,camera_done,read_start,read_done,solve_done,sent)
                intervals = [(b-a)*1000 for a,b in zip(stamps,stamps[1:])]
                active = [int(bool(read and read.step.active[side])) for side in ("left","right")]
                holds = [int(bool(read and read.step.ik_hold[side])) for side in ("left","right")]
                errors = read.step.ik_error if quest_updated else {}
                left_error = errors.get("left", ("", ""))
                right_error = errors.get("right", ("", ""))
                stalled = [int(read.step.stalls[side] > 0) if side in errors else "" for side in ("left", "right")]
                timing.write(",".join(map(str,[sent,*intervals,int(read is not None),*active,*deltas,
                                              int(episodes.task is not None),*holds,int(quest_updated),
                                              *left_error,*right_error,*stalled]))+"\n")
                if errors:
                    for side, positions in read.step.ik_positions.items():
                        pose_log.writerow((read.tracking["received"], read_done, sent, side, *positions))
                previous_action = applied
                if read:
                    read.accepted(applied)
                if position_first:
                    held = applied.copy()
                if now-last_report < .001 and read:
                    print("Quest入力・SDK指令受理",flush=True)
                host_times = (read_done, sent, *(cached_stamps[s] for s in camera_sides))
                episodes.frame(resized,state,applied,robot.last_states,host_times,[valid[s] for s in camera_sides])
                mode = f"収録中 {max(0., now-start):.1f}秒" if episodes.task else (last_result or "待機中")
                count = "" if last_result and not episodes.task else f"　採用 {episodes.accepted} 件"
                if camera_paused:
                    mode += "　腕保持・カメラ同期待ち"
                set_status(f"{mode}{count}　Quest " + ("接続" if read else "再接続中"))
            except Exception as exc:
                if isinstance(exc, RuntimeError) and str(exc).endswith(
                    ("読出し周期が欠落しました", "IDまたは項目が欠落しました")
                ):
                    if read_gap_since is None:
                        read_gap_since = time.monotonic()
                        print(f"実機読取を再試行: {exc}", flush=True)
                    set_status("関節角の再受信中")
                    if not connected or time.monotonic() - read_gap_since >= 1.:
                        # 収録バッファとカメラは開いたまま、フォロワーのポートだけを開き直す。
                        connected = False
                        try:
                            follower.close()
                        except RuntimeError as close_error:
                            print(f"フォロワー切断時: {close_error}", flush=True)
                        next_probe = time.monotonic() + 1.
                        previous_action = None
                        camera_paused = False
                        set_status("フォロワー再接続中")
                        print("フォロワー再接続中", flush=True)
                    return .02
                set_status(f"停止: {exc}")
                import traceback
                traceback.print_exc()
                return None
            return 1 / fps

        stop = threading.Event()
        def control_loop():
            deadline = time.monotonic()
            while not stop.is_set():
                period = tick()
                if period is None:
                    return
                deadline += period
                now = time.monotonic()
                if deadline < now - period:
                    deadline = now
                stop.wait(max(0., deadline - now))

        displayed_stamps = {}

        def update_headers(now):
            rates = streams.frame_rates()
            reader = read
            tracking = reader.tracking if reader is not None else {}
            live = 0 <= now - tracking.get("received", -1e9) <= cfg.input_timeout_s
            for side, title in titles.items():
                box = panels[side].master
                rate = rates.get(side)
                label = f"{title}　取得 {rate:.1f} fps" if rate is not None else f"{title}　0 fps　カメラ LOST"
                if rate is None and side in displayed_stamps:
                    label += "・最終画像"
                if not live or (side != "front" and not tracking.get(side)):
                    label += "　Quest LOST"
                elif side != "front" and reader.step.ik_hold[side]:
                    label += "　IK保持"
                if box.cget("text") != label:
                    box.configure(text=label)

        refresh_deadline = time.monotonic()

        def refresh():
            nonlocal refresh_deadline
            # 表示はカメラごとの鮮度で更新する。同期やQuest再接続の待ち時間はここに持ち込まない。
            images, stamps, valid = streams.latest_per_camera(max_skew=float("inf"))
            updates = {}
            for side, available in valid.items():
                if available and displayed_stamps.get(side) != stamps[side]:
                    updates[side] = cv2.resize(images[side], (320, 240), interpolation=cv2.INTER_AREA)
                    displayed_stamps[side] = stamps[side]
            view.display(updates, keep_missing=True)
            status.set(latest_status)
            update_headers(time.monotonic())
            refresh_deadline = max(refresh_deadline + 1 / DISPLAY_FPS, time.monotonic())
            window.after(max(1, round((refresh_deadline - time.monotonic()) * 1000)), refresh)

        worker = threading.Thread(target=control_loop, name="teleop-control")
        worker.start()
        try:
            window.after(1,refresh)
            window.mainloop()
        finally:
            stop.set()
            worker.join()


def recording_config(path):
    path = path.resolve()
    cfg = yaml.safe_load(path.read_text())
    required = {"quest", "root", "task", "robot", "cameras"}
    if not isinstance(cfg, dict) or cfg.keys() != required:
        raise ValueError("収録YAMLにはquest・root・task・robot・camerasを指定します")
    if any(not isinstance(value, str) or not value.strip() for value in cfg.values()):
        raise ValueError("収録YAMLの各項目には空でない文字列を指定してください")
    args = {key: (path.parent / cfg[key]).resolve() for key in ("quest", "root", "robot", "cameras")}
    if args["root"].exists():
        raise ValueError(f"未使用の収録先を指定してください: {args['root']}")
    for key in ("quest", "robot", "cameras"):
        if not args[key].is_file():
            raise ValueError(f"{key}の設定ファイルがありません: {args[key]}")
    return args["quest"], args["root"], args["robot"], args["cameras"], cfg["task"].strip()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config",type=Path)
    parser.add_argument("--root",type=Path)
    parser.add_argument("--hardware",type=Path)
    parser.add_argument("--cameras",type=Path)
    parser.add_argument("--read-profile", choices=("teleop", "bam"), help="実機SDKの読取プロファイル")
    parser.add_argument("--position-first", action=argparse.BooleanOptionalAction,
                        help="実機で位置優先IKを使う（既定で有効）")
    parser.add_argument("--ik-hz", type=int, choices=(20, POSITION_FIRST_HZ),
                        help=f"実機の位置優先IKの更新周波数（既定 {POSITION_FIRST_HZ} Hz）")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--record", type=Path, help="収録用YAML")
    modes.add_argument("--inference", type=Path, help="ACTまたはDiffusion Policyの推論用YAML")
    args = parser.parse_args()
    if args.record or args.inference:
        if args.hardware or args.cameras or args.root or args.position_first is not None or args.ik_hz or args.read_profile or args.config:
            parser.error("--recordまたは--inferenceでは設定をYAMLにまとめて指定してください")
        if args.record:
            try:
                config, root, hardware, cameras, task_name = recording_config(args.record)
            except (ValueError, OSError, yaml.YAMLError) as exc:
                parser.error(str(exc))
            run(config, root, hardware, cameras, task_name=task_name)
        else:
            from act_viewer import run_inference
            run_inference(args.inference)
        sys.exit(0)
    args.config = args.config or ROOT/"config/yubi-local.json"
    args.root = args.root or ROOT/"data"/datetime.now().strftime("quest-gui-%Y%m%d-%H%M%S")
    if args.ik_hz is not None and args.position_first is False:
        parser.error("--ik-hz には実機の位置優先IKが必要です")
    if not args.hardware or not args.cameras:
        parser.error("実機収録には--hardwareと--camerasが必要です")
    run(args.config,args.root,args.hardware,args.cameras,args.read_profile,
        position_first=args.position_first is not False, ik_hz=args.ik_hz or POSITION_FIRST_HZ)
