"""MacでMuJoCoの3視点を表示し、Questで操作・収録する。"""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from datetime import datetime
import json
import os
from pathlib import Path
from queue import SimpleQueue
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk

import draccus
from PIL import Image, ImageTk
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples"))
from episode import DEFAULT_ROBOT, YubiTeleoperatorConfig, inputs
from topoarm_recording.episodes import Episodes
from topoarm_recording.simulation import SimTarget
from topoarm_sdk import Device, load_config
from scene_window import SceneWindow
from sim_frames import Frame, SimFrames

DISPLAY_FPS = 20
POSITION_FIRST_HZ = 20


def run(config, destination, hardware=None, cameras=None, read_profile=None, *, position_first=False):
    scene = Path(__file__).with_name("stack_boxes.yaml")
    sim_profile = yaml.safe_load(DEFAULT_ROBOT.read_text())
    if hardware:
        sim = Device(hardware, read_profile=read_profile)
        quest_settings = yaml.safe_load((DEFAULT_ROBOT.parent / sim_profile["home"]).read_text())
    else:
        sim = SimTarget(sim_profile, DEFAULT_ROBOT.parent, scene=scene, cameras=())
        quest_settings = sim.home
    direct_quest_pose = quest_settings["pose"]
    cfg = draccus.decode(YubiTeleoperatorConfig, json.loads(config.read_text()))
    cfg.urdf_path = load_config(str(sim.hardware)).options.robot_model
    for key, name in (("TCL_LIBRARY", "tcl"), ("TK_LIBRARY", "tk")):
        library = Path(sys.base_prefix)/"lib"/f"{name}{tk.TclVersion}"
        if library.is_dir():
            os.environ.setdefault(key,str(library))
    window = tk.Tk()
    window.title("ToPoDualArm | " + ("実機" if hardware else "MuJoCo") + "・Quest・LeRobot収録")
    screen = window.winfo_screenwidth()
    window.geometry("+4+48")
    panels = {}
    titles = {"left":"左手", "front":"正面", "right":"右手"}
    for col, (side, title) in enumerate(titles.items()):
        panel = ttk.LabelFrame(window, text=f"{title}　計測中")
        panel.grid(row=0, column=col)
        panels[side] = ttk.Label(panel)
        panels[side].pack()
    controls = ttk.Frame(window)
    controls.grid(row=1,column=0,columnspan=3,sticky="ew")
    task = tk.StringVar(value=yaml.safe_load(scene.read_text())["task"])
    ttk.Entry(controls,textvariable=task,width=44).pack(side="left")
    status = tk.StringVar(value="準備中")
    ttk.Label(window,textvariable=status).grid(row=2,column=0,columnspan=3)
    fps = 100 if hardware else 20
    events = SimpleQueue()
    # Tkはメインスレッド専用。実機ループと表示の間では直近の画像・文字列だけ共有する。
    latest_images = None
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
        if hardware:
            from topoarm_recording.mac_cameras import CameraStreams
            devices = yaml.safe_load(cameras.read_text())
            if set(devices) != {"front", "left", "right"}:
                raise ValueError("front/left/rightの実カメラ設定が必要です")
            streams = cleanup.enter_context(CameraStreams(devices))
        else:
            cleanup.enter_context(sim)
        placeholder = tk.PhotoImage(width=320, height=240)
        for panel in panels.values():
            panel.configure(image=placeholder)
        window.update_idletasks()
        home = {}
        if not hardware:
            scene_x = window.winfo_rootx() + window.winfo_reqwidth() + 12
            scene_window = SceneWindow(sim.sim.model, scene_x, max(240, screen-scene_x-14), min(580,window.winfo_screenheight()-110))
            cleanup.callback(scene_window.close)
            sim.enable()
            home = sim.read().position
            sim.sim.reset_episode(home)
        episodes = Episodes(destination,sim.joints, fps=fps, hardware_timing=bool(hardware), bam=bool(hardware and sim.cfg.options.read_profile == "bam"))
        cleanup.callback(episodes.close)
        if not hardware:
            sim_frames = SimFrames(sim.sim, episodes)
            cleanup.callback(sim_frames.close)
            episodes = sim_frames
            display_timing = cleanup.enter_context((destination/"display-timing.csv").open("w", buffering=1))
            display_timing.write("display_time,sample_time\n")
        conn = ExitStack()
        cleanup.callback(conn.close)
        pending = None
        if not hardware:
            connector = cleanup.enter_context(ThreadPoolExecutor(max_workers=1))
        def connect_quest():
            return inputs("meta",cfg,conn,home,list(sim.joints),sim.hardware,None,
                          real_robot=bool(hardware),button_event=lambda t,k: events.put((t,k)),
                          position_first=position_first,direct_quest_pose=direct_quest_pose,
                          simulation=None if hardware else sim)
        read = None
        retry = barrier = start = 0.
        next_ik = 0.
        held = home.copy()
        for key,title in (("A","A 開始／採用"),("B","B 棄却")):
            ttk.Button(controls,text=title,command=lambda k=key: events.put((time.monotonic(),k))).pack(side="left")

        camera_paused = False
        connected = not hardware
        next_probe = 0.
        last_report = 0.
        timing = cleanup.enter_context((destination/"control-timing.csv").open("w", buffering=1))
        timing.write("command_time,camera_gui_ms,prepare_ms,read_ms,quest_ms,write_ms,input_ok,left_active,right_active,left_delta_rad,right_delta_rad,recording,left_ik_hold,right_ik_hold,quest_updated\n")
        previous_action = None
        cached_stamps = {}
        resized = {}
        def display(images):
            for side, pixels in images.items():
                frame = Image.fromarray(pixels)
                panel = panels[side]
                if getattr(panel, "image", None) is None:
                    panel.image = ImageTk.PhotoImage(frame)
                    panel.configure(image=panel.image)
                else:
                    panel.image.paste(frame)

        def tick():
            nonlocal camera_paused, read, retry, barrier, start, held, connected, next_probe, last_report, home
            nonlocal previous_action, cached_stamps, resized, latest_images, pending, next_ik
            now = time.monotonic()
            try:
                if hardware:
                    import cv2
                    streams.reconnect_stale()
                    snapshot = streams.latest_synced()
                    fresh = snapshot is not None
                    if fresh:
                        raw, stamps = snapshot
                        # 各カメラの新着分だけ縮小し、制御・収録間で同じ写真を参照する。
                        resized = {side: resized[side] if cached_stamps.get(side) == stamps[side]
                                   else cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)
                                   for side,frame in raw.items()}
                        cached_stamps = stamps
                        images = resized
                        latest_images = images
                    else:
                        latest_images = {side: cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)
                                         for side,frame in streams.preview().items()}
                    if now-last_report > 5:
                        print("実カメラHz",streams.frame_rates(),flush=True)
                        last_report = now
                    if not fresh or not connected:
                        while not events.empty():
                            events.get()
                        if read and not camera_paused:
                            held = sim.command(sim.read().position)
                            read.pause(held)
                            camera_paused = True
                        if episodes.task is not None:
                            episodes.button("B",task_text)
                        if not fresh:
                            set_status("実カメラ同期待ち")
                        elif not cfg.hardware_verified:
                            set_status("実カメラ表示中・Questの実機座標設定が未確認")
                        elif now >= next_probe:
                            try:
                                cleanup.enter_context(sim)
                            except RuntimeError as exc:
                                print(exc, flush=True)
                                set_status(f"フォロワー待機: {exc}")
                                next_probe = now+3
                            else:
                                sim.enable()
                                home = held = sim.read().position
                                connected = True
                                print("実機18軸 接続・駆動開始",flush=True)
                        return .05
                camera_done = time.monotonic()
                if camera_paused:
                    while not events.empty():
                        events.get()
                    camera_paused = False
                while not events.empty():
                    stamp,key = events.get()
                    if stamp < barrier:
                        continue
                    was_idle = episodes.task is None
                    if episodes.button(key,task_text):
                        if not hardware:
                            sim.sim.reset_episode(home)
                        held = sim.read().position if hardware else home.copy()
                        if pending is None:
                            conn.close()
                        read = None
                        barrier = time.monotonic()
                    elif was_idle and episodes.task:
                        start = now
                if read is None and now >= retry:
                    try:
                        if hardware:
                            read = connect_quest()
                        elif pending is None:
                            pending = connector.submit(connect_quest)
                        elif pending.done():
                            attempt, pending = pending, None
                            read = attempt.result()
                    except (ValueError, RuntimeError, OSError) as exc:
                        print(f"Quest入力待機: {exc}",flush=True)
                        conn.close()
                        retry = time.monotonic()+1
                read_start = time.monotonic()
                state = sim.read().position
                read_done = time.monotonic()
                quest_updated = False
                if read and read_start >= next_ik:
                    if hardware and position_first:
                        # 1回の関節変化量はシミュレーションで検証した20 Hzを前提とする。
                        next_ik = read_start + 1/POSITION_FIRST_HZ
                    try:
                        held = read(0,state)
                        quest_updated = True
                    except (ValueError, RuntimeError, OSError) as exc:
                        print(f"Quest入力待機: {exc}",flush=True)
                        conn.close()
                        read = None
                        retry = now+1
                        if hardware:
                            held = state.copy()
                solve_done = time.monotonic()
                if not hardware:
                    # 画像と実測角は同じ指令適用前の姿勢。箱の自由関節も含む。
                    qpos = sim.sim.data.qpos.copy()
                    sim_time = sim.sim.data.time
                applied = sim.command(held)
                sent = time.monotonic()
                deltas = [max((abs(applied[n]-previous_action[n]) for n in applied
                               if n.startswith(prefix)), default=0.) if previous_action else 0.
                          for prefix in ("L_joint","R_joint")]
                stamps = (now,camera_done,read_start,read_done,solve_done,sent)
                intervals = [(b-a)*1000 for a,b in zip(stamps,stamps[1:])]
                active = [int(bool(read and read.step.active[side])) for side in ("left","right")]
                holds = [int(bool(read and read.step.ik_hold[side])) for side in ("left","right")]
                timing.write(",".join(map(str,[sent,*intervals,int(read is not None),*active,*deltas,
                                              int(episodes.task is not None),*holds,int(quest_updated)]))+"\n")
                previous_action = applied
                if read:
                    read.accepted(applied)
                if hardware and position_first:
                    held = applied.copy()
                if hardware and now-last_report < .001 and read:
                    print("Quest入力・SDK指令受理",flush=True)
                if hardware:
                    host_times = (read_done, sent, *(cached_stamps[s] for s in ("left","front","right")))
                    episodes.frame(images,state,applied,sim.last_states,host_times)
                else:
                    sim_frames.publish(Frame(sent, qpos, sim_time, state, applied))
                mode = f"収録中 {now-start:.1f}秒" if episodes.task else "待機中"
                set_status(f"{mode}　採用 {episodes.accepted} 件　Quest " + ("接続" if read else "再接続中"))
            except Exception as exc:
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

        display_times = deque()
        def update_headers(now, fresh_display=False):
            if fresh_display:
                display_times.append(now)
            while display_times and now - display_times[0] > 1.:
                display_times.popleft()
            if hardware:
                rates = streams.frame_rates()
            else:
                count = len(display_times)
                hz = (count-1)/(display_times[-1]-display_times[0]) if count > 1 else 0.
                rates = dict.fromkeys(titles, hz) if count and now-display_times[-1] < .25 else {}
            reader = read
            tracking = reader.tracking if reader is not None else {}
            live = 0 <= now - tracking.get("received", -1e9) <= cfg.input_timeout_s
            for side,title in titles.items():
                rate = rates.get(side)
                source = "取得" if hardware else "表示"
                label = f"{title}　{source} {rate:.1f} fps" if rate is not None else f"{title}　0 fps　カメラ LOST"
                if not live or (side != "front" and not tracking.get(side)):
                    label += "　Quest LOST"
                elif side != "front" and reader.step.ik_hold[side]:
                    label += "　IK保持"
                box = panels[side].master
                if box.cget("text") != label:
                    box.configure(text=label)
        last_displayed = None
        refresh_deadline = time.monotonic()
        def refresh():
            nonlocal refresh_deadline, last_displayed
            sample = None if hardware else sim_frames.latest
            if not hardware and sample is last_displayed:
                update_headers(time.monotonic())
                status.set(latest_status)
                window.after(2, refresh)
                return
            images = latest_images
            if sample is not None:
                sim_frames.pose(sample)
                if sample.images is None:
                    sample.images = sim_frames.view.images()
                images = sample.images
            if images is not None:
                for side in panels.keys() - images.keys():
                    panels[side].configure(image=placeholder)
                    panels[side].image = None
                display(images)
            if sample is not None:
                scene_window.draw(sim_frames.view.model, sim_frames.view.data)
                update_headers(time.monotonic(), True)
                last_displayed = sample
                display_timing.write(f"{time.monotonic()},{sample.stamp}\n")
                # 新着画像を再利用できる周期は追加描画なし。保存用の過去姿勢だけ再描画する。
                sim_frames.record_pending(max(sample.stamp + sim.dt, time.monotonic()+.002))
                status.set(latest_status)
                # 同じ姿勢を描き直さず、次の20 Hzサンプルが届くころに確認する。
                wait = sample.stamp + sim.dt - time.monotonic() - .003
                window.after(max(1,round(wait*1000)), refresh)
                return
            status.set(latest_status)
            update_headers(time.monotonic())
            refresh_deadline = max(refresh_deadline + 1/DISPLAY_FPS, time.monotonic())
            window.after(max(1, round((refresh_deadline-time.monotonic())*1000)), refresh)

        worker = threading.Thread(target=control_loop, name="teleop-control")
        worker.start()
        try:
            window.after(1,refresh)
            window.mainloop()
        finally:
            stop.set()
            worker.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config",type=Path,default=ROOT/"config/yubi-local.json")
    parser.add_argument("--root",type=Path,default=ROOT/"data"/datetime.now().strftime("quest-gui-%Y%m%d-%H%M%S"))
    parser.add_argument("--hardware",type=Path)
    parser.add_argument("--cameras",type=Path)
    parser.add_argument("--read-profile", choices=("teleop", "bam"), help="実機SDKの読取プロファイル")
    parser.add_argument("--position-first", action="store_true", help="実機で位置優先IKを20 Hzで使う")
    args = parser.parse_args()
    if args.position_first and not args.hardware:
        parser.error("--position-first は実機専用です。MuJoCoは常に実測補正付きMinkを使います")
    if args.read_profile and not args.hardware:
        parser.error("--read-profileには--hardwareが必要です")
    if args.hardware and not args.cameras:
        parser.error("--hardwareには--camerasが必要です")
    run(args.config,args.root,args.hardware,args.cameras,args.read_profile,
        position_first=args.position_first)
