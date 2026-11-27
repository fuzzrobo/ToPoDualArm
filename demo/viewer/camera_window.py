"""収録・推論で共用する左手、正面、右手の表示。"""
import os
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk


CAMERA_TITLES = {"left": "左手", "front": "正面", "right": "右手"}


class CameraWindow:
    def __init__(self, title, sides):
        for key, name in (("TCL_LIBRARY", "tcl"), ("TK_LIBRARY", "tk")):
            library = Path(sys.base_prefix)/"lib"/f"{name}{tk.TclVersion}"
            if library.is_dir():
                os.environ.setdefault(key, str(library))
        self.root = tk.Tk()
        self.root.title("ToPoDualArm | " + title)
        self.root.geometry("+4+48")
        self.titles = {side: text for side, text in CAMERA_TITLES.items() if side in sides}
        self.panels = {}
        self.placeholder = tk.PhotoImage(width=320, height=240)
        for col, (side, text) in enumerate(self.titles.items()):
            box = ttk.LabelFrame(self.root, text=f"{text}　計測中")
            box.grid(row=0, column=col)
            panel = ttk.Label(box, image=self.placeholder)
            panel.pack()
            self.panels[side] = panel
        self.controls = ttk.Frame(self.root)
        self.controls.grid(row=1, column=0, columnspan=len(self.titles), sticky="ew")
        self.status = tk.StringVar(value="準備中")
        ttk.Label(self.root, textvariable=self.status).grid(row=2, column=0, columnspan=len(self.titles))

    def display(self, images, *, keep_missing=False):
        for side, panel in self.panels.items():
            if side not in images:
                if not keep_missing and getattr(panel, "image", None) is not None:
                    panel.configure(image=self.placeholder)
                    panel.image = None
                continue
            frame = Image.fromarray(images[side])
            if getattr(panel, "image", None) is None:
                panel.image = ImageTk.PhotoImage(frame)
                panel.configure(image=panel.image)
            else:
                panel.image.paste(frame)
