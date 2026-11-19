"""フォロワーを生成しない入力確認デモ。--mockでは外部機器へ接続しない。"""
from contextlib import ExitStack
import argparse
import json
import math
from pathlib import Path
import time
import draccus
from .config import YubiTeleoperatorConfig, SIDES
from .quest import QuestInput
from .gripper import GripperInput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config_path", required=True)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--duration", type=float, default=10)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--durationは正の有限値にしてください")
    raw = json.loads(Path(args.config_path).read_text())["teleop"]
    if raw.pop("type") != "yubi":
        parser.error("YUBI設定を指定してください")
    config = draccus.decode(YubiTeleoperatorConfig, raw)
    config.validate()
    # 読取り専用デモでは未校正の角度も表示する。制御用入力は範囲外を拒否する。
    quest = QuestInput(config)
    grippers = {side: GripperInput(config.arms[side], input_timeout_s=config.input_timeout_s,
                                   connect_timeout_s=config.connect_timeout_s, strict=False) for side in SIDES}
    with ExitStack() as cleanup:
        for source in [quest, *grippers.values()]:
            cleanup.callback(source.close)
        if not args.mock:
            quest.connect()
            for source in grippers.values():
                source.connect()
        started = time.monotonic()
        try:
            while time.monotonic()-started < args.duration:
                if args.mock:
                    fingers = {side: {"opening": 0.5, "encoder_rad":
                               (arm.encoder_open_rad+arm.encoder_closed_rad)/2} for side, arm in config.arms.items()}
                    hands = {f"{side}.{key}": value for side in SIDES
                             for key, value in {"p0": 0.1, "p1": 0.0, "p2": 0.2, "clutch": False}.items()}
                else:
                    hands = quest.snapshot()
                    fingers = {side: source.snapshot() for side, source in grippers.items()}
                print(json.dumps({"mock": args.mock, "hands": hands, "grippers": fingers}, ensure_ascii=False), flush=True)
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
