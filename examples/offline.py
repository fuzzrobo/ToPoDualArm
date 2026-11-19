"""18軸をゼロ姿勢に置き、右肩の目標角をSDKで補正する。"""

from pathlib import Path

from topoarm_sdk import OfflineLimiter, load_config

config_path = Path(__file__).resolve().parents[1] / "config" / "example.yaml"
config = load_config(str(config_path))
names = [joint.name for joint in config.joints]
current = [0.0] * len(names)
target = current.copy()
shoulder = names.index("R_joint1")
target[shoulder] = 1.0

limited = OfflineLimiter(config).prepare(current, target)
print(f"R_joint1: {target[shoulder]:.2f} → {limited[shoulder]:.2f} rad")
