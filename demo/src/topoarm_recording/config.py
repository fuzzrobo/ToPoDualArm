"""デモの共通YAMLと、旧形式の収録・推論設定を読む。"""
from pathlib import Path

import yaml

from topoarm_recording import ROOT

DEFAULT_CONFIG = ROOT / "demo.yaml"
DEFAULT_OUTPUT = Path("outputs/right_to_left_act")


def read_config(path):
    file = Path(path).resolve()
    config = yaml.safe_load(file.read_text())
    if not isinstance(config, dict):
        raise ValueError(f"設定はYAMLのマップで指定してください: {file}")
    return config, file.parent


def settings(config, section):
    if not {"record", "train", "infer"} & config.keys():
        return {} if section == "train" else config.copy()
    options = config.get(section, {})
    if not isinstance(options, dict):
        raise ValueError(f"{section}の設定はYAMLのマップで指定してください")
    shared = {key: config[key] for key in ("robot", "cameras", "task") if key in config}
    return {**shared, **options}


def device_arguments(value, folder):
    options = value.copy() if isinstance(value, dict) else {"hardware": value}
    path = options.pop("hardware", None)
    if not isinstance(path, (str, Path)) or not str(path).strip():
        raise ValueError("機体YAMLのパスを指定してください")
    if options.keys() - {"port", "hardware_verified"}:
        raise ValueError("機体設定で指定できる項目はhardware・port・hardware_verifiedです")
    return {"path": (folder / path).resolve(), **options}


def camera_devices(value, folder):
    if not isinstance(value, (dict, str, Path)):
        raise ValueError("camerasにはカメラの対応表を指定してください")
    devices = value if isinstance(value, dict) else yaml.safe_load((folder / value).read_text())
    if not isinstance(devices, dict) or set(devices) not in ({"left", "right"}, {"left", "front", "right"}):
        raise ValueError("カメラ設定にはleft/rightが必要です。frontは省略できます")
    return devices


def training_output(config, folder):
    return folder / settings(config, "train").get("output", DEFAULT_OUTPUT)


def inference_config(path):
    config, folder = read_config(path)
    options = settings(config, "infer")
    if "infer" in config and "checkpoint" not in options and "models" not in options:
        checkpoint = training_output(config, folder) / "checkpoints/last/pretrained_model"
        options["checkpoint"] = checkpoint
        if "initial_pose_rad" not in options:
            options.setdefault("initial_pose_file", checkpoint / "initial_pose.json")
    # 推論プリセットに機体・カメラの指定がなければ demo.yaml を使う。
    if not {"robot", "cameras"} <= options.keys():
        defaults, default_folder = read_config(DEFAULT_CONFIG)
    else:
        defaults, default_folder = {}, folder
    robot_folder = folder if "robot" in options else default_folder
    camera_folder = folder if "cameras" in options else default_folder
    options["robot"] = device_arguments(options.get("robot", defaults.get("robot")), robot_folder)
    backend = options.setdefault("backend", "u2d2")
    if backend not in ("u2d2", "vsido"):
        raise ValueError("推論のbackendにはu2d2またはvsidoを指定してください")
    if backend == "vsido":
        options["robot"].pop("port", None)
        for field in ("peer_address", "bind_address"):
            value = options.get(f"vsido_{field}", "")
            if not isinstance(value, str):
                raise ValueError(f"vsido_{field}にはIPv4アドレスを指定してください")
            options["robot"][field] = value
    options["cameras"] = camera_devices(options.get("cameras", defaults.get("cameras")), camera_folder)
    return options, folder
