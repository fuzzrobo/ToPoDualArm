"""共通シミュレーション設定と任意のシーン設定を読み、各ファイル基準で参照を解決する。"""
from pathlib import Path
import copy
import yaml

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = ROOT/'config/simulation.yaml'


def merge(base, overlay):
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        result[key] = merge(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else copy.deepcopy(value)
    return result


def _read(path):
    path = Path(path).resolve()
    value = yaml.safe_load(path.read_text()) or {}
    if not isinstance(value, dict):
        raise ValueError(f'設定には辞書を指定してください: {path}')
    if 'camera_rig' in value:
        rig = (path.parent/value['camera_rig']).resolve()
        mounts = yaml.safe_load(rig.read_text())
        for mount in mounts.values():
            if 'lens_file' in mount:
                mount['lens_file'] = str((rig.parent/mount['lens_file']).resolve())
        value['cameras'] = merge(mounts, value.get('cameras', {}))
        value['camera_rig'] = str(rig)
    for mount in value.get('cameras', {}).values():
        if 'lens_file' in mount:
            mount['lens_file'] = str((path.parent/mount['lens_file']).resolve())
    for model in value.get('actuators', {}).get('models', {}).values():
        model['parameters'] = str((path.parent/model['parameters']).resolve())
    return value


def load_settings(config_file=DEFAULT_CONFIG, scene_file=None):
    base = _read(config_file)
    return merge(base, _read(scene_file)) if scene_file and Path(scene_file).resolve() != Path(config_file).resolve() else base
