"""300 mmモデルと、指定されたオプションをMuJoCoに読み込む。"""
import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco


def load_model(modules=()):
    names = tuple(modules)
    if names not in ((), ("pan_tilt",)):
        raise ValueError("modulesは[]または[pan_tilt]を指定してください")

    folder = Path(__file__).resolve().parent
    base = folder / "robot.xml"
    if not names:
        return mujoco.MjModel.from_xml_path(str(base))

    robot = ET.parse(base).getroot()
    part = ET.parse(folder / "pan_tilt.xml").getroot()
    torso = robot.find(".//body[@name='torso_link']")
    # 既存のパンチルト付きモデルの関節順を維持する。
    arm = torso.find("body[@name='L_shoulder_mount']")
    index = list(torso).index(arm)
    torso[index:index] = list(part.findall("body"))
    actuators = robot.find("actuator")
    actuators[:0] = list(part.find("actuator"))
    for section in ("asset", "contact"):
        robot.find(section).extend(part.find(section))

    # 組み立てたXMLをメモリーから読むので、資産パスを実ファイルに合わせる。
    for mesh in robot.findall("asset/mesh"):
        mesh.set("file", str((folder / mesh.attrib["file"]).resolve()))
    return mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="機体YAML")
    args = parser.parse_args()
    modules = ()
    if args.config:
        import yaml

        config = yaml.safe_load(args.config.read_text())
        modules = config.get("modules", ())
    from mujoco import viewer

    viewer.launch(load_model(modules))


if __name__ == "__main__":
    main()
