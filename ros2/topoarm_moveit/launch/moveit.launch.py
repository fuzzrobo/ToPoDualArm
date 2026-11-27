"""300 mmモデル、関節コントローラー、MoveItとRVizを起動する。"""
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def read_yaml(path):
    return yaml.safe_load(path.read_text())


def start(context, description, config, defaults):
    hardware_args = {name: LaunchConfiguration(name).perform(context) for name in defaults}
    hardware_path = Path(hardware_args["config_file"]).resolve()
    hardware_args["config_file"] = str(hardware_path)
    model = xacro.process_file(str(description / "urdf/topoarm.urdf.xacro"), mappings=hardware_args).toxml()
    urdf = ET.fromstring(model)
    links = {link.attrib["name"] for link in urdf.findall("link")}
    joints = [joint.attrib["name"] for joint in urdf.findall("ros2_control/joint")
              if joint.find("command_interface") is not None]
    semantic = ET.parse(config / "topoarm.srdf").getroot()
    for pair in semantic.findall("disable_collisions"):
        if not {pair.attrib["link1"], pair.attrib["link2"]} <= links:
            semantic.remove(pair)
    if "neck_pan_joint" not in joints:
        semantic.remove(semantic.find("group[@name='neck']"))
    controllers = read_yaml(config / "moveit_controllers.yaml")
    controllers["joint_trajectory_controller"]["joints"] = joints
    limits = read_yaml(config / "joint_limits.yaml")["joint_limits"]
    ompl_defaults = Path(get_package_share_directory("moveit_configs_utils")) / "default_configs/ompl_planning.yaml"
    ompl = {**read_yaml(ompl_defaults), **read_yaml(config / "ompl_planning.yaml")}
    robot = {
        "robot_description": model,
        "robot_description_semantic": ET.tostring(semantic, encoding="unicode"),
        "robot_description_kinematics": read_yaml(config / "kinematics.yaml"),
        "robot_description_planning": {"joint_limits": {name: limits[name] for name in joints}},
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
    }
    bringup = Path(get_package_share_directory("topoarm_bringup"))
    actions = [
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch/hardware.launch.py")),
            launch_arguments={**hardware_args, "start_rviz": "false"}.items(),
        )]),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[robot, {
                "allow_trajectory_execution": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_manage_controllers": False,
                "moveit_simple_controller_manager": controllers,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(config / "moveit.rviz")],
            parameters=[robot],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            output="screen",
        ),
    ]
    floor = read_yaml(hardware_path).get("options", {}).get("floor_config")
    if floor:
        actions.append(Node(
            package="topoarm_moveit",
            executable="floor_scene.py",
            parameters=[{"floor_config": str((hardware_path.parent / floor).resolve())}],
            output="screen",
        ))
    return actions


def generate_launch_description():
    description = Path(get_package_share_directory("topoarm_description"))
    config = Path(get_package_share_directory("topoarm_moveit")) / "config"
    defaults = {
        "use_fake_hardware": "true",
        "config_file": str(description / "config/hardware/follower.yaml"),
        "backend": "u2d2",
        "torque": "false",
        "hardware_verified": "false",
        "vsido_peer_address": "",
        "vsido_bind_address": "",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        DeclareLaunchArgument("start_rviz", default_value="true"),
        OpaqueFunction(function=start, args=[description, config, defaults]),
    ])
