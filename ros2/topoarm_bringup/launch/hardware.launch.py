"""関節状態と軌道指令を、実機またはfake hardwareで起動する。"""
import json
from pathlib import Path
from xml.etree import ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import xacro


def start(context, model, arguments):
    mappings = {name: LaunchConfiguration(name).perform(context) for name in arguments}
    mappings["config_file"] = str(Path(mappings["config_file"]).resolve())
    xacro_file = Path(model.perform(context)) / "urdf/topoarm.urdf.xacro"
    urdf = xacro.process_file(str(xacro_file), mappings=mappings).toxml()
    robot = ET.fromstring(urdf)
    joints = [joint.attrib["name"] for joint in robot.findall("ros2_control/joint")
              if joint.find("command_interface") is not None]
    context.launch_configurations["topoarm_command_joints"] = json.dumps(joints)
    controller_path = PathJoinSubstitution([FindPackageShare("topoarm_bringup"), "config", "controllers.yaml"])
    controllers = ParameterFile(controller_path, allow_substs=True)
    controller_parameters = str(controllers.evaluate(context))
    read_only = LaunchConfiguration("read_only")

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], remappings=[("~/robot_description", "/robot_description")], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "joint_trajectory_controller",
                        "--controller-manager-timeout", "30", "--param-file", controller_parameters],
             condition=UnlessCondition(read_only), output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30", "--param-file", controller_parameters],
             condition=IfCondition(read_only), output="screen"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([model, "rviz", "model.rviz"])],
             condition=IfCondition(LaunchConfiguration("start_rviz")), output="screen"),
    ]


def generate_launch_description():
    model = FindPackageShare("topoarm_description")
    defaults = {
        "config_file": PathJoinSubstitution([model, "config", "hardware", "follower.yaml"]),
        "use_fake_hardware": "false",
        "read_only": "false",
        "torque": "false",
        "hardware_verified": "false",
        "configure_devices": "false",
        "vsido_peer_address": "",
        "vsido_bind_address": "",
    }
    declarations = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    declarations.extend([
        DeclareLaunchArgument("backend", default_value="u2d2", choices=["u2d2", "vsido"]),
        DeclareLaunchArgument("start_rviz", default_value="false"),
    ])

    return LaunchDescription(declarations + [OpaqueFunction(function=start, args=[model, (*defaults, "backend")])])
