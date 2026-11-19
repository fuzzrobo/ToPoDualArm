"""リーダーとフォロワーを別の名前空間で起動する。既定は両方fake。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def start(context, bringup):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return IfCondition(LaunchConfiguration(name)).evaluate(context)

    leader_fake = enabled("leader_fake")
    follower_fake = enabled("follower_fake")
    follower_torque = enabled("follower_torque")
    if follower_torque and (leader_fake or follower_fake or not enabled("follower_verified")):
        raise RuntimeError("follower_torqueには実機リーダー・実機フォロワーとfollower_verifiedが必要です")
    if not leader_fake and not enabled("leader_verified"):
        raise RuntimeError("リーダー実機の使用にはleader_verifiedが必要です")
    if not follower_fake and not enabled("follower_verified"):
        raise RuntimeError("フォロワー実機の使用にはfollower_verifiedが必要です")

    config = yaml.safe_load((bringup / "config/controllers.yaml").read_text())
    joints = config["/**/full_position_controller"]["ros__parameters"]["joints"]
    leader = yaml.safe_load(Path(value("leader_config")).read_text())
    if not leader_fake and not follower_fake:
        follower = yaml.safe_load(Path(value("follower_config")).read_text())
        if Path(leader["connection"]["port"]).resolve() == Path(follower["connection"]["port"]).resolve():
            raise RuntimeError("リーダーとフォロワーに別々のシリアルポートが必要です")
    neck = {"neck_pan_joint", "neck_tilt_joint"}
    required = set(joints) if value("neck_mode") == "follow" or leader_fake else set(joints) - neck
    if not required <= set(leader["joints"]):
        raise RuntimeError("leader YAMLに必要な関節が揃っていません")
    scale = float(leader.get("input_transform", {}).get("neck_tilt_negative_scale", 1.0))

    def system(namespace, fake, read_only, torque, controllers, rviz):
        arguments = {
            "namespace": namespace,
            # RVizのTFは標準トピックを使い、leaderのTFだけを隔離する。
            "tf_namespace": "/leader" if namespace == "leader" else "/",
            "joint_states_topic": "joint_state_broadcaster/joint_states",
            "use_fake_hardware": str(fake).lower(),
            "read_only": str(read_only).lower(),
            "torque": str(torque).lower(),
            "hardware_verified": value(f"{namespace}_verified"),
            "config_file": value(f"{namespace}_config"),
            "configure_devices": "false",
            "controllers": controllers,
            "update_rate": "" if fake else "30",
            "start_rviz": rviz,
        }
        return GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch/base.launch.py")),
            launch_arguments=arguments.items())])

    state = "joint_state_broadcaster"
    position_control = f"{state} full_position_controller"
    return [
        system("leader", leader_fake, not leader_fake, False, position_control if leader_fake else state, "false"),
        system("follower", follower_fake, False, follower_torque, position_control, value("start_rviz")),
        Node(package="topoarm_teleop", executable="leader_follower_node", output="screen",
             parameters=[{"joints": joints, "neck_mode": value("neck_mode"),
                          "neck_tilt_negative_scale": scale, "leader_fake": leader_fake}]),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory("topoarm_bringup"))
    hardware = Path(get_package_share_directory("topoarm_description")) / "config/hardware"
    return LaunchDescription([
        DeclareLaunchArgument("leader_fake", default_value="true"),
        DeclareLaunchArgument("follower_fake", default_value="true"),
        DeclareLaunchArgument("leader_verified", default_value="false"),
        DeclareLaunchArgument("follower_verified", default_value="false"),
        DeclareLaunchArgument("follower_torque", default_value="false"),
        DeclareLaunchArgument("leader_config", default_value=str(hardware / "leader.yaml")),
        DeclareLaunchArgument("follower_config", default_value=str(hardware / "follower.yaml")),
        DeclareLaunchArgument("neck_mode", default_value="fixed", choices=["fixed", "follow"]),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        OpaqueFunction(function=start, args=[bringup]),
    ])
