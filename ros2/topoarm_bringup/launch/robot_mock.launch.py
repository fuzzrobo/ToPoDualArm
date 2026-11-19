"""公開API用の単一mock。リーダーの中継ノードは起動しない。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = PathJoinSubstitution([FindPackageShare("topoarm_bringup"), "launch", "base.launch.py"])
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(base), launch_arguments={
        "use_fake_hardware": "true", "namespace": "follower",
        "joint_states_topic": "joint_state_broadcaster/joint_states",
        "controllers": "joint_state_broadcaster full_position_controller",
    }.items())])
