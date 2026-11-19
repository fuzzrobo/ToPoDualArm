"""通常の軌道コントローラーをfake hardwareで起動する。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("topoarm_bringup"), "launch", "hardware.launch.py"])),
        launch_arguments={
            "use_fake_hardware": "true",
            "torque": "false",
            "hardware_verified": "false",
            "configure_devices": "false",
        }.items())])
