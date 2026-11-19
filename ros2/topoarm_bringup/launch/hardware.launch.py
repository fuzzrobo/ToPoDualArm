"""実機用の明示起動。既定ではTorque OFFかつ未校正のためconfigureを拒否する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def start(context, defaults):
    controllers = ["joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                   "left_gripper_controller", "right_gripper_controller", "neck_controller"]
    if IfCondition(LaunchConfiguration("start_current_limits_controller")).evaluate(context):
        controllers.append("current_limits_controller")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("topoarm_bringup"), "launch", "base.launch.py"])),
        launch_arguments={**{key: LaunchConfiguration(key) for key in defaults},
                          "controllers": " ".join(controllers)}.items())]


def generate_launch_description():
    defaults = {"start_rviz":"false", "use_fake_hardware":"false", "torque":"false",
                "hardware_verified":"false","configure_devices":"false",
                "config_file":PathJoinSubstitution([FindPackageShare("topoarm_description"),
                    "config","hardware","follower.yaml"])}
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value, description="実機確認前に変更しないでください")
          for key, value in defaults.items()],
        DeclareLaunchArgument("start_current_limits_controller", default_value="false",
                              description="運転中にROSから電流上限を変更するコントローラーを起動する"),
        OpaqueFunction(function=start, args=[defaults]),
    ])
