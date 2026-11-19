"""指定されたコントローラーと共通のモデル・状態配信を起動する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {"start_rviz":"false", "use_fake_hardware":"true",
                "torque":"false", "read_only":"false",
                "hardware_verified":"false", "configure_devices":"false",
                "config_file": PathJoinSubstitution([
                    FindPackageShare("topoarm_description"), "config", "hardware", "follower.yaml"])}
    declarations = [DeclareLaunchArgument(k, default_value=v,
                    description="実機利用には校正・可動域・停止方法の確認が必要です") for k,v in defaults.items()]
    command = [FindExecutable(name="xacro"), " ", PathJoinSubstitution([
        FindPackageShare("topoarm_description"), "urdf", "topoarm.urdf.xacro"])]
    namespace = LaunchConfiguration("namespace")
    tf_namespace = LaunchConfiguration("tf_namespace")
    tf_remaps = [(f"/{topic}", PathJoinSubstitution([tf_namespace, topic])) for topic in ("tf", "tf_static")]
    for name in defaults:
        if name != "start_rviz":
            command += [" ", name+":=", LaunchConfiguration(name)]
    description = {"robot_description":ParameterValue(Command(command),value_type=str)}
    state = Node(package="robot_state_publisher", executable="robot_state_publisher",
                 namespace=namespace, parameters=[description], output="screen",
                 remappings=tf_remaps + [("joint_states", LaunchConfiguration("joint_states_topic"))])
    rviz = Node(package="rviz2",executable="rviz2",namespace=namespace,
                remappings=[("/robot_description", "robot_description")], arguments=["-d",PathJoinSubstitution([
        FindPackageShare("topoarm_description"),"rviz","model.rviz"])],
        condition=IfCondition(LaunchConfiguration("start_rviz")))
    return LaunchDescription(declarations + [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_namespace", default_value="/"),
        DeclareLaunchArgument("joint_states_topic", default_value="joint_states"),
        DeclareLaunchArgument("controllers", description="起動するコントローラー名を空白で区切る"),
        DeclareLaunchArgument("update_rate", default_value="", description="更新周期の上書き（Hz）"),
        OpaqueFunction(function=start_controllers), state, rviz,
    ])


def start_controllers(context):
    names = LaunchConfiguration("controllers").perform(context).split()
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    manager = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    config = PathJoinSubstitution([FindPackageShare("topoarm_bringup"), "config", "controllers.yaml"])
    parameters = [config]
    update_rate = LaunchConfiguration("update_rate").perform(context)
    if update_rate:
        parameters.append({"update_rate": int(update_rate)})
    control = Node(package="controller_manager",executable="ros2_control_node",
                   namespace=namespace,
                   parameters=parameters,
                   remappings=[("~/robot_description", "robot_description")],
                   output="screen")
    # 一つのspawnerでcontrollerを順にロードし、起動失敗をログに残す。
    spawner = Node(package="controller_manager",executable="spawner", namespace=namespace,
                   arguments=names+["--controller-manager",manager,
                                    "--controller-manager-timeout","30", "--param-file", config],output="screen")
    return [control, spawner]
