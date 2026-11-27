"""実機を使わず、ROS 2の構成とMoveItの計画・実行を確認する。"""

from contextlib import suppress
import os
from pathlib import Path
import signal
import subprocess
from tempfile import TemporaryDirectory
import time
from xml.etree import ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml


def wait(node, ready, timeout, message):
    deadline = time.monotonic() + timeout
    while not ready():
        if time.monotonic() >= deadline:
            raise AssertionError(message)
        rclpy.spin_once(node, timeout_sec=0.1)


def complete(node, client, goal, label):
    request = client.send_goal_async(goal)
    wait(node, request.done, 10, f"{label}: 受付タイムアウト")
    handle = request.result()
    assert handle.accepted, f"{label}: 指令を拒否"
    result = handle.get_result_async()
    wait(node, result.done, 30, f"{label}: 完了タイムアウト")
    reply = result.result()
    assert reply.status == GoalStatus.STATUS_SUCCEEDED and reply.result.error_code.val == MoveItErrorCodes.SUCCESS, (
        label, reply.status, reply.result.error_code
    )
    return reply.result


def check_motion(node, planner, executor, positions, group, target):
    plan = MoveGroup.Goal()
    plan.request.group_name = group
    plan.request.start_state.is_diff = True
    plan.request.allowed_planning_time = 5.0
    plan.request.num_planning_attempts = 5
    plan.request.max_velocity_scaling_factor = 0.6
    plan.request.max_acceleration_scaling_factor = 0.6
    plan.request.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=name, position=angle, tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
        for name, angle in target.items()
    ])]
    plan.planning_options.plan_only = True
    result = complete(node, planner, plan, f"{group}の計画")
    trajectory = result.planned_trajectory.joint_trajectory
    assert set(trajectory.joint_names) == set(target) and len(trajectory.points) >= 2, group
    complete(node, executor, ExecuteTrajectory.Goal(trajectory=result.planned_trajectory), f"{group}の実行")
    wait(node, lambda: all(abs(positions[name] - angle) < 0.004 for name, angle in target.items()),
         5, f"{group}: /joint_statesで目標角への到達を確認できません")
    print(f"{group}: {len(target)}軸の軌道を実行", flush=True)


def check_robot(config, reach, pan_tilt, domain_id):
    rclpy.init(domain_id=domain_id)
    node = rclpy.create_node("topoarm_ci")
    positions = {}
    descriptions = []
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: positions.update(zip(msg.name, msg.position, strict=True)),
                             qos_profile_sensor_data)
    node.create_subscription(String, "/robot_description", lambda msg: descriptions.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    planner = ActionClient(node, MoveGroup, "/move_action")
    executor = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    try:
        expected = set(config["joints"])
        if pan_tilt:
            expected.update(config["pan_tilt"]["joints"])
        assert len(expected) == (18 if pan_tilt else 16)
        wait(node, lambda: bool(descriptions) and expected <= positions.keys(), 40,
             "URDFと全駆動軸のjoint_statesを受信できません")
        assert set(positions) == expected | {"L_gripper_mimic", "R_gripper_mimic"}, set(positions)

        robot = ET.fromstring(descriptions[-1])
        links = {link.get("name") for link in robot.findall("link")}
        for name in ("neck_pan_servo_link", "camera_link", "camera_optical_frame"):
            assert (name in links) == pan_tilt, (name, pan_tilt)
        base = robot.find("link[@name='base_link']")
        assert all("base_link_07.stl" not in mesh.get("filename") for mesh in base.iter("mesh")), (
            "標準胴体にパン側の筐体が残っています"
        )

        assert planner.wait_for_server(timeout_sec=30.0), "/move_actionが未起動です"
        assert executor.wait_for_server(timeout_sec=10.0), "/execute_trajectoryが未起動です"
        check_motion(node, planner, executor, positions, "both_arms", reach)
        for prefix, group, limit in (("L", "left_gripper", "min"), ("R", "right_gripper", "max")):
            joint = f"{prefix}_gripper_joint"
            check_motion(node, planner, executor, positions, group, {joint: config["joints"][joint][limit]})
        if pan_tilt:
            check_motion(node, planner, executor, positions, "neck", {"neck_pan_joint": 0.15, "neck_tilt_joint": 0.12})
        print(f"{len(expected)}軸のURDF、joint_states、MoveItを確認", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_variant(source, reach, directory, pan_tilt, domain_id):
    config = yaml.safe_load(source.read_text())
    config["modules"] = ["pan_tilt"] if pan_tilt else []
    settings = config["options"]
    settings["floor_config"] = str((source.parent / settings["floor_config"]).resolve())
    hardware = directory / f"follower-{domain_id}.yaml"
    hardware.write_text(yaml.safe_dump(config, sort_keys=False))
    log_path = directory / f"launch-{domain_id}.log"
    launch_args = ["ros2", "launch", "topoarm_moveit", "moveit.launch.py", "start_rviz:=false",
                   "use_fake_hardware:=true", f"config_file:={hardware}"]
    env = {**os.environ, "ROS_DOMAIN_ID": str(domain_id), "ROS_LOG_DIR": str(directory / f"ros-{domain_id}")}
    with log_path.open("w") as log:
        launch = subprocess.Popen(launch_args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            check_robot(config, reach, pan_tilt, domain_id)
        except Exception:
            print("\n".join(log_path.read_text(errors="replace").splitlines()[-100:]), flush=True)
            raise
        finally:
            if launch.poll() is None:
                with suppress(ProcessLookupError):
                    os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    with suppress(ProcessLookupError):
                        os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()


if __name__ == "__main__":
    hardware = Path(get_package_share_directory("topoarm_description")) / "config/hardware/follower.yaml"
    srdf = Path(get_package_share_directory("topoarm_moveit")) / "config/topoarm.srdf"
    reach = {joint.get("name"): float(joint.get("value")) for joint in
             ET.parse(srdf).findall("group_state[@name='reach'][@group='both_arms']/joint")}
    assert len(reach) == 14, "両腕のreach姿勢には14軸が必要です"
    with TemporaryDirectory(prefix="topoarm-ci-") as folder:
        for pan_tilt, domain_id in ((False, 51), (True, 52)):
            run_variant(hardware, reach, Path(folder), pan_tilt, domain_id)
