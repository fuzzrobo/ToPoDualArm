#!/usr/bin/env python3
"""リーダーの校正済み関節状態をフォロワーの位置コントローラーへ渡す。"""
import math
import time

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from topoarm_leader_follower import LeaderFollowerTransform, NECK_JOINTS, scale_negative_angle


TIMEOUT_S = 0.15


class LeaderFollower(Node):
    def __init__(self):
        super().__init__("leader_follower")
        self.joints = tuple(self.declare_parameter("joints", Parameter.Type.STRING_ARRAY).value)
        neck_mode = self.declare_parameter("neck_mode", "fixed").value
        self.scale = self.declare_parameter("neck_tilt_negative_scale", 1.0).value
        self.fake = self.declare_parameter("leader_fake", True).value
        if len(set(self.joints)) != len(self.joints) or not set(NECK_JOINTS) <= set(self.joints):
            raise ValueError("コントローラーの関節名が不正です")
        if neck_mode not in ("fixed", "follow"):
            raise ValueError("ROS 2の首モードはfixedまたはfollowを指定してください")
        scale_negative_angle(0.0, self.scale)
        self.transform = LeaderFollowerTransform({name: name for name in self.joints}, neck_mode)
        self.last_stamp = self.last_bus_read = 0
        self.pending = None
        self.publisher = self.create_publisher(
            Float64MultiArray, "/follower/full_position_controller/commands", 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            DynamicJointState, "/leader/joint_state_broadcaster/dynamic_joint_states",
            self.on_state, qos)
        self.create_timer(1 / 30, self.tick)

    def on_state(self, message):
        self.pending = None
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        if stamp_ns <= self.last_stamp or not self.fresh_stamp(stamp_ns):
            return
        self.last_stamp = stamp_ns
        states = dict(zip(message.joint_names, message.interface_values))
        if len(states) != len(message.joint_names) or len(states) != len(message.interface_values):
            return
        positions = {}
        try:
            for name in self.transform.source_joints:
                joint = states[name]
                values = dict(zip(joint.interface_names, joint.values))
                if not self.fake and (values["valid"] != 1 or values["torque_enabled"] != 0):
                    return
                positions[name] = values["position"]
            if not self.fake:
                # 状態配信の再送と、実機バスでの新規読取りを区別する。
                bus = states["bus"]
                read_end = bus.values[bus.interface_names.index("read_end_monotonic_s")]
                if not math.isfinite(read_end) or read_end <= self.last_bus_read:
                    return
                self.last_bus_read = read_end
            goals = self.transform.targets(positions, neck_tilt_negative_scale=self.scale)
        except (KeyError, IndexError, ValueError):
            return
        command = [goals[name] for name in self.joints]
        if all(math.isfinite(value) for value in command):
            self.pending = time.monotonic(), stamp_ns, command

    def fresh_stamp(self, stamp_ns):
        return 0 <= self.get_clock().now().nanoseconds - stamp_ns <= TIMEOUT_S * 10**9

    def tick(self):
        if self.pending is None or not self.publisher.get_subscription_count():
            return
        received, stamp, command = self.pending
        self.pending = None
        if time.monotonic() - received <= TIMEOUT_S and self.fresh_stamp(stamp):
            self.publisher.publish(Float64MultiArray(data=command))


def main():
    rclpy.init()
    node = None
    try:
        node = LeaderFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
