#!/usr/bin/env python3
"""共通の床設定をMoveItのplanning sceneへ適用する。"""
from pathlib import Path

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from shape_msgs.msg import SolidPrimitive
import yaml


def main():
    rclpy.init()
    node = rclpy.create_node("topoarm_floor_scene")
    try:
        config_file = node.declare_parameter("floor_config", "").value
        settings = yaml.safe_load(Path(config_file).read_text())
        if not settings["enabled"]:
            return
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError("MoveIt planning sceneサービスがありません")
        height = float(settings["z_m"]) + float(settings["margin_m"])
        floor = CollisionObject(id="topoarm_floor", operation=CollisionObject.ADD)
        floor.header.frame_id = "world"
        thickness = 0.4
        floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[10.0, 10.0, thickness])]
        pose = Pose()
        pose.position.z = height - thickness / 2
        pose.orientation.w = 1.0
        floor.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [floor]
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or not future.result().success:
            raise RuntimeError("床の追加に失敗しました")
        node.get_logger().info(f"床の計画境界 Z={height:g} m を追加しました")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
