"""Verify Nav2 planning only: costmap alive + one ComputePathToPose action goal.

Sends no velocity command and does not touch the vehicle.
"""
from __future__ import annotations

import sys
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def main() -> int:
    forward = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node("wildos_nav2_path_probe")
    state = {"costmap": 0, "pose": None}

    def on_costmap(_msg: OccupancyGrid):
        state["costmap"] += 1

    def on_odom(msg: Odometry):
        state["pose"] = msg.pose.pose

    node.create_subscription(OccupancyGrid, "/global_costmap/costmap", on_costmap, 10)
    node.create_subscription(Odometry, "/wildos/odom", on_odom, 10)

    deadline = time.time() + 20.0
    while time.time() < deadline and (state["pose"] is None or state["costmap"] < 3):
        rclpy.spin_once(node, timeout_sec=0.2)
    print("costmap_msgs", state["costmap"])
    if state["pose"] is None:
        print("RESULT no odom pose")
        rclpy.shutdown()
        return 1
    pose = state["pose"]
    print("robot_pose", round(pose.position.x, 2), round(pose.position.y, 2))

    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=15.0):
        print("RESULT compute_path_to_pose action unavailable")
        rclpy.shutdown()
        return 1

    goal = ComputePathToPose.Goal()
    goal.use_start = False
    goal.planner_id = "GridBased"
    goal.goal.header.frame_id = "map"
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = pose.position.x + forward
    goal.goal.pose.position.y = pose.position.y
    goal.goal.pose.position.z = 0.0
    goal.goal.pose.orientation.w = 1.0
    goal.goal.pose.orientation.x = 0.0
    goal.goal.pose.orientation.y = 0.0
    goal.goal.pose.orientation.z = 0.0
    print("goal", round(goal.goal.pose.position.x, 2), round(goal.goal.pose.position.y, 2))

    send_future = client.send_goal_async(goal)
    deadline = time.time() + 20.0
    while not send_future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not send_future.done():
        print("RESULT goal send timed out")
        rclpy.shutdown()
        return 1
    handle = send_future.result()
    if handle is None or not handle.accepted:
        print("RESULT goal rejected")
        rclpy.shutdown()
        return 1

    result_future = handle.get_result_async()
    deadline = time.time() + 25.0
    while not result_future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not result_future.done():
        print("RESULT planning timed out")
        rclpy.shutdown()
        return 1
    path = result_future.result().result.path
    poses = list(path.poses) if path is not None else []
    print("path_poses", len(poses))
    if poses:
        print("first", round(poses[0].pose.position.x, 2), round(poses[0].pose.position.y, 2))
        print("last", round(poses[-1].pose.position.x, 2), round(poses[-1].pose.position.y, 2))
        print("RESULT planning_ok")
    else:
        print("RESULT empty_path")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
