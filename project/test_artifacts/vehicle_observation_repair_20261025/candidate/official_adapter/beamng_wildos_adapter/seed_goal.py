"""Publish one explicit experiment waypoint to start official graph exploration."""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .geometry import forward_xy


class SeedGoal(Node):
    def __init__(self):
        super().__init__("beamng_wildos_seed_goal")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("goal_topic", "/spot1/imgnav_waypoint")
        self.declare_parameter("distance", 25.0)
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.subscription = self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.callback, 10)
        self.sent = False

    def callback(self, odom):
        if self.sent:
            return
        fx, fy = forward_xy(odom.pose.pose.orientation)
        norm = math.hypot(fx, fy)
        if norm < 1e-6:
            return
        distance = float(self.get_parameter("distance").value)
        goal = PoseStamped(); goal.header = odom.header
        goal.pose.position.x = odom.pose.pose.position.x + distance * fx / norm
        goal.pose.position.y = odom.pose.pose.position.y + distance * fy / norm
        goal.pose.position.z = odom.pose.pose.position.z
        goal.pose.orientation = odom.pose.pose.orientation
        for _ in range(5):
            goal.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self.sent = True
        self.get_logger().warn(
            "Published explicit experiment seed goal; official graph planner remains authoritative")


def main(args=None):
    rclpy.init(args=args); node = SeedGoal()
    deadline = time.monotonic() + 30.0
    while rclpy.ok() and not node.sent and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    if not node.sent:
        raise SystemExit("no odometry received for seed goal")
