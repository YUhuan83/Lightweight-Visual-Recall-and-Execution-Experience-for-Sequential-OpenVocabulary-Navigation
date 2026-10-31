"""Convert read-only WildOS JSON results to PoseStamped goals.

No Nav2 action is called and no vehicle command is published in this phase.
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class GoalAdapter(Node):
    def __init__(self):
        super().__init__("wildos_goal_adapter")
        self.declare_parameter("result_topic", "/wildos/perception/result")
        self.declare_parameter("goal_topic", "/wildos/goal_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("odom_topic", "/wildos/odom")
        self.declare_parameter("target_standoff", 4.0)
        self.current_xy = None
        self.pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("result_topic").value), self.callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)

    def odom_callback(self, message: Odometry):
        self.current_xy = (message.pose.pose.position.x, message.pose.pose.position.y)

    def callback(self, message: String):
        data = json.loads(message.data)
        point = data.get("target_world_xyz")
        if point is None or len(point) != 3 or self.current_xy is None:
            return
        x, y, z = (float(v) for v in point)
        dx, dy = x - self.current_xy[0], y - self.current_xy[1]
        distance = math.hypot(dx, dy)
        if data.get("mode") == "target" and distance > 1e-6:
            standoff = min(float(self.get_parameter("target_standoff").value),
                           max(0.0, distance - 0.5))
            x -= dx / distance * standoff
            y -= dy / distance * standoff
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = str(self.get_parameter("frame_id").value)
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
        yaw = math.atan2(dy, dx)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
