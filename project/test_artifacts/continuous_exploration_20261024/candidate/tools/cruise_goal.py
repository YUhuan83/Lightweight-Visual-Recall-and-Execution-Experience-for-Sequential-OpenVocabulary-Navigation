"""Publish a rolling goal ~D metres ahead of the vehicle on the robot's heading.

Feeds the official Nav2 execution layer (/spot1/goal_pose) so the vehicle keeps
moving down the road while WildOS builds the frontier/traversability map.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Cruise(Node):
    def __init__(self):
        super().__init__("wildos_cruise_goal")
        self.declare_parameter("distance", 12.0)
        self.declare_parameter("period", 3.0)
        self.declare_parameter("goal_topic", "/spot1/goal_pose")
        self.pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.create_subscription(Odometry, "/spot1/odom", self.cb, 10)
        self.odom = None
        self.n = 0
        self.create_timer(float(self.get_parameter("period").value), self.tick)

    def cb(self, m):
        self.odom = m

    def tick(self):
        if self.odom is None:
            return
        q = self.odom.pose.pose.orientation
        fx = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        fy = 2.0 * (q.x * q.y + q.w * q.z)
        norm = math.hypot(fx, fy) or 1.0
        d = float(self.get_parameter("distance").value)
        g = PoseStamped()
        g.header = self.odom.header
        g.pose.position.x = self.odom.pose.pose.position.x + d * fx / norm
        g.pose.position.y = self.odom.pose.pose.position.y + d * fy / norm
        g.pose.position.z = self.odom.pose.pose.position.z
        g.pose.orientation = q
        for _ in range(3):
            g.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(g)
        self.n += 1
        if self.n % 5 == 0:
            self.get_logger().info(
                f"cruise goal #{self.n} at ({g.pose.position.x:.1f},{g.pose.position.y:.1f})")


def main():
    rclpy.init()
    node = Cruise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
