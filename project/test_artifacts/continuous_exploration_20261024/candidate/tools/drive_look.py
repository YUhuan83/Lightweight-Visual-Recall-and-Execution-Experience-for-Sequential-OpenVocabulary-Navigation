"""Drive along the road to a target x, then turn left to face the left-side house.

Uses the official cmd_vel->control adapter. Read-only perception; commands only
the vehicle via /wildos/cmd_vel (which the adapter converts to BeamNG control).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveLook(Node):
    def __init__(self):
        super().__init__("wildos_drive_look")
        self.declare_parameter("target_x", 46.0)
        self.declare_parameter("approach_speed", 2.0)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("turn_rate", 0.55)
        self.declare_parameter("turn_degrees", 85.0)
        self.declare_parameter("timeout", 90.0)
        self.pub = self.create_publisher(Twist, "/wildos/cmd_vel", 10)
        self.create_subscription(Odometry, "/spot1/odom", self.cb, 10)
        self.odom = None
        self.phase = "approach"
        self.yaw0 = None
        self.t0 = time.time()
        self.create_timer(0.1, self.tick)
        self.get_logger().info("drive_look target_x=%.1f turn=%.0fdeg" % (
            self.get_parameter("target_x").value, self.get_parameter("turn_degrees").value))

    def cb(self, m):
        self.odom = m

    def tick(self):
        if self.odom is None:
            return
        if time.time() - self.t0 > float(self.get_parameter("timeout").value):
            self.stop("timeout")
            return
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        if self.phase == "approach":
            if p.x >= float(self.get_parameter("target_x").value):
                self.phase = "turn"
                self.yaw0 = yaw_of(q)
                self.get_logger().info("at x=%.1f y=%.1f, turning left" % (p.x, p.y))
                return
            c = Twist()
            c.linear.x = float(self.get_parameter("approach_speed").value)
            self.pub.publish(c)
        elif self.phase == "turn":
            dyaw = math.degrees(abs(math.atan2(math.sin(yaw_of(q) - self.yaw0),
                                               math.cos(yaw_of(q) - self.yaw0))))
            if dyaw >= float(self.get_parameter("turn_degrees").value):
                self.stop("turned %.0f deg at x=%.1f y=%.1f" % (dyaw, p.x, p.y))
                return
            c = Twist()
            c.linear.x = float(self.get_parameter("turn_speed").value)
            c.angular.z = float(self.get_parameter("turn_rate").value)
            self.pub.publish(c)
        else:
            self.pub.publish(Twist())

    def stop(self, msg):
        self.get_logger().info("stopping: " + msg)
        self.phase = "done"
        end = time.time() + 2.0
        while time.time() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        raise SystemExit(0)


def main():
    rclpy.init()
    node = DriveLook()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
