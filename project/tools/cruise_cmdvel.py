"""Drive straight at a controlled speed until a target distance is reached.

Publishes geometry_msgs/Twist to the official cmd_vel->control adapter
(/wildos/cmd_vel), which converts it to BeamNG VehicleControl. Stops (zero
command) once the travelled distance or timeout is reached.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CruiseCmd(Node):
    def __init__(self):
        super().__init__("wildos_cruise_cmdvel")
        self.declare_parameter("speed", 3.0)
        self.declare_parameter("distance", 100.0)
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("cmd_vel_topic", "/wildos/cmd_vel")
        self.pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.create_subscription(Odometry, "/spot1/odom", self.cb, 10)
        self.odom = None
        self.start = None
        self.done = False
        self.stop_sent = 0
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            "cruise speed=%.1f distance=%.1f timeout=%.0f" % (
                self.get_parameter("speed").value,
                self.get_parameter("distance").value,
                self.get_parameter("timeout").value))

    def cb(self, m):
        self.odom = m

    def travelled(self):
        if self.odom is None or self.start is None:
            return 0.0
        p = self.odom.pose.pose.position
        return math.hypot(p.x - self.start[0], p.y - self.start[1])

    def tick(self):
        now = time.time()
        if self.odom is None:
            return
        if self.start is None:
            p = self.odom.pose.pose.position
            self.start = (p.x, p.y)
            self.t0 = now
        d = self.travelled()
        elapsed = now - self.t0
        speed = float(self.get_parameter("speed").value)
        if self.done:
            self.pub.publish(Twist())
            self.stop_sent += 1
            if self.stop_sent > 5:
                self.get_logger().info("cruise complete after %.1f m in %.0f s" % (d, elapsed))
                raise SystemExit(0)
            return
        if d >= float(self.get_parameter("distance").value) or elapsed > float(self.get_parameter("timeout").value):
            self.done = True
            self.get_logger().info("target reached: %.1f m in %.0f s" % (d, elapsed))
            return
        cmd = Twist()
        cmd.linear.x = speed
        self.pub.publish(cmd)
        if int(elapsed * 10) % 50 == 0:
            self.get_logger().info("cruising: %.1f m, %.0f s" % (d, elapsed))


def main():
    rclpy.init()
    node = CruiseCmd()
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
