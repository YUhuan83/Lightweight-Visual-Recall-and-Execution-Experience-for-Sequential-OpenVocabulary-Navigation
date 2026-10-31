"""Brake BeamNG after a bounded acquisition movement."""
import math
import time

import rclpy
from beamng_msgs.msg import StateSensor, VehicleControl
from rclpy.node import Node


class Guard(Node):
    def __init__(self):
        super().__init__("bounded_motion_guard")
        self.declare_parameter("limit", 1.5)
        self.start = None
        self.done = False
        self.publisher = self.create_publisher(VehicleControl, "/control", 10)
        self.create_subscription(StateSensor, "/vehicles/ego/sensors/state", self.callback, 10)

    def callback(self, message):
        point = (message.position.x, message.position.y, message.position.z)
        if self.start is None:
            self.start = point
            return
        if math.dist(self.start, point) >= float(self.get_parameter("limit").value):
            command = VehicleControl(steering=0.0, throttle=0.0, brake=1.0,
                                     parkingbrake=0.0, clutch=0.0, gear=2)
            for _ in range(20):
                self.publisher.publish(command)
                time.sleep(0.05)
            self.done = True
            self.get_logger().warn("bounded acquisition motion reached limit; full brake applied")


rclpy.init(); node=Guard(); deadline=time.monotonic()+120.0
while rclpy.ok() and not node.done and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=0.1)
node.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if node.done else 1)
