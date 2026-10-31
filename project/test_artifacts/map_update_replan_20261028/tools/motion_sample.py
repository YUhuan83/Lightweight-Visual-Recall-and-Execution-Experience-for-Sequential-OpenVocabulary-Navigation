"""Collect a bounded simulator-truth motion sample for server acceptance."""
import json
import math
import time

import rclpy
from beamng_msgs.msg import StateSensor
from rclpy.node import Node


class Sample(Node):
    def __init__(self):
        super().__init__("official_motion_sample")
        self.samples = []
        self.create_subscription(StateSensor, "/vehicles/ego/sensors/state", self.callback, 10)

    def callback(self, msg):
        velocity = math.sqrt(msg.velocity.x**2 + msg.velocity.y**2 + msg.velocity.z**2)
        self.samples.append({"t": time.monotonic(), "x": msg.position.x,
                             "y": msg.position.y, "z": msg.position.z,
                             "speed": velocity})


rclpy.init(); node=Sample(); deadline=time.monotonic()+15.0
while rclpy.ok() and time.monotonic()<deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
samples=node.samples
report={"samples":len(samples),"max_speed":max((s["speed"] for s in samples),default=None)}
if len(samples)>=2:
    a,b=samples[0],samples[-1]
    report.update({"start":[a["x"],a["y"],a["z"]],"end":[b["x"],b["y"],b["z"]],
                   "displacement":math.dist([a["x"],a["y"],a["z"]],[b["x"],b["y"],b["z"]])})
print(json.dumps(report,indent=2)); node.destroy_node(); rclpy.shutdown()
