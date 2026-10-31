"""Save one camera frame from the BeamNG ROS2 image topic."""
from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg

TOPIC = "/vehicles/ego/sensors/front_cam/colour"


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/scene_frame.png"
    topic = sys.argv[2] if len(sys.argv) > 2 else TOPIC
    rclpy.init()
    node = Node("wildos_frame_dump")
    holder = {}
    node.create_subscription(ImageMsg, topic, lambda m: holder.setdefault("msg", m), 10)
    deadline = time.time() + 20.0
    while "msg" not in holder and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "msg" not in holder:
        print("FRAME_TIMEOUT")
        rclpy.shutdown()
        return 1
    msg = holder["msg"]
    array = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
    Image.fromarray(array, mode="RGB").save(out)
    print(f"FRAME {out} {msg.width}x{msg.height}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
