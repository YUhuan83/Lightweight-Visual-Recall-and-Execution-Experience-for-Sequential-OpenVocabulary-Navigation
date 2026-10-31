"""Capture one ROS rgb8/bgr8 image for navigation diagnostics."""
import sys
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image


class Capture(Node):
    def __init__(self, topic):
        super().__init__("capture_ros_image")
        self.message = None
        self.create_subscription(Image, topic,
                                 lambda m: setattr(self, "message", m), 10)


def main():
    topic, output = sys.argv[1:3]
    rclpy.init()
    node = Capture(topic)
    until = time.monotonic() + 15
    while node.message is None and time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.2)
    msg = node.message
    if msg is None:
        raise RuntimeError("image topic did not publish")
    if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
        raise RuntimeError(f"unsupported encoding {msg.encoding}")
    channels = 4 if "a8" in msg.encoding else 3
    pixels = np.ndarray((msg.height, msg.width, channels), dtype=np.uint8,
                        buffer=msg.data, strides=(msg.step, channels, 1)).copy()
    if msg.encoding.startswith("bgr"):
        pixels = pixels[:, :, [2, 1, 0, 3] if channels == 4 else [2, 1, 0]]
    PILImage.fromarray(pixels).save(output)
    print(msg.width, msg.height, msg.encoding, output)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
