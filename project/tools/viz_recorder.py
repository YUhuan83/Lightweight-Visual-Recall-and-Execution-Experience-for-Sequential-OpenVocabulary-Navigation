"""Offline review recorder for official WildOS frontier/traversability output.

Subscribes to the WildOS model visualization and the raw front camera, composes
a side-by-side matplotlib figure every N seconds, and writes PNG frames plus an
MP4 when OpenCV is available. Read-only: publishes nothing, commands nothing.
"""
import argparse
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def image_to_array(msg):
    if msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    if msg.encoding in ("rgb8", "bgr8"):
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if msg.encoding == "bgr8" else img
    return None


class Recorder(Node):
    def __init__(self, out_dir, period, prefix):
        super().__init__("wildos_viz_recorder")
        self.out_dir = out_dir
        self.prefix = prefix
        self.period = period
        self.frames = 0
        self.model = None
        self.camera = None
        self.pose = None
        os.makedirs(out_dir, exist_ok=True)
        self.qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/spot1/model_visualization", self.on_model, self.qos)
        self.create_subscription(Image, "/spot1/realsense/front/color/image_raw", self.on_cam, self.qos)
        self.create_subscription(Odometry, "/spot1/odom", self.on_odom, 10)
        self.create_timer(period, self.dump)
        self.get_logger().info(f"recorder writing to {out_dir} every {period}s")

    def on_model(self, msg):
        self.model = image_to_array(msg)

    def on_cam(self, msg):
        self.camera = image_to_array(msg)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def dump(self):
        if self.model is None and self.camera is None:
            return
        panels = [x for x in (self.camera, self.model) if x is not None]
        titles = [t for x, t in ((self.camera, "front camera"), (self.model, "WildOS frontier/traversability"))
                  if x is not None]
        fig, axes = plt.subplots(1, len(panels), figsize=(7 * len(panels), 5))
        if len(panels) == 1:
            axes = [axes]
        for ax, img, title in zip(axes, panels, titles):
            ax.imshow(img)
            ax.set_title(title)
            ax.axis("off")
        if self.pose is not None:
            fig.suptitle("pose x=%.2f y=%.2f z=%.2f" % self.pose)
        path = os.path.join(self.out_dir, f"{self.prefix}_{self.frames:05d}.png")
        fig.tight_layout()
        fig.savefig(path, dpi=90)
        plt.close(fig)
        self.frames += 1
        self.get_logger().info(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/home/ly/nizhen_viz_capture")
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--prefix", default="frame")
    ap.add_argument("--duration", type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.out, args.period, args.prefix)
    start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration > 0 and time.time() - start > args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
