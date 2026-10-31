"""Record the whole run as a single side-by-side MP4.

Left: BeamNG front camera. Right: WildOS frontier/traversability visualization.
A top bar shows wall-clock time and the vehicle pose. Read-only: it publishes
nothing and commands nothing.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TARGET_H = 480
CAM_W = 640
MODEL_W = 560


def image_to_bgr(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "mono8":
        return cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    if msg.encoding in ("rgb8", "bgr8"):
        row = arr.reshape(msg.height, msg.step)
        im = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(im, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else im
    return None


def fit(img, w, h):
    if img is None:
        return np.zeros((h, w, 3), np.uint8)
    return cv2.resize(img, (w, h))


class Recorder(Node):
    def __init__(self, out, fps, max_seconds):
        super().__init__("wildos_video_recorder")
        self.out = out
        self.fps = fps
        self.max_seconds = max_seconds
        self.cam = None
        self.model = None
        self.pose = None
        self.frames = 0
        self.writer = None
        self.t0 = time.time()
        self.qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/spot1/realsense/front/color/image_raw", self.on_cam, self.qos)
        self.create_subscription(Image, "/spot1/model_visualization", self.on_model, self.qos)
        self.create_subscription(Odometry, "/spot1/odom", self.on_odom, 10)
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f"video recorder -> {out} at {fps} fps")

    def on_cam(self, msg):
        try:
            self.cam = image_to_bgr(msg)
        except Exception:
            pass

    def on_model(self, msg):
        try:
            self.model = image_to_bgr(msg)
        except Exception:
            pass

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def _compose(self):
        left = fit(self.cam, CAM_W, TARGET_H)
        right = fit(self.model, MODEL_W, TARGET_H)
        body = np.hstack([left, right])
        bar = np.zeros((34, body.shape[1], 3), np.uint8)
        cv2.putText(bar, "front camera", (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, "WildOS: image / frontier / traversability / frontier nodes",
                    (CAM_W + 8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1, cv2.LINE_AA)
        stamp = np.zeros((30, body.shape[1], 3), np.uint8)
        txt = time.strftime("%Y-%m-%d %H:%M:%S")
        if self.pose is not None:
            txt += "  pose x=%.2f y=%.2f z=%.2f" % self.pose
        cv2.putText(stamp, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        return np.vstack([bar, stamp, body])

    def tick(self):
        if self.cam is None and self.model is None:
            return
        frame = self._compose()
        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.out, fourcc, self.fps, (w, h))
            if not self.writer.isOpened():
                self.get_logger().error("mp4 writer failed to open")
        self.writer.write(frame)
        self.frames += 1
        if self.frames % (self.fps * 10) == 0:
            self.get_logger().info(f"recorded {self.frames} frames ({self.frames/self.fps:.0f}s)")
        if self.max_seconds > 0 and (time.time() - self.t0) > self.max_seconds:
            self.get_logger().info("max seconds reached, closing")
            self.close_and_exit()

    def close_and_exit(self):
        if self.writer is not None:
            self.writer.release()
        raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    rclpy.init()
    node = Recorder(args.out, args.fps, args.seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.writer is not None:
            node.writer.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
