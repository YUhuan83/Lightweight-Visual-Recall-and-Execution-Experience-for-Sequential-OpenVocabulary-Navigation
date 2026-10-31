"""Save a short ROS data snapshot for one East Coast spawn region."""

import argparse
import json
import statistics
import time
from pathlib import Path

import rclpy
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry, Path as NavPath
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


TOPICS = {
    "image": (Image, "/spot1/realsense/front/color/image_raw"),
    "lidar": (PointCloud2, "/spot1/ouster/front/points_filtered"),
    "odom": (Odometry, "/spot1/odom"),
    "graph": (NavigationGraph, "/spot1/nav_graph"),
    "scored_graph": (NavigationGraph, "/spot1/scored_nav_graph"),
    "path": (NavPath, "/spot1/graphnav_planner/path"),
}


class Snapshot(Node):
    def __init__(self, output):
        super().__init__("east_coast_snapshot")
        self.output = output
        self.received = {name: [] for name in TOPICS}
        self.details = {}
        for name, (message_type, topic) in TOPICS.items():
            self.create_subscription(
                message_type, topic,
                lambda message, key=name: self.record(key, message), 10,
            )

    def record(self, name, message):
        self.received[name].append(time.monotonic())
        if name == "image":
            self.details["image"] = {
                "width": message.width, "height": message.height,
                "encoding": message.encoding,
            }
            image_path = self.output / "camera.png"
            if not image_path.exists() and message.encoding in ("rgb8", "bgr8"):
                raw = memoryview(message.data)
                rows = b"".join(
                    raw[row * message.step:row * message.step + message.width * 3]
                    for row in range(message.height)
                )
                image = PILImage.frombytes("RGB", (message.width, message.height), rows)
                if message.encoding == "bgr8":
                    image = PILImage.merge("RGB", image.split()[::-1])
                image.save(image_path)
        elif name == "lidar":
            self.details["lidar"] = {
                "points": message.width * message.height,
                "frame": message.header.frame_id,
            }
        elif name == "odom":
            p = message.pose.pose.position
            self.details["odom"] = {
                "position": [p.x, p.y, p.z],
                "frame": message.header.frame_id,
            }
        elif name in ("graph", "scored_graph"):
            self.details[name] = {
                "nodes": len(message.nodes), "edges": len(message.edges),
                "frame": message.header.frame_id,
                "frontiers": sum(
                    bool(node.trav_properties and node.trav_properties[0].is_frontier)
                    for node in message.nodes
                ),
            }
        elif name == "path":
            self.details["path"] = {"poses": len(message.poses)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Snapshot(args.output)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = {"duration_seconds": args.seconds, "topics": {},
                  "details": node.details}
        for name, times in node.received.items():
            intervals = [later - earlier for earlier, later in zip(times, times[1:])]
            report["topics"][name] = {
                "count": len(times),
                "median_hz": (1.0 / statistics.median(intervals) if intervals else None),
            }
        (args.output / "snapshot.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    