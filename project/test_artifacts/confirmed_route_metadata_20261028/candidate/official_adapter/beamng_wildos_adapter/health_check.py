"""One-shot semantic health gate for official WildOS deployment."""
from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from .health import (camera_info_valid, cloud_valid, goal_valid, graph_stats,
                     graph_valid, image_valid, occupancy_valid, odom_valid,
                     path_valid)


class HealthCheck(Node):
    def __init__(self):
        super().__init__("official_wildos_health_check")
        self.declare_parameter("mode", "graph")
        self.declare_parameter("timeout", 30.0)
        self.declare_parameter("output_path", "/tmp/official_wildos_health.json")
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("graph", "scored", "path", "full"):
            raise ValueError("mode must be graph, scored, path, or full")
        self.results = {}
        self.details = {}
        self.create_subscription(Image, "/spot1/realsense/front/color/image_raw",
                                 lambda m: self.record("image", image_valid(m)), 1)
        self.create_subscription(CameraInfo, "/spot1/realsense/front/color/camera_info",
                                 lambda m: self.record("camera_info", camera_info_valid(m)), 1)
        self.create_subscription(Odometry, "/spot1/odom",
                                 lambda m: self.record("odom", odom_valid(m)), 10)
        self.create_subscription(PointCloud2, "/spot1/ouster/front/points_filtered",
                                 lambda m: self.record("lidar", cloud_valid(m)), 1)
        self.create_subscription(OccupancyGrid, "/spot1/local_map",
                                 lambda m: self.record("occupancy", occupancy_valid(m)), 1)
        self.create_subscription(NavigationGraph, "/spot1/nav_graph",
                                 lambda m: self.record_graph("nav_graph", m, False), 10)
        self.create_subscription(NavigationGraph, "/spot1/scored_nav_graph",
                                 lambda m: self.record_graph("scored_graph", m, True), 10)
        self.create_subscription(Path, "/spot1/graphnav_planner/path",
                                 lambda m: self.record("graph_path", path_valid(m)), 10)
        self.create_subscription(PoseStamped, "/spot1/goal_pose",
                                 lambda m: self.record("path_follower_goal", goal_valid(m)), 10)
        self.create_subscription(Path, "/wildos/planned_path",
                                 lambda m: self.record("nav2_path", len(m.poses) >= 2), 10)

    def record(self, name, valid):
        self.results[name] = bool(valid)

    def record_graph(self, name, message, require_scores):
        stats = graph_stats(message)
        self.details[name] = stats
        self.results[name] = graph_valid(stats, require_scores)

    def required(self):
        groups = {
            "graph": ["image", "camera_info", "odom", "lidar", "occupancy", "nav_graph"],
            "scored": ["scored_graph"],
            "path": ["graph_path", "path_follower_goal", "nav2_path"],
        }
        if self.mode == "full":
            return groups["graph"] + groups["scored"] + groups["path"]
        return groups[self.mode]

    def ready(self):
        return all(self.results.get(name, False) for name in self.required())

    def report(self, timed_out=False):
        required = self.required()
        return {
            "mode": self.mode,
            "ok": self.ready(),
            "timed_out": bool(timed_out),
            "required": required,
            "results": {name: self.results.get(name, False) for name in required},
            "details": self.details,
        }


def main(args=None):
    rclpy.init(args=args)
    node = HealthCheck()
    deadline = time.monotonic() + float(node.get_parameter("timeout").value)
    while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    report = node.report(timed_out=not node.ready())
    text = json.dumps(report, indent=2, sort_keys=True)
    with open(str(node.get_parameter("output_path").value), "w", encoding="utf-8") as handle:
        handle.write(text + "\n")
    print(text)
    ok = report["ok"]
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
