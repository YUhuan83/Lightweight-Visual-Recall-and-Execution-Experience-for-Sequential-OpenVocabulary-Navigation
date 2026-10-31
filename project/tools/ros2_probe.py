"""Standalone rclpy probe: counts sensor/odom messages and TF without ros2 CLI."""
from __future__ import annotations

import sys
import time

import rclpy
from beamng_msgs.msg import StateSensor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformListener


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = Node("wildos_probe")
    counts = {"state": 0, "camera": 0, "lidar": 0, "odom": 0}
    last = {"odom": None}
    node.create_subscription(
        StateSensor, "/vehicles/ego/sensors/state",
        lambda _m: counts.__setitem__("state", counts["state"] + 1), 10)
    node.create_subscription(
        Image, "/vehicles/ego/sensors/front_cam/colour",
        lambda _m: counts.__setitem__("camera", counts["camera"] + 1), 10)
    node.create_subscription(
        PointCloud2, "/vehicles/ego/sensors/front_lidar",
        lambda _m: counts.__setitem__("lidar", counts["lidar"] + 1), 10)
    node.create_subscription(
        Odometry, "/wildos/odom", lambda m: (
            counts.__setitem__("odom", counts["odom"] + 1),
            last.__setitem__("odom", m.pose.pose)), 10)
    buffer = Buffer()
    TransformListener(buffer, node)
    start = time.time()
    tf_ok = {"map->ego": False, "odom->base_link": False}
    while time.time() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not tf_ok["map->ego"]:
            tf_ok["map->ego"] = buffer.can_transform("map", "ego", rclpy.time.Time())
        if not tf_ok["odom->base_link"]:
            tf_ok["odom->base_link"] = buffer.can_transform(
                "odom", "base_link", rclpy.time.Time())
    elapsed = max(time.time() - start, 1e-6)
    print("counts", counts)
    print("rates", {k: round(v / elapsed, 2) for k, v in counts.items()})
    print("tf", tf_ok)
    pose = last["odom"]
    if pose is not None:
        print("odom_pose x=%.2f y=%.2f z=%.2f" % (
            pose.position.x, pose.position.y, pose.position.z))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
