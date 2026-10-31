"""Read-only ROS probe for costmap cells covered by the vehicle footprint."""
import math
import time
import struct
import numpy as np

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Probe(Node):
    def __init__(self):
        super().__init__("probe_costmap_footprint")
        self.grid = self.odom = self.cloud = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: setattr(self, "grid", m), 10)
        self.create_subscription(Odometry, "/spot1/odom",
                                 lambda m: setattr(self, "odom", m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/wildos/points",
                                 lambda m: setattr(self, "cloud", m), 10)


def cell(grid, x, y):
    info = grid.info
    i = math.floor((x - info.origin.position.x) / info.resolution)
    j = math.floor((y - info.origin.position.y) / info.resolution)
    if 0 <= i < info.width and 0 <= j < info.height:
        return grid.data[j * info.width + i]
    return 101


def main():
    rclpy.init()
    node = Probe()
    end = time.monotonic() + 15
    while time.monotonic() < end and (node.grid is None or node.odom is None or node.cloud is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.grid is None or node.odom is None:
        print("missing costmap or odom", flush=True)
        return
    pose = node.odom.pose.pose
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    x, y = pose.position.x, pose.position.y
    grid = node.grid
    counts = {"free": 0, "inflated": 0, "inscribed": 0, "lethal": 0,
              "unknown": 0, "outside": 0}
    obstacles = []
    # Configured footprint spans 2.3 m fore/aft and 1.0 m either side.
    for i in range(-23, 24):
        for j in range(-10, 11):
            a, b = i * .1, j * .1
            wx = x + math.cos(yaw) * a - math.sin(yaw) * b
            wy = y + math.sin(yaw) * a + math.cos(yaw) * b
            cost = cell(grid, wx, wy)
            key = ("outside" if cost == 101 else "unknown" if cost < 0 else
                   "lethal" if cost >= 100 else "inscribed" if cost >= 99 else
                   "inflated" if cost > 0 else "free")
            counts[key] += 1
            if cost >= 99 and len(obstacles) < 10:
                obstacles.append((round(a, 1), round(b, 1), cost))
    print("pose", round(x, 3), round(y, 3), "yaw", round(yaw, 3),
          "center_cost", cell(grid, x, y), flush=True)
    print("footprint", counts, "bad_samples", obstacles, flush=True)
    nearby = []
    for i in range(-60, 61):
        for j in range(-60, 61):
            a, b = i * .1, j * .1
            wx = x + math.cos(yaw) * a - math.sin(yaw) * b
            wy = y + math.sin(yaw) * a + math.cos(yaw) * b
            cost = cell(grid, wx, wy)
            if cost >= 100 and cost < 101:
                nearby.append((math.hypot(a, b), round(a, 1), round(b, 1), cost))
    nearby.sort()
    print("nearest_lethal", nearby[:16], flush=True)
    if node.cloud is not None and node.cloud.point_step == 12:
        points = list(struct.iter_unpack("<fff", node.cloud.data))
        for label, target in [("front_left", (1.8, 1.6)),
                              ("forward_left", (4.5, .7)),
                              ("forward_right", (4., -2.2))]:
            hits = []
            for a, b, z in points:
                d = math.hypot(a - target[0], b - target[1])
                if d < 1.0:
                    hits.append((round(d, 2), round(a, 2), round(b, 2), round(z, 2)))
            hits.sort()
            heights = [row[3] for row in hits]
            bins = [sum(lo <= value < hi for value in heights)
                    for lo, hi in [(0., .2), (.2, .4), (.4, .7), (.7, 1.), (1., 3.)]]
            print(label, len(hits), "height_bins", bins, "nearest", hits[:8], flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
