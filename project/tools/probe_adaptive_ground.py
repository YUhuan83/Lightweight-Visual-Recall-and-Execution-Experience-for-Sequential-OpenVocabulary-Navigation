"""Read-only comparison of legacy and slope-aware obstacle classifications."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from wildos_nav2_adapter.pointcloud import decode_xyz, local_ground_plane, split_costmap_points


class Probe(Node):
    def __init__(self):
        super().__init__("probe_adaptive_ground")
        self.cloud = None
        self.create_subscription(PointCloud2, "/wildos/clearing_points",
                                 lambda m: setattr(self, "cloud", m), 10)


def classify(points, radius):
    marked, _ = split_costmap_points(
        points, min_height=.1, max_height=3., filter_footprint=False,
        front_extent=2.3, rear_extent=2.3, half_width=1.,
        voxel=.2, max_points=20000,
        ground_plane_radius=radius, ground_clearance=.25)
    close = (np.hypot(marked[:, 0] - 1.8, marked[:, 1] - 1.6) < 1.5)
    return len(marked), int(close.sum())


def main():
    rclpy.init()
    node = Probe()
    end = time.monotonic() + 15
    while node.cloud is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.2)
    if node.cloud is None:
        print("missing cloud", flush=True)
        return
    points = decode_xyz(node.cloud)
    plane = local_ground_plane(points, 8.)
    marked_only = points[(points[:, 2] >= .1) & (points[:, 2] <= 3.)]
    marked_plane = local_ground_plane(marked_only, 8.)
    print("points", len(points), "plane", None if plane is None else plane.tolist(),
          "marked_plane", None if marked_plane is None else marked_plane.tolist(),
          "legacy", classify(points, 0.), "adaptive", classify(points, 8.), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
