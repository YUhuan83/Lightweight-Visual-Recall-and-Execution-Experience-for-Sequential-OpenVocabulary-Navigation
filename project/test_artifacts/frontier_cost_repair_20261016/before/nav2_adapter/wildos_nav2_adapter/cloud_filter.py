"""Prepare the BeamNG LiDAR cloud for Nav2 costmaps.

Transforms into the robot base frame, removes ground/overhead points using
base-relative height, then voxel-downsamples. Nav2 later transforms the cloud
into odom/map frames, whose absolute z can be far from zero in BeamNG worlds.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from .math3d import matrix_from_quaternion
from .pointcloud import (
    decode_xyz,
    split_costmap_points,
)


class CloudFilter(Node):
    def __init__(self):
        super().__init__("wildos_cloud_filter")
        self.declare_parameter("input_topic", "/vehicles/ego/sensors/front_lidar")
        self.declare_parameter("output_topic", "/wildos/points")
        self.declare_parameter("clearing_topic", "")
        self.declare_parameter("voxel", 0.2)
        self.declare_parameter("max_points", 20000)
        self.declare_parameter("max_range", 40.0)
        self.declare_parameter("min_height", 0.1)
        self.declare_parameter("max_height", 3.0)
        self.declare_parameter("ground_plane_radius", 0.0)
        self.declare_parameter("ground_clearance", 0.25)
        self.declare_parameter("filter_footprint_returns", True)
        self.declare_parameter("footprint_front_extent", 2.3)
        self.declare_parameter("footprint_rear_extent", 2.3)
        self.declare_parameter("footprint_half_width", 1.0)
        self.declare_parameter("output_frame", "base_link")
        self.pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), 10)
        clearing_topic = str(self.get_parameter("clearing_topic").value)
        self.clearing_pub = (self.create_publisher(PointCloud2, clearing_topic, 10)
                             if clearing_topic else None)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value), self.callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("WildOS cloud filter started (x/y/z only)")

    def callback(self, msg: PointCloud2):
        try:
            points = decode_xyz(msg)
        except ValueError as exc:
            self.get_logger().warn(f"dropping cloud: {exc}")
            return
        output_frame = str(self.get_parameter("output_frame").value)
        if msg.header.frame_id != output_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    output_frame, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.2))
            except Exception as exc:
                self.get_logger().warn(f"dropping cloud: TF {output_frame} <- {msg.header.frame_id}: {exc}")
                return
            t = transform.transform.translation
            q = transform.transform.rotation
            rotation = np.asarray(matrix_from_quaternion([q.x, q.y, q.z, q.w]), dtype=float)
            points = points @ rotation.T + np.array([t.x, t.y, t.z], dtype=float)
        limit = float(self.get_parameter("max_range").value)
        if limit > 0:
            radius = np.linalg.norm(points, axis=1)
            points = points[radius <= limit]
        front_extent = float(self.get_parameter("footprint_front_extent").value)
        rear_extent = float(self.get_parameter("footprint_rear_extent").value)
        half_width = float(self.get_parameter("footprint_half_width").value)
        voxel = float(self.get_parameter("voxel").value)
        max_points = int(self.get_parameter("max_points").value)
        # Keep ground returns for clearing. They trace free space up to the
        # first LiDAR hit, but must never mark a ground cell as an obstacle.
        obstacle_points, clearing_points = split_costmap_points(
            points,
            min_height=float(self.get_parameter("min_height").value),
            max_height=float(self.get_parameter("max_height").value),
            filter_footprint=bool(self.get_parameter("filter_footprint_returns").value),
            front_extent=front_extent, rear_extent=rear_extent,
            half_width=half_width, voxel=voxel, max_points=max_points,
            ground_plane_radius=float(self.get_parameter("ground_plane_radius").value),
            ground_clearance=float(self.get_parameter("ground_clearance").value),
        )
        self.pub.publish(self.make_cloud(msg, output_frame, obstacle_points))
        if self.clearing_pub is not None:
            self.clearing_pub.publish(self.make_cloud(msg, output_frame,
                                                      clearing_points))

    @staticmethod
    def make_cloud(msg: PointCloud2, output_frame: str, points: np.ndarray) -> PointCloud2:
        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = output_frame
        out.height = 1
        out.width = int(len(points))
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * out.width
        out.is_dense = True
        out.data = np.asarray(points, dtype="<f4").tobytes(order="C")
        return out


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
