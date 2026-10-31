"""Publish BeamNG data using the exact topic/frame contracts of official WildOS."""
from __future__ import annotations

import math
import time
import numpy as np
import rclpy
from beamng_msgs.msg import StateSensor
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                     TransformListener)

from .geometry import (body_rotation, matrix_quaternion, optical_rotation,
                       quaternion_matrix, xyz, shifted_body_position,
                       shifted_body_velocity, rotation_velocity)
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from .mapping import occupancy_from_points
from .pointcloud import decode_xyz, limit_points
from .measurement_clock import MeasurementClock


class Adapter(Node):
    def __init__(self):
        super().__init__("beamng_wildos_adapter")
        self.state = None
        self.state_ros_stamp = None
        self.previous_rotation = None
        self.previous_source_ns = None
        self.angular_world = np.zeros(3)
        self.declare_parameter("base_offset", list(SBR_BASE_OFFSET))
        self.base_offset = np.asarray(self.get_parameter("base_offset").value, dtype=float)
        shifted_body_position(np.zeros(3), np.eye(3), self.base_offset)
        self.measurement_clock = MeasurementClock()
        self.declare_parameter("align_sensor_clock_to_ros", True)
        self.lidar_frames = 0
        self.last_lidar_skew_warning = 0.0
        self.last_camera_tf_warning = 0.0
        self.declare_parameter("max_pose_lidar_skew", 0.1)
        self.declare_parameter("ground_plane_radius", 8.0)
        self.declare_parameter("ground_clearance", 0.25)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/spot1/odom", 10)
        self.image_pub = self.create_publisher(Image, "/spot1/realsense/front/color/image_raw", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/spot1/realsense/front/color/camera_info", 1)
        self.lidar_pub = self.create_publisher(PointCloud2, "/spot1/ouster/front/points_filtered", 1)
        self.map_pub = self.create_publisher(OccupancyGrid, "/spot1/local_map", 1)
        self.create_subscription(StateSensor, "/vehicles/ego/sensors/state", self.state_cb, 10)
        self.create_subscription(Image, "/vehicles/ego/sensors/front_cam/colour", self.image_cb, 1)
        self.create_subscription(PointCloud2, "/vehicles/ego/sensors/front_lidar", self.lidar_cb, 1)
        self.publish_static_lidar_tf()

    def publish_static_lidar_tf(self):
        tf=TransformStamped(); tf.header.stamp=self.get_clock().now().to_msg()
        tf.header.frame_id="spot1/base_link"; tf.child_frame_id="spot1/ouster/front/os_lidar"
        # Preserve the existing virtual cloud frame at the StateSensor origin.
        # Moving base_link to the rear axle must not translate world obstacles.
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, -self.base_offset)
        tf.transform.rotation.w=1.0
        self.static_tf_pub.sendTransform(tf)

    def state_cb(self, msg):
        stamp = msg.header.stamp
        if (self.get_parameter("align_sensor_clock_to_ros").value
                and not self.get_parameter("use_sim_time").value):
            mapped = self.measurement_clock.update(
                stamp.sec * 1_000_000_000 + stamp.nanosec,
                self.get_clock().now().nanoseconds)
            if mapped is None:
                return
            stamp = rclpy.time.Time(nanoseconds=mapped).to_msg()
        rotation = body_rotation(msg.dir, msg.up)
        source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.previous_source_ns is None or source_ns > self.previous_source_ns:
            dt = (source_ns - self.previous_source_ns) * 1e-9 if self.previous_source_ns is not None else 0.0
            self.angular_world = rotation_velocity(self.previous_rotation, rotation, dt)
            self.previous_rotation = rotation
            self.previous_source_ns = source_ns
        position = shifted_body_position(xyz(msg.position), rotation, self.base_offset)
        q = matrix_quaternion(rotation)
        odom = Odometry(); odom.header.stamp = stamp
        odom.header.frame_id = "spot1/odom"; odom.child_frame_id = "spot1/base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = q[0], q[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2], q[3]
        velocity = shifted_body_velocity(xyz(msg.velocity), rotation, self.base_offset, self.angular_world)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, velocity)
        angular = self.angular_world @ rotation
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, angular)
        self.odom_pub.publish(odom)
        self.publish_tf("spot1/odom", "spot1/base_link", position, q, stamp)
        # Nav2's Smac paths live on z=0. RPP transforms them in 3D before
        # discarding z; a ~52 m altitude gap times body pitch/roll otherwise
        # creates ~1 m of false longitudinal/lateral error and early reversing.
        # Keep the physical 3D frame for sensors and a yaw-only rear axle for
        # planar navigation. Both frames have exactly the same world XY.
        heading = math.atan2(rotation[1,0], rotation[0,0])
        self.publish_tf("spot1/odom", "spot1/nav_base_link", [position[0], position[1], 0.0],
                        [0.0, 0.0, math.sin(heading/2), math.cos(heading/2)], stamp)
        self.state = msg
        self.state_ros_stamp = stamp

    def measurement_stamp(self, stamp):
        if (not self.get_parameter("align_sensor_clock_to_ros").value
                or self.get_parameter("use_sim_time").value):
            return stamp
        mapped = self.measurement_clock.map(stamp.sec * 1_000_000_000 + stamp.nanosec)
        return rclpy.time.Time(nanoseconds=mapped).to_msg() if mapped is not None else None

    def image_cb(self, msg):
        if msg.encoding != "rgb8" or (msg.width, msg.height) != (960, 640): return
        stamp = self.measurement_stamp(msg.header.stamp)
        if stamp is None: return
        if not self.publish_sensor_tf("ego_front_cam", "spot1/realsense/front_color_optical_frame",
                                      msg.header.stamp, optical=True, output_stamp=stamp):
            return
        out = Image(); out.header.stamp = stamp; out.header.frame_id = "spot1/realsense/front_color_optical_frame"
        out.height, out.width, out.encoding = msg.height, msg.width, msg.encoding
        out.is_bigendian, out.step, out.data = msg.is_bigendian, msg.step, msg.data
        self.image_pub.publish(out)
        info = CameraInfo(); info.header = out.header; info.width=960; info.height=640
        fy = 320.0 / math.tan(math.radians(30.0)); fx=fy; cx=480.0; cy=320.0
        info.distortion_model="plumb_bob"; info.d=[0.0]*5
        info.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.]
        info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
        self.info_pub.publish(info)

    def lidar_cb(self, msg):
        state = self.state
        if state is None: return
        state_stamp = (state.header.stamp.sec
                       + state.header.stamp.nanosec * 1e-9)
        lidar_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        skew = state_stamp - lidar_stamp
        now = time.monotonic()
        max_skew = float(self.get_parameter("max_pose_lidar_skew").value)
        if abs(skew) > max_skew:
            if now - self.last_lidar_skew_warning > 5.0:
                self.get_logger().warning(
                    f"dropping LiDAR frame: body-pose timestamp differs by "
                    f"{skew:+.3f}s (limit {max_skew:.3f}s)")
                self.last_lidar_skew_warning = now
            return
        try:
            world_points = decode_xyz(msg)
        except ValueError as exc:
            self.get_logger().warn(f"dropping lidar decode: {exc}")
            return
        world_points = limit_points(world_points, 30000)
        body = body_rotation(state.dir, state.up)
        points = (world_points - xyz(state.position)) @ body
        points = points[np.linalg.norm(points,axis=1) <= 40.0]
        # The world cloud is converted with this state pose, so downstream TF
        # must use the same pose timestamp when mapping it back into odom.
        out=PointCloud2(); out.header.stamp=self.state_ros_stamp
        out.header.frame_id="spot1/ouster/front/os_lidar"
        out.height=1; out.width=len(points); out.is_bigendian=False; out.point_step=12
        out.row_step=12*len(points); out.is_dense=True
        out.fields=[PointField(name="x",offset=0,datatype=PointField.FLOAT32,count=1),
                    PointField(name="y",offset=4,datatype=PointField.FLOAT32,count=1),
                    PointField(name="z",offset=8,datatype=PointField.FLOAT32,count=1)]
        out.data=np.asarray(points,dtype="<f4").tobytes(); self.lidar_pub.publish(out)
        self.publish_occupancy(points - self.base_offset, self.state_ros_stamp)
        self.lidar_frames += 1
        if self.lidar_frames == 1:
            self.get_logger().info(
                f"published first official lidar frame with {len(points)} points and local occupancy")

    def publish_sensor_tf(self, source, child, stamp, optical=False, output_stamp=None):
        try:
            tf=self.tf_buffer.lookup_transform("map", source,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_camera_tf_warning > 5.0:
                self.get_logger().warning(
                    f"dropping camera frame: TF map <- {source} unavailable at "
                    f"measurement time: {exc}")
                self.last_camera_tf_warning = now
            return False
        q=tf.transform.rotation; rotation=quaternion_matrix([q.x,q.y,q.z,q.w])
        if optical: rotation=optical_rotation(rotation)
        quat=matrix_quaternion(rotation); t=tf.transform.translation
        self.publish_tf("spot1/odom", child, [t.x,t.y,t.z], quat, output_stamp or stamp)
        return True

    def publish_tf(self, parent, child, translation, quat, stamp):
        tf=TransformStamped(); tf.header.stamp=stamp; tf.header.frame_id=parent; tf.child_frame_id=child
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,translation)
        tf.transform.rotation.x,tf.transform.rotation.y=quat[0],quat[1]
        tf.transform.rotation.z,tf.transform.rotation.w=quat[2],quat[3]
        self.tf_pub.sendTransform(tf)

    def publish_occupancy(self, points, stamp):
        res=0.5; width=80; height=80
        grid=occupancy_from_points(points, res, width, height,
            ground_plane_radius=float(self.get_parameter("ground_plane_radius").value),
            ground_clearance=float(self.get_parameter("ground_clearance").value),
            sensor_origin_xy=-self.base_offset[:2])
        msg=OccupancyGrid(); msg.header.stamp=stamp; msg.header.frame_id="spot1/base_link"
        msg.info.resolution=res; msg.info.width=width; msg.info.height=height
        msg.info.origin.position.x=-width*res/2; msg.info.origin.position.y=-height*res/2
        msg.info.origin.orientation.w=1.0; msg.data=grid.ravel().tolist(); self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node=Adapter(); executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
