"""Publish BeamNG data using the exact topic/frame contracts of official WildOS."""
from __future__ import annotations

import json
import math
import time
from collections import deque
import numpy as np
import rclpy
from beamng_msgs.msg import StateSensor
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .geometry import (body_rotation, matrix_quaternion, optical_rotation,
                       quaternion_matrix, xyz, shifted_body_position,
                       shifted_body_velocity, rotation_velocity)
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.sensor_geometry import (SBR_CAMERA_OFFSET, SBR_LIDAR_OFFSET,
                                                geometry_restart_reason)
from rcl_interfaces.msg import SetParametersResult
from .mapping import occupancy_from_points
from .terrain import terrain_layers, grid_map_values
from .pointcloud import decode_xyz, limit_points
from .measurement_clock import MeasurementClock, closest_state_sample


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
        # BeamNG's ego_front_cam frame is attached to the vehicle but the
        # BeamNG ROS bridge does not publish its mounting transform.  The
        # Actual mounted pose differs from the nominal scenario mount. Use the
        # measured rig calibration; keep it configurable for other rigs.
        self.declare_parameter("camera_offset", list(SBR_CAMERA_OFFSET))
        self.camera_offset = np.asarray(self.get_parameter("camera_offset").value,
                                        dtype=float)
        if self.camera_offset.shape != (3,) or not np.isfinite(self.camera_offset).all():
            raise ValueError("camera_offset must contain three finite values")
        self.measurement_clock = MeasurementClock()
        self.state_history = deque(maxlen=100)
        self.pending_images = deque(maxlen=8)
        self.pending_lidars = deque(maxlen=8)
        self.declare_parameter("align_sensor_clock_to_ros", True)
        self.lidar_frames = 0
        self.last_lidar_skew_warning = 0.0
        self.last_terrain_stamp_ns = None
        self.declare_parameter("max_pose_lidar_skew", 0.1)
        self.declare_parameter("ground_plane_radius", 8.0)
        self.declare_parameter("ground_clearance", 0.25)
        # Height above the local road plane that must be treated as an
        # obstacle. This is deliberately separate from the base-frame
        # min_height gate so a low kerb/rock cannot disappear before marking.
        self.declare_parameter("obstacle_min_height", 0.10)
        self.declare_parameter("ground_max_slope", 0.35)
        self.declare_parameter("carrier_model", "ackermann")
        self.carrier_model = str(self.get_parameter("carrier_model").value).strip().lower()
        if self.carrier_model not in ("ideal_point", "ackermann"):
            raise ValueError("carrier_model must be ideal_point or ackermann")
        self.declare_parameter("lidar_offset", list(SBR_LIDAR_OFFSET)
                               if self.carrier_model == "ackermann" else list(-self.base_offset))
        self.lidar_offset = np.asarray(self.get_parameter("lidar_offset").value, dtype=float)
        if self.lidar_offset.shape != (3,) or not np.isfinite(self.lidar_offset).all():
            raise ValueError("lidar_offset must contain three finite values")
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/spot1/odom", 10)
        self.timing_pub = self.create_publisher(String, "/spot1/measurement_timing", 10)
        self.image_pub = self.create_publisher(Image, "/spot1/realsense/front/color/image_raw", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/spot1/realsense/front/color/camera_info", 1)
        self.lidar_pub = self.create_publisher(PointCloud2, "/spot1/ouster/front/points_filtered", 1)
        self.map_pub = self.create_publisher(OccupancyGrid, "/spot1/local_map", 1)
        self.terrain_pub = self.create_publisher(GridMap, "/spot1/local_terrain", 1)
        self.create_subscription(StateSensor, "/vehicles/ego/sensors/state", self.state_cb, 10)
        self.create_subscription(Image, "/vehicles/ego/sensors/front_cam/colour", self.image_cb, 1)
        self.create_subscription(PointCloud2, "/vehicles/ego/sensors/front_lidar", self.lidar_cb, 1)
        self.publish_static_lidar_tf()
        self.create_timer(0.01, self.flush_pending_sensors)
        self.add_on_set_parameters_callback(self.on_parameters)

    def on_parameters(self, parameters):
        reason = geometry_restart_reason(parameters, self.get_parameter)
        return SetParametersResult(successful=reason is None, reason=reason or '')

    def publish_measurement_timing(self, sensor, **details):
        """Publish small timestamp-only records without copying sensor payloads."""
        publisher = getattr(self, "timing_pub", None)
        if publisher is None:
            return
        data = dict(schema_version=1, sensor=sensor,
                    adapter_rx_ros_ns=self.get_clock().now().nanoseconds, **details)
        publisher.publish(String(data=json.dumps(data, separators=(",", ":"))))

    def measurement_output_clock(self):
        return ("ros" if self.get_parameter("align_sensor_clock_to_ros").value
                and not self.get_parameter("use_sim_time").value else "beamng")

    def publish_static_lidar_tf(self):
        tf=TransformStamped(); tf.header.stamp=self.get_clock().now().to_msg()
        tf.header.frame_id="spot1/base_link"; tf.child_frame_id="spot1/ouster/front/os_lidar"
        # Preserve the existing virtual cloud frame at the StateSensor origin.
        # Moving base_link to the rear axle must not translate world obstacles.
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, -self.base_offset)
        tf.transform.rotation.w=1.0
        ray = TransformStamped(); ray.header.stamp = tf.header.stamp
        ray.header.frame_id = "spot1/base_link"; ray.child_frame_id = "spot1/lidar_ray_origin"
        ray.transform.translation.x, ray.transform.translation.y, ray.transform.translation.z = map(float, self.lidar_offset)
        ray.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform([tf, ray])

    def state_cb(self, msg):
        stamp = msg.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        received_ros_ns = self.get_clock().now().nanoseconds
        clock_reset = False
        if (self.get_parameter("align_sensor_clock_to_ros").value
                and not self.get_parameter("use_sim_time").value):
            mapped = self.measurement_clock.update(
                source_ns, received_ros_ns)
            if mapped is None:
                self.publish_measurement_timing(
                    "state", source_clock="beamng", source_stamp_ns=source_ns,
                    output_clock="ros", mapped_stamp_ns=None,
                    status="clock_mapping_rejected")
                return
            clock_reset = self.measurement_clock.last_update_reset
            if self.measurement_clock.last_update_reset:
                # The source simulator restarted its clock. Do not estimate
                # angular velocity across two unrelated time epochs.
                self.previous_rotation = None
                self.previous_source_ns = None
                self.angular_world.fill(0.0)
                self.state_history.clear()
                self.pending_images.clear()
                self.pending_lidars.clear()
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
        self.publish_tf("spot1/odom", "spot1/terrain_base_link", position,
                        [0.0, 0.0, math.sin(heading/2), math.cos(heading/2)], stamp)
        self.state = msg
        self.state_ros_stamp = stamp
        self.state_history.append((source_ns, stamp, msg))
        self.publish_measurement_timing(
            "state", source_clock="beamng",
            output_clock=self.measurement_output_clock(),
            source_stamp_ns=source_ns,
            mapped_stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            clock_offset_ns=(self.measurement_clock.offset
                             if self.measurement_clock.offset is not None else None),
            clock_reset=clock_reset, status="published")

    def measurement_stamp(self, stamp):
        if (not self.get_parameter("align_sensor_clock_to_ros").value
                or self.get_parameter("use_sim_time").value):
            return stamp
        mapped = self.measurement_clock.map(stamp.sec * 1_000_000_000 + stamp.nanosec)
        return rclpy.time.Time(nanoseconds=mapped).to_msg() if mapped is not None else None

    def image_cb(self, msg):
        self.pending_images.append((msg, time.monotonic()))

    def process_image(self, msg, state, stamp):
        source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg.encoding != "rgb8" or (msg.width, msg.height) != (960, 640):
            self.publish_measurement_timing(
                "camera", source_clock="beamng", source_stamp_ns=source_ns,
                output_clock=self.measurement_output_clock(), mapped_stamp_ns=None,
                status="dropped_invalid_format",
                width=msg.width, height=msg.height, encoding=msg.encoding)
            return
        # A camera is rigidly mounted to the sensor carrier. Derive its pose
        # from the state sample matched to this image instead of looking up a
        # transform that may not have arrived yet in the TF listener.
        body = body_rotation(state.dir, state.up)
        camera_position = shifted_body_position(
            xyz(state.position), body, self.base_offset + self.camera_offset)
        camera_rotation = optical_rotation(body)
        self.publish_tf("spot1/odom", "spot1/realsense/front_color_optical_frame",
                        camera_position, matrix_quaternion(camera_rotation), stamp)
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
        self.publish_measurement_timing(
            "camera", source_clock="beamng", source_stamp_ns=source_ns,
            output_clock=self.measurement_output_clock(),
            mapped_stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            status="published", width=msg.width, height=msg.height,
            encoding=msg.encoding)

    def lidar_cb(self, msg):
        self.pending_lidars.append((msg, time.monotonic()))

    def process_lidar(self, msg, state, state_ros_stamp):
        source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        state_source_ns = (state.header.stamp.sec * 1_000_000_000
                           + state.header.stamp.nanosec)
        skew_ns = state_source_ns - source_ns
        skew = skew_ns * 1e-9
        now = time.monotonic()
        max_skew = float(self.get_parameter("max_pose_lidar_skew").value)
        if abs(skew) > max_skew:
            self.publish_measurement_timing(
                "lidar", source_clock="beamng", source_stamp_ns=source_ns,
                state_source_stamp_ns=state_source_ns,
                output_clock=self.measurement_output_clock(),
                mapped_stamp_ns=None, skew_ns=skew_ns,
                max_skew_ns=round(max_skew * 1_000_000_000),
                status="dropped_pose_skew")
            if now - self.last_lidar_skew_warning > 5.0:
                self.get_logger().warning(
                    f"dropping LiDAR frame: body-pose timestamp differs by "
                    f"{skew:+.3f}s (limit {max_skew:.3f}s)")
                self.last_lidar_skew_warning = now
            return
        try:
            world_points = decode_xyz(msg)
        except ValueError as exc:
            self.publish_measurement_timing(
                "lidar", source_clock="beamng", source_stamp_ns=source_ns,
                output_clock=self.measurement_output_clock(), mapped_stamp_ns=None,
                status="dropped_decode_error", error=str(exc))
            self.get_logger().warn(f"dropping lidar decode: {exc}")
            return
        world_points = limit_points(world_points, 30000)
        body = body_rotation(state.dir, state.up)
        points = (world_points - xyz(state.position)) @ body
        in_range = np.linalg.norm(points, axis=1) <= 40.0
        # Terrain must use the same accepted returns as occupancy. Finite
        # out-of-range sentinel coordinates can overflow its float32 filters.
        world_points = world_points[in_range]
        points = points[in_range]
        # The world cloud is converted with this state pose, so downstream TF
        # must use the same pose timestamp when mapping it back into odom.
        out=PointCloud2(); out.header.stamp=state_ros_stamp
        out.header.frame_id="spot1/ouster/front/os_lidar"
        out.height=1; out.width=len(points); out.is_bigendian=False; out.point_step=12
        out.row_step=12*len(points); out.is_dense=True
        out.fields=[PointField(name="x",offset=0,datatype=PointField.FLOAT32,count=1),
                    PointField(name="y",offset=4,datatype=PointField.FLOAT32,count=1),
                    PointField(name="z",offset=8,datatype=PointField.FLOAT32,count=1)]
        out.data=np.asarray(points,dtype="<f4").tobytes(); self.lidar_pub.publish(out)
        self.publish_occupancy(points - self.base_offset, state_ros_stamp)
        terrain_ns = state_ros_stamp.sec*1_000_000_000+state_ros_stamp.nanosec
        if self.last_terrain_stamp_ns is None or terrain_ns-self.last_terrain_stamp_ns >= 500_000_000:
            position = shifted_body_position(xyz(state.position), body, self.base_offset)
            heading = math.atan2(body[1, 0], body[0, 0])
            level = np.array([[math.cos(heading), -math.sin(heading), 0.],
                              [math.sin(heading), math.cos(heading), 0.], [0., 0., 1.]])
            # Use world Z, not pitched body Z: a tilted car must still measure
            # an actual slope. Keep the exact acquisition pose and timestamp.
            terrain_points = (world_points-position) @ level
            sensor_position = shifted_body_position(
                xyz(state.position), body, self.base_offset + self.lidar_offset)
            origin = (sensor_position-position) @ level
            self.publish_terrain(terrain_points, origin[:2], state_ros_stamp)
            self.last_terrain_stamp_ns = terrain_ns
        self.lidar_frames += 1
        if self.lidar_frames == 1:
            self.get_logger().info(
                f"published first official lidar frame with {len(points)} points and local occupancy")
        self.publish_measurement_timing(
            "lidar", source_clock="beamng", source_stamp_ns=source_ns,
            state_source_stamp_ns=state_source_ns,
            output_clock=self.measurement_output_clock(),
            mapped_stamp_ns=(state_ros_stamp.sec * 1_000_000_000
                             + state_ros_stamp.nanosec),
            skew_ns=skew_ns, point_count=len(points),
            status="published")

    def flush_pending_sensors(self):
        if not self.state_history:
            return
        now = time.monotonic()
        latest_source = self.state_history[-1][0]
        max_skew_ns = int(float(self.get_parameter("max_pose_lidar_skew").value)
                          * 1_000_000_000)

        images = list(self.pending_images)
        self.pending_images.clear()
        for msg, received in images:
            source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            match = closest_state_sample(self.state_history, source_ns, max_skew_ns)
            nearest = min(self.state_history, key=lambda item: abs(item[0] - source_ns))
            state_source, _, state = nearest
            skew_ns = state_source - source_ns
            if match is not None:
                (state_source, _, state), skew_ns = match
                stamp = self.measurement_stamp(msg.header.stamp)
                if stamp is not None:
                    self.process_image(msg, state, stamp)
                    continue
            if source_ns > latest_source and now - received <= 0.35:
                self.pending_images.append((msg, received))
            elif now - received > 0.35:
                self.publish_measurement_timing(
                    "camera", source_clock="beamng", source_stamp_ns=source_ns,
                    output_clock=self.measurement_output_clock(), mapped_stamp_ns=None,
                    status="dropped_unmatched_state", skew_ns=skew_ns)
            else:
                self.pending_images.append((msg, received))

        lidars = list(self.pending_lidars)
        self.pending_lidars.clear()
        for msg, received in lidars:
            source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            match = closest_state_sample(self.state_history, source_ns, max_skew_ns)
            nearest = min(self.state_history, key=lambda item: abs(item[0] - source_ns))
            state_source, state_stamp, state = nearest
            skew_ns = state_source - source_ns
            if match is not None:
                (state_source, state_stamp, state), skew_ns = match
                self.process_lidar(msg, state, state_stamp)
            elif source_ns > latest_source and now - received <= 0.35:
                self.pending_lidars.append((msg, received))
            elif now - received > 0.35:
                self.publish_measurement_timing(
                    "lidar", source_clock="beamng", source_stamp_ns=source_ns,
                    state_source_stamp_ns=state_source,
                    output_clock=self.measurement_output_clock(), mapped_stamp_ns=None,
                    skew_ns=skew_ns, max_skew_ns=max_skew_ns,
                    status="dropped_pose_skew")
            else:
                self.pending_lidars.append((msg, received))

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
            sensor_origin_xy=self.lidar_offset[:2], clear_robot_footprint=True,
            carrier_model=self.carrier_model,
            obstacle_min_height=float(self.get_parameter("obstacle_min_height").value),
            ground_max_slope=float(self.get_parameter("ground_max_slope").value))
        msg=OccupancyGrid(); msg.header.stamp=stamp; msg.header.frame_id="spot1/base_link"
        msg.info.resolution=res; msg.info.width=width; msg.info.height=height
        msg.info.origin.position.x=-width*res/2; msg.info.origin.position.y=-height*res/2
        msg.info.origin.orientation.w=1.0; msg.data=grid.ravel().tolist(); self.map_pub.publish(msg)

    def publish_terrain(self, points, origin, stamp):
        layers = terrain_layers(points, sensor_origin_xy=origin,
            ground_plane_radius=float(self.get_parameter("ground_plane_radius").value),
            ground_clearance=float(self.get_parameter("ground_clearance").value),
            carrier_model=self.carrier_model,
            obstacle_min_height=float(self.get_parameter("obstacle_min_height").value),
            max_slope=float(self.get_parameter("ground_max_slope").value))
        msg = GridMap(); msg.header.stamp = stamp
        msg.header.frame_id = "spot1/terrain_base_link"
        msg.info.resolution = .5; msg.info.length_x = msg.info.length_y = 40.
        msg.info.pose.orientation.w = 1.
        # Keep the diagnostic envelope in-process. Only the layer the Mapper
        # consumes is added to the DDS payload so sensor timing is not loaded
        # with redundant 80x80 arrays.
        published = ("traversability", "unknown", "elevation",
                     "elevation_valid", "witnessed_step")
        msg.layers = [name for name in layers if name in published]
        msg.basic_layers = ["traversability", "unknown"]
        for name, layer in layers.items():
            if name not in published:
                continue
            data = Float32MultiArray()
            data.layout.dim = [MultiArrayDimension(label="column_index", size=80, stride=6400),
                               MultiArrayDimension(label="row_index", size=80, stride=80)]
            data.data = grid_map_values(layer)
            msg.data.append(data)
        self.terrain_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node=Adapter(); executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
