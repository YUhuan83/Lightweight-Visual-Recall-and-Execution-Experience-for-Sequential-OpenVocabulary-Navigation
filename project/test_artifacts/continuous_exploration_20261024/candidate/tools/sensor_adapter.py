#!/usr/bin/env python3
"""Standalone ROS 2 adapter. Requires installed rclpy, messages and tf2_ros."""
import time

from sensor_contract import CONFIG as C, intrinsics, safe_command, transforms, watchdog


def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class Adapter(Node):
        def __init__(self):
            super().__init__('lane_sensor_adapter')
            self.declare_parameter('debug_wheel_tf', False)
            self.debug_tf = self.get_parameter('debug_wheel_tf').value
            self.static_tf = StaticTransformBroadcaster(self)
            self.dynamic_tf = TransformBroadcaster(self) if self.debug_tf else None
            messages = []
            for parent, child, xyz, quat in transforms():
                msg = TransformStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id, msg.child_frame_id = parent, child
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
                msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, quat)
                messages.append(msg)
            self.static_tf.sendTransform(messages)
            reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            self.handles = []
            self.seen_cloud_fields = False
            for name in C['cameras']:
                topic = f'camera/{name}'
                image_pub = self.create_publisher(Image, f'/rover/{topic}/image_raw', reliable)
                info_pub = self.create_publisher(CameraInfo, f'/rover/{topic}/camera_info', reliable)
                self.handles.append(self.create_subscription(
                    Image, f'/sim/{topic}/image_raw',
                    lambda msg, n=name, p=image_pub, i=info_pub: self.camera(msg, n, p, i),
                    qos_profile_sensor_data))
            for msg_type, topic, frame in [(Imu, 'imu', 'imu_link'),
                                            (PointCloud2, 'lidar/points', 'lidar_link'),
                                            (Odometry, 'wheel_odom', C['odom_frame'])]:
                pub = self.create_publisher(msg_type, f'/rover/{topic}', reliable)
                self.handles.append(self.create_subscription(
                    msg_type, f'/sim/{topic}',
                    lambda msg, p=pub, f=frame: self.relay(msg, p, f), qos_profile_sensor_data))
            self.command = (0.0, 0.0)
            self.received = float('-inf')
            self.cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', QoSProfile(depth=1))
            self.cmd_sub = self.create_subscription(Twist, '/rover/cmd_vel', self.command_input,
                                                    QoSProfile(depth=1))
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1 / C['command_rate'], self.tick, clock=self.steady_clock)

        def camera(self, msg, name, pub, info_pub):
            if (msg.width, msg.height) != (C['width'], C['height']):
                self.get_logger().error('Image size differs from SDF contract; refusing incorrect calibration', throttle_duration_sec=5)
                return
            msg.header.frame_id = f'camera_{name}_optical_frame'
            info = CameraInfo()
            info.header = msg.header
            info.width, info.height = C['width'], C['height']
            info.distortion_model, info.d = 'plumb_bob', [0.0] * 5
            fx, fy, cx, cy = intrinsics()
            info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
            pub.publish(msg)
            info_pub.publish(info)

        def relay(self, msg, pub, frame):
            # These private inputs belong only to our SDF sensors, not arbitrary devices.
            msg.header.frame_id = frame
            if isinstance(msg, PointCloud2) and not self.seen_cloud_fields:
                fields = [field.name for field in msg.fields]
                missing = sorted({'ring', 'time'} - set(fields))
                self.get_logger().info(f'PointCloud2 fields={fields}; absent ring/time={missing}; no fields synthesized')
                self.seen_cloud_fields = True
            if isinstance(msg, Odometry):
                msg.child_frame_id = C['base_frame']
                if self.dynamic_tf:
                    tf = TransformStamped()
                    tf.header = msg.header
                    tf.child_frame_id = msg.child_frame_id
                    tf.transform.translation.x = msg.pose.pose.position.x
                    tf.transform.translation.y = msg.pose.pose.position.y
                    tf.transform.translation.z = msg.pose.pose.position.z
                    tf.transform.rotation = msg.pose.pose.orientation
                    self.dynamic_tf.sendTransform(tf)
            pub.publish(msg)

        def command_input(self, msg):
            self.command = safe_command((msg.linear.x, msg.linear.y, msg.linear.z,
                                         msg.angular.x, msg.angular.y, msg.angular.z))
            self.received = time.monotonic()

        def tick(self, stop=False):
            msg = Twist()
            msg.linear.x, msg.angular.z = (0.0, 0.0) if stop else watchdog(
                self.command, self.received, time.monotonic())
            self.cmd_pub.publish(msg)

    # Keep the context alive during signal handling so final stop can be sent.
    from rclpy.signals import SignalHandlerOptions
    import signal
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Adapter()
    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.tick(stop=True)
                time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
