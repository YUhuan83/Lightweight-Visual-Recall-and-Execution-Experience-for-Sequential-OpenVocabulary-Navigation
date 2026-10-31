"""Convert BeamNG StateSensor to a read-only Nav2-compatible odometry input.

This adapter uses simulator state as an explicitly named debug/oracle odometry
source. It does not claim sensor-only localization and publishes no controls.
"""
from __future__ import annotations

import rclpy
from beamng_msgs.msg import StateSensor
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .math3d import quaternion_from_forward_up, world_velocity_to_body


class StateOdom(Node):
    def __init__(self):
        super().__init__("wildos_state_odom")
        self.declare_parameter("state_topic", "/vehicles/ego/sensors/state")
        self.declare_parameter("odom_topic", "/wildos/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_sim_truth", True)
        if not bool(self.get_parameter("use_sim_truth").value):
            raise RuntimeError(
                "This first adapter only supports explicit simulator-truth debug odometry")
        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            StateSensor, str(self.get_parameter("state_topic").value), self.callback, 10)
        self.get_logger().warn(
            "Publishing explicit simulator-truth odometry; not sensor-only localization")

    def callback(self, msg: StateSensor):
        stamp = msg.header.stamp
        q = quaternion_from_forward_up(msg.dir, msg.up)
        body_velocity = world_velocity_to_body(msg.velocity, msg.dir, msg.up)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = odom_frame
        out.child_frame_id = base_frame
        out.pose.pose.position.x = msg.position.x
        out.pose.pose.position.y = msg.position.y
        out.pose.pose.position.z = msg.position.z
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]
        out.twist.twist.linear.x = body_velocity[0]
        out.twist.twist.linear.y = body_velocity[1]
        out.twist.twist.linear.z = body_velocity[2]
        self.pub.publish(out)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x = msg.position.x
        tf.transform.translation.y = msg.position.y
        tf.transform.translation.z = msg.position.z
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = StateOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
