"""Broadcast the base transform represented by WildOS odometry."""
from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("wildos_odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self.on_odometry, 10)

    def on_odometry(self, message: Odometry) -> None:
        parent = message.header.frame_id
        child = message.child_frame_id
        if not parent or not child:
            self.get_logger().warning("Skipping odometry with an empty TF frame")
            return
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = child
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
