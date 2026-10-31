#!/usr/bin/env python3
import rclpy
from graphnav_msgs.msg import TraversalFeedback


def main():
    rclpy.init()
    node = rclpy.create_node('wildos_round_feedback_injector')
    pub = node.create_publisher(TraversalFeedback, '/spot1/traversal_feedback', 10)
    msg = TraversalFeedback()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'spot1/odom'
    msg.robot_profile_id = 'default'
    msg.traversability_class = 'default'
    msg.outcome = TraversalFeedback.OUTCOME_SUCCEEDED
    msg.retryable = False
    msg.commanded_heading = 0.0
    msg.observed_heading = 0.0
    msg.commanded_distance = 1.0
    msg.observed_distance = 1.0
    msg.duration = 1.0
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
