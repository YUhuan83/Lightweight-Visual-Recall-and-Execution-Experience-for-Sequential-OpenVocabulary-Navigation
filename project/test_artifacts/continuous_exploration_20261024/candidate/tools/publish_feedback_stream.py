#!/usr/bin/env python3
import time

import rclpy
from graphnav_msgs.msg import TraversalFeedback


def main():
    rclpy.init()
    node = rclpy.create_node('wildos_feedback_round_stream')
    publisher = node.create_publisher(TraversalFeedback, '/spot1/traversal_feedback', 10)
    end = time.monotonic() + 20.0
    while rclpy.ok() and time.monotonic() < end:
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
        msg.duration = 0.0
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
