#!/usr/bin/env python3
import time

import rclpy
from graphnav_msgs.msg import TraversalFeedback
from nav_msgs.msg import Odometry


class MatchedFeedback:
    def __init__(self, node):
        self.node = node
        self.odom = None
        self.pub = node.create_publisher(TraversalFeedback, '/spot1/traversal_feedback', 10)
        node.create_subscription(Odometry, '/spot1/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.odom = msg

    def publish(self):
        if self.odom is None:
            return False
        msg = TraversalFeedback()
        msg.header = self.odom.header
        msg.robot_profile_id = 'default'
        msg.traversability_class = 'default'
        msg.outcome = TraversalFeedback.OUTCOME_SUCCEEDED
        msg.retryable = False
        msg.commanded_heading = 0.0
        msg.observed_heading = 0.0
        msg.commanded_distance = 1.0
        msg.observed_distance = 1.0
        msg.duration = 0.0
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return True


def main():
    rclpy.init()
    node = rclpy.create_node('wildos_matched_feedback')
    state = MatchedFeedback(node)
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and state.odom is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if state.publish():
        print('MATCHED_FEEDBACK_SENT')
    else:
        print('NO_ODOM')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
