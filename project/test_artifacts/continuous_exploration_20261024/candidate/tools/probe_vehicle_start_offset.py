"""Read-only Nav2 probe comparing real and hypothetical nearby start poses."""
import math
import time
import copy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Probe(Node):
    def __init__(self):
        super().__init__("probe_vehicle_start_offset")
        self.odom = None
        self.create_subscription(Odometry, "/spot1/odom",
                                 lambda m: setattr(self, "odom", m), qos_profile_sensor_data)
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")


def main():
    rclpy.init()
    node = Probe()
    until = time.monotonic() + 15
    while node.odom is None and time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.2)
    if node.odom is None or not node.client.wait_for_server(timeout_sec=3):
        print("missing odom or planner", flush=True)
        return
    pose = node.odom.pose.pose
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    for offset in [0., -1., -2., -3.]:
        goal = ComputePathToPose.Goal()
        goal.planner_id = "CarLike"
        goal.use_start = True
        start = PoseStamped()
        start.header.frame_id = "spot1/odom"
        start.header.stamp = node.get_clock().now().to_msg()
        start.pose = copy.deepcopy(pose)
        start.pose.position.x += offset * math.cos(yaw)
        start.pose.position.y += offset * math.sin(yaw)
        end = PoseStamped()
        end.header.frame_id = "spot1/odom"
        end.header.stamp = start.header.stamp
        end.pose.orientation = pose.orientation
        end.pose.position.x = start.pose.position.x + 6 * math.cos(yaw)
        end.pose.position.y = start.pose.position.y + 6 * math.sin(yaw)
        goal.start = start
        goal.goal = end
        sent = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=4)
        if not sent.done() or not sent.result().accepted:
            print("rejected", offset, flush=True)
            continue
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=12)
        if result.done():
            response = result.result()
            print("offset", offset, "status", response.status,
                  "poses", len(response.result.path.poses), flush=True)
        else:
            print("timeout", offset, flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
