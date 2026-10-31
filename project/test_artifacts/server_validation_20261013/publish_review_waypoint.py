"""Publish one fresh waypoint at a forward, traversable graph node for planning-only review."""
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def yaw_from(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    output = Path("/home/ly/town_memory_validation_20261013/review_waypoint.json")
    if output.exists():
        raise FileExistsError(output)
    rclpy.init()
    node = rclpy.create_node("review_waypoint_planning_only")
    latest = {"graph": None, "odom": None}
    node.create_subscription(NavigationGraph, "/spot1/nav_graph",
                             lambda msg: latest.__setitem__("graph", msg),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, "/spot1/odom",
                             lambda msg: latest.__setitem__("odom", msg),
                             qos_profile_sensor_data)
    publisher = node.create_publisher(PoseStamped, "/spot1/imgnav_waypoint", 10)
    deadline = time.monotonic() + 5.0
    try:
        while time.monotonic() < deadline and (latest["graph"] is None or latest["odom"] is None):
            rclpy.spin_once(node, timeout_sec=0.05)
        graph, odom = latest["graph"], latest["odom"]
        if graph is None or odom is None or not graph.nodes:
            raise RuntimeError("fresh navigation graph or odometry unavailable")
        origin = odom.pose.pose.position
        yaw = yaw_from(odom.pose.pose.orientation)
        candidates = []
        for index, item in enumerate(graph.nodes):
            dx = item.pose.position.x - origin.x
            dy = item.pose.position.y - origin.y
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
            distance = math.hypot(dx, dy)
            assessment = next((value for value in item.robot_traversability
                               if value.robot_profile_id == "default"), None)
            if (8.0 < forward < 16.0 and abs(lateral) < 1.5 and assessment is not None
                    and assessment.feasible and assessment.minimum_clearance >= 4.5
                    and assessment.dynamic_risk < 1.0):
                candidates.append((forward, abs(lateral), index, item, assessment,
                                   distance, lateral))
        if not candidates:
            raise RuntimeError("no forward graph node passed feasibility and clearance gates")
        forward, _, index, target_node, assessment, distance, lateral = min(candidates)
        pose = PoseStamped()
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.header.frame_id = graph.header.frame_id
        pose.pose.position = target_node.pose.position
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        deadline = time.monotonic() + 2.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("GraphPathBridge waypoint subscriber unavailable")
        publisher.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.2)
        report = {
            "published": True,
            "motion_command_published": False,
            "goal_topic": "/spot1/imgnav_waypoint",
            "frame_id": pose.header.frame_id,
            "stamp_ns": pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec,
            "graph_stamp_ns": graph.header.stamp.sec * 1_000_000_000 + graph.header.stamp.nanosec,
            "current_node_idx": int(graph.current_node_idx),
            "selected_node_idx": index,
            "start_xy": [origin.x, origin.y],
            "start_yaw_rad": yaw,
            "target_xy": [pose.pose.position.x, pose.pose.position.y],
            "distance_m": distance,
            "forward_m": forward,
            "lateral_m": lateral,
            "assessment": {
                "profile_id": assessment.robot_profile_id,
                "feasible": assessment.feasible,
                "minimum_clearance_m": assessment.minimum_clearance,
                "physical_traversability": assessment.physical_traversability,
                "dynamic_risk": assessment.dynamic_risk,
            },
        }
        output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
