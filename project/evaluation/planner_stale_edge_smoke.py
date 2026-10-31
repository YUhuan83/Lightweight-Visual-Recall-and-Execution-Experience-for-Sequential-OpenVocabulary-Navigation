"""Verify that stale dynamic evidence does not disconnect a physical graph edge.

Run on a ROS 2 host after sourcing Humble and the WildOS workspace.
Pass the planner executable as the sole argument.
"""

import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from graphnav_msgs.msg import (
    Edge, EdgeTraversability, NavigationGraph, Node,
    NodeTraversabilityProperties, RobotEdgeTraversability,
    RobotNodeTraversability,
)
from nav_msgs.msg import Path
from rclpy.node import Node as RosNode


def make_graph(stamp):
    graph = NavigationGraph()
    graph.header.frame_id = "map"
    graph.header.stamp = stamp
    graph.trav_classes = ["default"]
    graph.current_node_idx = 0
    for index, x in enumerate((0.0, 10.0)):
        node = Node()
        node.uuid.id = [index + 1] * 16
        node.pose.position.x = x
        node.pose.orientation.w = 1.0
        terrain = NodeTraversabilityProperties()
        terrain.is_frontier = index == 1
        terrain.explored_radius = 2.0
        if index == 1:
            terrain.frontier_points = [Point(x=11.0)]
        node.trav_properties = [terrain]
        physical = RobotNodeTraversability()
        physical.robot_profile_id = "default"
        physical.feasible = True
        physical.physical_traversability = 1.0
        physical.confidence = 1.0
        node.robot_traversability = [physical]
        graph.nodes.append(node)
    edge = Edge()
    edge.from_idx = 0
    edge.to_idx = 1
    cost = EdgeTraversability()
    cost.traversability_cost = 10.0
    edge.traversability = [cost]
    evidence = RobotEdgeTraversability()
    evidence.robot_profile_id = "default"
    evidence.physical_traversability = 1.0
    evidence.confidence = 1.0
    evidence.observed_at.sec = stamp.sec - 30
    evidence.observed_at.nanosec = stamp.nanosec
    edge.robot_traversability = [evidence]
    graph.edges = [edge]
    return graph


def main(executable):
    rclpy.init()
    observer = RosNode("stale_edge_observer", namespace="/codex_stale_edge")
    publisher = observer.create_publisher(
        NavigationGraph, "/codex_stale_edge/planner_node/nav_graph", 10)
    paths = []
    observer.create_subscription(
        Path, "/codex_stale_edge/planner_node/path", paths.append, 10)
    process = subprocess.Popen([
        executable, "--ros-args", "-r", "__ns:=/codex_stale_edge",
        "-p", "autonomous_exploration_enabled:=true",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 7.0
        while time.monotonic() < deadline:
            publisher.publish(make_graph(observer.get_clock().now().to_msg()))
            rclpy.spin_once(observer, timeout_sec=0.1)
            if any(len(path.poses) >= 2 for path in paths):
                print("PASS stale edge retained for physical path")
                return
            if process.poll() is not None:
                raise AssertionError(f"planner exited with {process.returncode}")
        raise AssertionError("stale edge disconnected the exploration path")
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    os.environ.setdefault("ROS_DOMAIN_ID", "91")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    main(sys.argv[1])
