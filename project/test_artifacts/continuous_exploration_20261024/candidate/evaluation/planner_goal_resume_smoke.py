"""Check that autonomous exploration resumes after an explicit goal is reached.

Run on a ROS 2 host after sourcing Humble and the WildOS workspace.
The planner executable is passed as the only argument.
"""

import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from graphnav_msgs.msg import (
    Edge, EdgeTraversability, NavigationGraph, Node,
    NodeTraversabilityProperties, RobotNodeTraversability,
)
from nav_msgs.msg import Odometry, Path
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
    graph.edges = [edge]
    return graph


def wait_for_path(observer, graph_pub, paths, destination, seconds=7.0):
    paths.clear()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        graph_pub.publish(make_graph(observer.get_clock().now().to_msg()))
        rclpy.spin_once(observer, timeout_sec=0.1)
        if any(msg.poses and abs(msg.poses[-1].pose.position.x - destination) < 1.0
               for msg in paths):
            return True
    return False


def main(executable):
    rclpy.init()
    observer = RosNode("goal_resume_observer", namespace="/codex_goal_resume")
    graph_pub = observer.create_publisher(
        NavigationGraph, "/codex_goal_resume/planner_node/nav_graph", 10)
    odom_pub = observer.create_publisher(
        Odometry, "/codex_goal_resume/planner_node/odom", 10)
    goal_pub = observer.create_publisher(
        PoseStamped, "/codex_goal_resume/planner_node/goal_pose", 10)
    paths = []
    observer.create_subscription(Path, "/codex_goal_resume/planner_node/path",
                                 paths.append, 10)
    process = subprocess.Popen([
        executable, "--ros-args", "-r", "__ns:=/codex_goal_resume",
        "-p", "autonomous_exploration_enabled:=true",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        assert wait_for_path(observer, graph_pub, paths, 10.0), \
            "initial exploration path was not published"
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.orientation.w = 1.0
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.orientation.w = 1.0
        # Send odometry before the goal, so the planner can recognize that
        # the robot is already at the explicit goal.
        for _ in range(10):
            odom.header.stamp = observer.get_clock().now().to_msg()
            odom_pub.publish(odom)
            rclpy.spin_once(observer, timeout_sec=0.05)
        paths.clear()
        goal.header.stamp = observer.get_clock().now().to_msg()
        goal_pub.publish(goal)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not paths:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert paths, "explicit goal was not handled"
        assert wait_for_path(observer, graph_pub, paths, 10.0), \
            "exploration did not resume after the explicit goal was reached"
        print("PASS exploration resumed after explicit goal")
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
    