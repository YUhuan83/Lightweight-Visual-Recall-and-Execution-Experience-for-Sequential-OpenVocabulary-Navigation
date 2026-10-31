"""Convert a semantic object position to a road-level vehicle standoff goal."""
from __future__ import annotations

import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .geometry import standoff_xy
from .geometry import quaternion_matrix
from .mapping import reachable_ring_candidate, safe_travel_distance


class GoalStandoff(Node):
    def __init__(self):
        super().__init__("beamng_vehicle_goal_standoff")
        self.declare_parameter("input_topic", "/spot1/raw_object_waypoint")
        self.declare_parameter("output_topic", "/spot1/imgnav_waypoint")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("query_topic", "/spot1/object_search/query")
        self.declare_parameter("standoff", 10.0)
        self.declare_parameter("min_update_distance", 1.0)
        self.declare_parameter("confirmation_frames", 5)
        self.declare_parameter("confirmation_radius", 1.5)
        self.declare_parameter("obstacle_margin", 2.0)
        self.odom = None
        self.occupancy = None
        self.last_goal = None
        self.targets = []
        self.active_query = None
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10)
        self.create_subscription(OccupancyGrid, "/spot1/local_map", self.on_occupancy, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("input_topic").value), self.on_target, 10)
        self.create_subscription(
            String, str(self.get_parameter("query_topic").value), self.on_query, 10)

    def on_odom(self, message):
        self.odom = message

    def on_occupancy(self, message):
        self.occupancy = message

    def on_query(self, message):
        query = " ".join(message.data.casefold().split())
        if not query or query == self.active_query:
            return
        self.active_query = query
        self.targets.clear()
        self.last_goal = None

    def on_target(self, target):
        if self.odom is None:
            return
        self.targets.append((float(target.pose.position.x), float(target.pose.position.y),
                             float(target.pose.position.z)))
        frames = int(self.get_parameter("confirmation_frames").value)
        self.targets = self.targets[-frames:]
        if len(self.targets) < frames:
            return
        values = np.asarray(self.targets, dtype=float)
        stable = np.median(values, axis=0)
        radius = float(self.get_parameter("confirmation_radius").value)
        if np.max(np.linalg.norm(values[:, :2] - stable[:2], axis=1)) > radius:
            return
        robot = self.odom.pose.pose.position
        point = target.pose.position
        standoff = float(self.get_parameter("standoff").value)
        x, y = standoff_xy((robot.x, robot.y), (stable[0], stable[1]), standoff)
        clamped = False
        if self.occupancy is not None:
            target_world = np.array([stable[0] - robot.x, stable[1] - robot.y, 0.0])
            q = self.odom.pose.pose.orientation
            rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
            target_local = target_world @ rotation
            info = self.occupancy.info
            grid = np.asarray(self.occupancy.data, dtype=np.int16).reshape(
                int(info.height), int(info.width))
            origin = (info.origin.position.x, info.origin.position.y)
            candidate_local = reachable_ring_candidate(
                grid, float(info.resolution), origin, target_local[:2], standoff,
                float(self.get_parameter("obstacle_margin").value))
            if candidate_local is not None:
                candidate_world = rotation @ np.array(
                    [candidate_local[0], candidate_local[1], 0.0])
                x = robot.x + candidate_world[0]
                y = robot.y + candidate_world[1]
                clamped = not np.allclose(candidate_local, target_local[:2] *
                    max(0.0, 1.0 - standoff / max(np.linalg.norm(target_local[:2]), 1e-6)),
                    atol=0.2)
            else:
                target_distance = float(np.linalg.norm(target_world[:2]))
                desired_travel = max(0.0, target_distance - standoff)
                safe_travel = safe_travel_distance(
                    grid, float(info.resolution), origin,
                    target_local[:2], desired_travel,
                    float(self.get_parameter("obstacle_margin").value))
                direction = target_world[:2] / max(target_distance, 1e-6)
                x = robot.x + direction[0] * safe_travel
                y = robot.y + direction[1] * safe_travel
                clamped = True
        candidate = (x, y)
        minimum = float(self.get_parameter("min_update_distance").value)
        if self.last_goal is not None and ((candidate[0] - self.last_goal[0]) ** 2
                + (candidate[1] - self.last_goal[1]) ** 2) ** 0.5 < minimum:
            return
        output = PoseStamped()
        output.header = target.header
        output.header.frame_id = "spot1/odom"
        output.pose.position.x = x
        output.pose.position.y = y
        output.pose.position.z = robot.z
        output.pose.orientation = self.odom.pose.pose.orientation
        self.publisher.publish(output)
        self.last_goal = candidate
        self.get_logger().info(
            f"stable object ({stable[0]:.2f}, {stable[1]:.2f}, {stable[2]:.2f}) -> "
            f"road standoff ({x:.2f}, {y:.2f}, {robot.z:.2f})"
            f" obstacle_clamped={clamped}")


def main(args=None):
    rclpy.init(args=args); node = GoalStandoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
