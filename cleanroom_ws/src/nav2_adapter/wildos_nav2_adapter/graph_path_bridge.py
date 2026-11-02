"""Topology guidance to rolling Nav2 actions, with one task owner."""

from copy import deepcopy
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav2_msgs.action import ComputePathToPose, FollowPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from graphnav_msgs.msg import NavigationGraph, TraversalFeedback, KeyValue

from .navigation import (
    Task,
    State,
    RecallCandidatePolicy,
    arrived,
    distance,
    route_candidates,
)


def xy(pose):
    return pose.position.x, pose.position.y


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GraphPathBridge(Node):
    def __init__(self):
        super().__init__("wildos_graph_path_bridge")
        self.task = Task()
        self.odom = None
        self.route = None
        self.grid = None
        self.local_grid = None
        self.active_path = None
        self.evidence = None
        self.query_started_ns = 0
        self.active = None
        self.active_endpoint = None
        self.compute = None
        self.follow_request = None
        self.last_request = 0.0
        self.last_destination = None
        self.last_planned_route = None
        self.planned_target = None
        self.recall = None
        self.candidates = []
        self.graph = None
        self.experience_start = None
        self.policy = RecallCandidatePolicy()
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.controller = ActionClient(self, FollowPath, "/follow_path")
        self.clear_goal = self.create_client(
            Trigger, "/spot1/graphnav_planner/clear_goal"
        )
        self.destination = self.create_publisher(
            PoseStamped, "/spot1/graphnav_destination", 10
        )
        self.events = self.create_publisher(String, "/wildos/plan_status", 10)
        self.recall_feedback = self.create_publisher(
            String, "/spot1/remembered_object_feedback", 10
        )
        self.traversal = self.create_publisher(
            TraversalFeedback, "/spot1/traversal_feedback", 10
        )
        self.create_subscription(NavigationGraph, "/spot1/nav_graph", self.on_graph, 1)
        self.create_subscription(Odometry, "/spot1/odom", self.on_odom, 10)
        self.create_subscription(Path, "/spot1/graphnav_planner/path", self.on_route, 1)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_map, 1
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local_map, 1
        )
        self.create_subscription(
            PoseStamped, "/spot1/raw_object_waypoint", self.on_target, 10
        )
        self.create_subscription(
            String, "/spot1/object_confirmation", self.on_confirmation, 10
        )
        self.create_subscription(
            String, "/spot1/object_search/query", self.on_query, 10
        )
        self.create_subscription(
            String, "/spot1/remembered_object_candidates", self.on_recall, 10
        )
        self.create_service(Trigger, "/wildos/stop", self.stop_service)
        self.create_timer(0.2, self.tick)

    def status(self, event, **details):
        self.events.publish(
            String(
                data=json.dumps(
                    dict(
                        state=event,
                        task_state=self.task.state.name,
                        query=self.task.query,
                        **details,
                    )
                )
            )
        )

    def stop(self):
        self.task.state = State.IDLE
        if self.active is not None:
            self.active.cancel_goal_async()
        self.active = self.active_endpoint = None
        self.experience_start = None
        # A pending acceptance checks the task before adopting its handle.

    def stop_service(self, request, response):
        self.stop()
        response.success = True
        response.message = "Stopped"
        return response

    def on_query(self, message):
        query = " ".join(message.data.casefold().split())
        if query == self.task.query and self.task.state not in (
            State.IDLE,
            State.ARRIVED,
        ):
            return
        self.stop()
        self.task.begin(query)
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.route = self.evidence = self.recall = None
        self.last_destination = self.last_planned_route = None
        self.candidates = []
        if self.clear_goal.service_is_ready():
            self.clear_goal.call_async(Trigger.Request())
        self.status("query_started")

    def on_odom(self, message):
        if message.header.frame_id == "spot1/odom":
            self.odom = message
            if (
                self.active is not None
                and self.experience_start is not None
                and distance(xy(message.pose.pose), xy(self.experience_start.pose.pose))
                >= 3.0
            ):
                self.record_traversal()

    def on_graph(self, message):
        self.graph = message

    def record_traversal(self):
        start, current, graph = self.experience_start, self.odom, self.graph
        if graph is None or not graph.nodes or start is None:
            return
        first = min(
            graph.nodes, key=lambda n: distance(xy(n.pose), xy(start.pose.pose))
        )
        last = min(
            graph.nodes, key=lambda n: distance(xy(n.pose), xy(current.pose.pose))
        )
        feedback = TraversalFeedback()
        feedback.header = deepcopy(current.header)
        feedback.from_node, feedback.to_node = first.uuid, last.uuid
        feedback.start_pose, feedback.end_pose = start.pose.pose, current.pose.pose
        feedback.robot_profile_id = feedback.traversability_class = "default"
        feedback.commanded_heading = math.atan2(
            current.pose.pose.position.y - start.pose.pose.position.y,
            current.pose.pose.position.x - start.pose.pose.position.x,
        )
        feedback.observed_heading = feedback.commanded_heading
        feedback.observed_distance = float(
            distance(xy(start.pose.pose), xy(current.pose.pose))
        )
        feedback.commanded_distance = feedback.observed_distance
        feedback.duration = max(
            0.001,
            (current.header.stamp.sec - start.header.stamp.sec)
            + (current.header.stamp.nanosec - start.header.stamp.nanosec) * 1e-9,
        )
        feedback.outcome = TraversalFeedback.OUTCOME_SUCCEEDED
        feedback.observations = [KeyValue(key="execution_source", value=[1.0])]
        self.traversal.publish(feedback)
        self.experience_start = deepcopy(current)

    def on_map(self, message):
        self.grid = message

    def on_local_map(self, message):
        self.local_grid = message

    def local_obstacle_ahead(self, current):
        grid, path = self.local_grid, self.active_path
        if grid is None or path is None:
            return False
        points = [xy(p.pose) for p in path.poses]
        first = min(range(len(points)), key=lambda i: distance(points[i], current))
        previous, length = current, 0.0
        # Center samples only. The C++ controller owns the full footprint.
        for point in points[first:]:
            length += distance(previous, point)
            previous = point
            if length > 8.0:
                break
            x = int(
                math.floor(
                    (point[0] - grid.info.origin.position.x) / grid.info.resolution
                )
            )
            y = int(
                math.floor(
                    (point[1] - grid.info.origin.position.y) / grid.info.resolution
                )
            )
            if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                if grid.data[y * grid.info.width + x] >= 100:
                    return True
        return False

    def on_route(self, message):
        if message.header.frame_id == "spot1/odom" and message.poses:
            points = tuple(
                (round(p.pose.position.x, 1), round(p.pose.position.y, 1))
                for p in message.poses
            )
            previous = (
                tuple(
                    (round(p.pose.position.x, 1), round(p.pose.position.y, 1))
                    for p in self.route.poses
                )
                if self.route
                else ()
            )
            if points != previous:
                self.route = message

    def on_target(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if message.header.frame_id != "spot1/odom" or stamp < self.query_started_ns:
            return
        p = message.pose.position
        if self.task.refine((p.x, p.y, p.z)):
            self.recall = None
            if (
                self.last_destination is None
                or distance(self.last_destination, self.task.target) > 0.5
            ):
                self.destination.publish(message)
                self.last_destination = self.task.target

    def on_confirmation(self, message):
        try:
            data = json.loads(message.data)
            stamp = int(data.get("observation_stamp_ns", 0))
        except (ValueError, TypeError):
            return
        now_ns = self.get_clock().now().nanoseconds
        if (
            data.get("query") == self.task.query
            and data.get("frame") == "spot1/odom"
            and data.get("search_confirmed") is True
            and self.query_started_ns <= stamp <= now_ns
            and now_ns - stamp <= 5_000_000_000
        ):
            self.evidence = data

    def on_recall(self, message):
        try:
            data = json.loads(message.data)
            if (
                data["query"] != self.task.query
                or self.task.target is not None
                or self.odom is None
            ):
                return
            views = self.policy.rank(
                data["views"], xy(self.odom.pose.pose), yaw(self.odom.pose.pose)
            )
        except (ValueError, KeyError, TypeError):
            return
        if views:
            if (
                self.recall is not None
                and self.recall["id"] == views[0]["id"]
                and distance(self.recall["position"], views[0]["position"]) < 0.5
            ):
                return
            self.recall = views[0]
            self.task.state = State.APPROACHING_TARGET
            self.publish_destination(self.recall["position"])
            self.status("recall_selected", id=self.recall["id"])

    def publish_destination(self, point):
        goal = PoseStamped()
        goal.header.frame_id = "spot1/odom"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(
            float, point
        )
        goal.pose.orientation.w = 1.0
        self.destination.publish(goal)

    def endpoint_clear(self, point):
        grid = self.grid
        if grid is None:
            return True
        x = int(
            math.floor((point[0] - grid.info.origin.position.x) / grid.info.resolution)
        )
        y = int(
            math.floor((point[1] - grid.info.origin.position.y) / grid.info.resolution)
        )
        if not 0 <= x < grid.info.width or not 0 <= y < grid.info.height:
            return True
        return grid.data[y * grid.info.width + x] < 99

    def tick(self):
        if self.task.state in (State.IDLE, State.ARRIVED) or self.odom is None:
            return
        current = xy(self.odom.pose.pose)
        if arrived(
            current,
            self.task.target,
            self.evidence,
            self.task.query,
            self.get_clock().now().nanoseconds,
        ):
            if self.active is not None:
                self.active.cancel_goal_async()
            self.active = self.active_endpoint = None
            self.task.state = State.ARRIVED
            self.status(
                "object_found",
                arrival_basis="visual_goal_distance_xy",
                target=self.task.target,
                distance=distance(current, self.task.target),
            )
            return
        if self.recall and distance(current, self.recall["position"]) < 3.0:
            self.recall_feedback.publish(
                String(
                    data=json.dumps(
                        dict(
                            query=self.task.query,
                            id=self.recall["id"],
                            outcome="unconfirmed",
                        )
                    )
                )
            )
            self.recall = None
            self.task.state = State.EXPLORING
            if self.clear_goal.service_is_ready():
                self.clear_goal.call_async(Trigger.Request())
        if (
            self.compute is not None
            or self.follow_request is not None
            or time.monotonic() - self.last_request < 1.0
        ):
            return
        if not self.planner.server_is_ready() or not self.controller.server_is_ready():
            return
        target = self.task.target or (self.recall or {}).get("position")
        points = [xy(p.pose) for p in self.route.poses] if self.route else []
        updated = self.route is not self.last_planned_route
        near_end = (
            self.active_endpoint is None
            or distance(current, self.active_endpoint) < 8.0
        )
        target_changed = target is not None and (
            self.planned_target is None or distance(target, self.planned_target) > 0.5
        )
        if (
            self.active is not None
            and not near_end
            and not updated
            and not target_changed
            and not self.local_obstacle_ahead(current)
        ):
            return
        self.candidates = [
            p
            for p in route_candidates(points, current, yaw(self.odom.pose.pose), target)
            if self.endpoint_clear(p)
        ]
        self.last_planned_route = self.route
        self.planned_target = target
        self.try_candidate()

    def try_candidate(self):
        self.last_request = time.monotonic()
        if not self.candidates:
            if self.active is None:
                self.task.state = State.RECOVERING
            self.status("no_vehicle_path", retained_action=self.active is not None)
            return
        x, y, heading = self.candidates.pop(0)
        goal = ComputePathToPose.Goal()
        goal.planner_id = "CarLike"
        goal.use_start = True
        goal.start.header = deepcopy(self.odom.header)
        goal.start.pose = deepcopy(self.odom.pose.pose)
        goal.start.pose.position.z = 0.0
        start_yaw = yaw(goal.start.pose)
        goal.start.pose.orientation.x = goal.start.pose.orientation.y = 0.0
        goal.start.pose.orientation.z = math.sin(start_yaw / 2)
        goal.start.pose.orientation.w = math.cos(start_yaw / 2)
        goal.goal.header = deepcopy(goal.start.header)
        goal.goal.pose.position.x, goal.goal.pose.position.y = x, y
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = (
            math.sin(heading / 2),
            math.cos(heading / 2),
        )
        query_started = self.query_started_ns
        self.compute = self.planner.send_goal_async(goal)
        self.compute.add_done_callback(
            lambda f: self.on_plan_accepted(f, query_started)
        )
        self.status(
            "planning",
            start=[goal.start.pose.position.x, goal.start.pose.position.y, start_yaw],
            start_stamp_ns=(
                goal.start.header.stamp.sec * 1_000_000_000
                + goal.start.header.stamp.nanosec
            ),
            goal=[x, y],
            goal_yaw=heading,
            retained_action=self.active is not None,
        )

    def on_plan_accepted(self, future, started):
        handle = future.result()
        if started != self.query_started_ns or self.task.state in (
            State.IDLE,
            State.ARRIVED,
        ):
            if handle.accepted:
                handle.cancel_goal_async()
            self.compute = None
            return
        if not handle.accepted:
            self.compute = None
            self.try_candidate()
            return
        self.compute = handle.get_result_async()
        self.compute.add_done_callback(lambda f: self.on_plan(f, started))

    def on_plan(self, future, started):
        self.compute = None
        if started != self.query_started_ns or self.task.state in (
            State.IDLE,
            State.ARRIVED,
        ):
            return
        result = future.result()
        path = result.result.path
        if result.status != 4 or len(path.poses) < 2:
            self.try_candidate()
            return
        if path.header.frame_id != "spot1/odom" or any(
            not all(math.isfinite(v) for v in (p.pose.position.x, p.pose.position.y))
            for p in path.poses
        ):
            self.try_candidate()
            return
        goal = FollowPath.Goal()
        goal.path, goal.controller_id, goal.goal_checker_id = (
            path,
            "FollowPath",
            "goal_checker",
        )
        endpoint = xy(path.poses[-1].pose)
        # No cancel first: Nav2 accepts and preempts its current FollowPath.
        self.follow_request = self.controller.send_goal_async(goal)
        self.follow_request.add_done_callback(
            lambda f: self.on_follow(f, path, endpoint, started)
        )

    def on_follow(self, future, path, endpoint, started):
        self.follow_request = None
        handle = future.result()
        if started != self.query_started_ns or self.task.state in (
            State.IDLE,
            State.ARRIVED,
        ):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.status("controller_rejected", retained_action=self.active is not None)
            return
        self.active, self.active_endpoint = handle, endpoint
        self.active_path = path
        if self.experience_start is None:
            self.experience_start = deepcopy(self.odom)
        self.task.state = (
            State.APPROACHING_TARGET
            if self.task.target or self.recall
            else State.EXPLORING
        )
        handle.get_result_async().add_done_callback(
            lambda f: self.on_follow_result(f, handle)
        )
        self.status("following", goal=list(endpoint))

    def on_follow_result(self, future, handle):
        if handle is not self.active:
            return
        self.active = self.active_endpoint = None
        self.status("path_finished", action_status=future.result().status)


def main(args=None):
    rclpy.init(args=args)
    node = GraphPathBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
