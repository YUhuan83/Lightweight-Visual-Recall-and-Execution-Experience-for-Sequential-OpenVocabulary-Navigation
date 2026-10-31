"""Follow the original GraphNav route using checked, car-like Nav2 segments."""
from __future__ import annotations

import json
import math
import time
from copy import deepcopy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .graph_route import (distance, distance_to_route, footprint_blockers,
                          freshness_issue, maneuver_cost, remaining_route, route_candidates,
                          validate_vehicle_path)
from .memory_selection import MemorySelection
from .motion_path import maneuver_reached, motion_sections


def xy(pose):
    return (float(pose.position.x), float(pose.position.y))


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class GraphPathBridge(MemorySelection, Node):
    def __init__(self):
        super().__init__("wildos_graph_path_bridge")
        self.declare_parameter("follow_path", False)
        self.declare_parameter("planner_id", "CarLike")
        self.declare_parameter("max_segment_length", 24.0)
        self.declare_parameter("max_candidates", 10)
        self.declare_parameter("minimum_turning_radius", 8.0)
        self.declare_parameter("heading_quantization_bins", 72)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("costmap_timeout_sec", 3.0)
        self.declare_parameter("tracking_error_limit", 1.0)
        self.follow_enabled = bool(self.get_parameter("follow_path").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.max_segment = float(self.get_parameter("max_segment_length").value)
        self.max_candidates = int(self.get_parameter("max_candidates").value)
        self.minimum_turning_radius = float(
            self.get_parameter("minimum_turning_radius").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.costmap_timeout = float(self.get_parameter("costmap_timeout_sec").value)
        self.tracking_error_limit = float(self.get_parameter("tracking_error_limit").value)
        if (not math.isfinite(self.max_segment) or self.max_segment < 4 or self.max_candidates < 1
                or not math.isfinite(self.minimum_turning_radius)
                or self.minimum_turning_radius <= 0
                or not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0
                or not math.isfinite(self.costmap_timeout) or self.costmap_timeout <= 0
                or not math.isfinite(self.tracking_error_limit) or self.tracking_error_limit <= 0):
            raise ValueError("invalid graph segment parameters")

        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.graph_reset_client = self.create_client(Trigger, "/spot1/graphnav_planner/clear_goal")
        self.follow_client = (ActionClient(self, FollowPath, "/follow_path")
                              if self.follow_enabled else None)
        self.path_pub = self.create_publisher(Path, "/wildos/planned_path", 10)
        self.full_path_pub = self.create_publisher(Path, "/wildos/full_planned_path", 10)
        self.status_pub = self.create_publisher(String, "/wildos/plan_status", 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, "/spot1/graphnav_destination", 10)
        self.create_subscription(Path, "/spot1/graphnav_planner/path", self.on_route, 10)
        self.create_subscription(PoseStamped, "/spot1/imgnav_waypoint", self.on_waypoint, 10)
        self.create_subscription(PoseStamped, "/spot1/remembered_object_goal", self.on_memory_goal, 10)
        self.create_subscription(String, "/spot1/object_search/query", self.on_query, 10)
        self.create_subscription(Odometry, "/spot1/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_map, 10)
        self.create_timer(1.0, self.repeat_status)
        self.create_timer(2.0, self.retry_when_planner_starts)
        self.create_timer(0.25, self.monitor_inputs)
        self.create_timer(0.5, self.retry_graph_reset)

        self.grid = self.odom = self.route = None
        self.latest_route = None
        self.grid_received_at = self.odom_received_at = None
        self.expected = self.memory_goal = None
        self.memory_heading = None
        self.query_grace_until = 0.0
        self.query = None
        self.query_generation = 0
        self.query_reset_pending = self.query_reset_inflight = False
        self.pending_destination = None
        self.minimum_route_stamp = 0
        self.epoch = 0
        self.pending = False
        self.plan_handle = None
        self.follow_handle = None
        self.following = False
        self.motion_queue = []
        self.route_endpoint = None
        self.blocked_endpoint = None
        self.blocked_route = None
        self.blocked_grid_hash = None
        self.blocked_until = 0.0
        self.recovery_attempts = 0
        self.last_plan_start = None
        self.last_plan_endpoint = None
        self.candidates = []
        self.candidate_index = 0
        self.failure_reasons = []
        self.normal_best = None
        self.last_status = {"status": "idle", "source": "graphnav"}
        self.init_memory_selection()
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info("GraphNav route bridge started, planner=%s, follow=%s" %
                               (self.planner_id, self.follow_enabled))

    def status(self, state, **details):
        self.last_status = {"status": state,
                            "source": "memory_via_graphnav" if self.memory_goal else "graphnav",
                            **details}
        self.status_pub.publish(String(data=json.dumps(self.last_status)))
        self.get_logger().info("plan status: " + json.dumps(self.last_status))

    def repeat_status(self):
        self.status_pub.publish(String(data=json.dumps(self.last_status)))

    def retry_when_planner_starts(self):
        # Recovery must not depend on GraphNav publishing another path. Use
        # only the already accepted destination, fresh inputs and bounded
        # backoff; start_segment performs all collision checks again.
        if (self.blocked_endpoint is not None and self.last_status.get("status")
                in ("follow_failed", "stale_odom", "stale_costmap")):
            if (not self.follow_enabled or self.query_reset_pending or self.route is None or self.pending
                    or self.following or self.input_freshness_issue() is not None
                    or time.monotonic() < self.blocked_until
                    or self.recovery_attempts >= 3
                    or not self.plan_client.wait_for_server(timeout_sec=0.0)
                    or (self.follow_enabled
                        and not self.follow_client.wait_for_server(timeout_sec=0.0))):
                return
            endpoint = xy(self.route.poses[-1].pose) if self.route.poses else None
            if endpoint is None or (self.expected is not None
                    and distance(endpoint, self.expected) > 7.0):
                return
            self.recovery_attempts += 1
            self.last_plan_start = self.last_plan_endpoint = None
            self.start_segment()
            return
        if (self.last_status.get("status") in (
                "planner_unavailable", "controller_unavailable", "stale_odom", "stale_costmap",
                "execution_ready", "planning_only")
                and self.blocked_endpoint is None
                and not self.query_reset_pending
                and self.route is not None and not self.pending and not self.following
                and self.input_freshness_issue() is None
                and self.plan_client.wait_for_server(timeout_sec=0.0)
                and (not self.follow_enabled
                     or self.follow_client.wait_for_server(timeout_sec=0.0))):
            self.start_segment()

    def on_parameters(self, parameters):
        follow = self.follow_enabled
        for parameter in parameters:
            if parameter.name == "follow_path":
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(successful=False, reason="follow_path must be bool")
                follow = parameter.value
            elif parameter.name != "use_sim_time":
                return SetParametersResult(successful=False,
                    reason="restart bridge to change " + parameter.name)
        if follow != self.follow_enabled:
            self.epoch += 1
            self.cancel_goal(self.plan_handle, "execution_mode_compute_path")
            self.cancel_goal(self.follow_handle, "execution_mode_follow_path")
            self.plan_handle = self.follow_handle = None
            self.pending = self.following = False
            self.motion_queue = []
            self.follow_enabled = follow
            if follow and self.follow_client is None:
                self.follow_client = ActionClient(self, FollowPath, "/follow_path")
            self.last_plan_start = self.last_plan_endpoint = None
            self.status("execution_ready" if follow else "planning_only")
        return SetParametersResult(successful=True)

    def retry_graph_reset(self):
        if not self.query_reset_pending or self.query_reset_inflight:
            return
        if not self.graph_reset_client.service_is_ready():
            if self.last_status.get("status") != "graph_reset_unavailable":
                self.status("graph_reset_unavailable", query=self.query)
            return
        generation = self.query_generation
        try:
            future = self.graph_reset_client.call_async(Trigger.Request())
        except Exception as exc:
            self.status("graph_reset_error", detail=str(exc)[:80])
            return
        self.query_reset_inflight = True
        future.add_done_callback(lambda f: self.on_graph_reset_done(f, generation))

    def on_graph_reset_done(self, future, generation):
        if generation != self.query_generation:
            return
        self.query_reset_inflight = False
        try:
            success = future.result().success
        except Exception:
            success = False
        if not success:
            self.status("graph_reset_error", query=self.query)
            return
        self.query_reset_pending = False
        self.minimum_route_stamp = self.get_clock().now().nanoseconds
        if self.pending_destination is not None:
            goal, self.pending_destination = self.pending_destination, None
            self.relay_destination(goal)
        if self.memory_goal is not None and self.odom is not None and distance(
                xy(self.odom.pose.pose), self.memory_goal) < 1.0:
            self.status("already_at_goal", goal=list(self.memory_goal), requires_visual_confirmation=True)
        else:
            self.status("awaiting_graph_route", query=self.query)

    def relay_destination(self, message):
        if self.query_reset_pending:
            self.pending_destination = deepcopy(message)
            return
        output = deepcopy(message)
        output.header.stamp = self.get_clock().now().to_msg()
        self.minimum_route_stamp = self.get_clock().now().nanoseconds
        self.waypoint_pub.publish(output)

    def monitor_inputs(self):
        if not self.pending and not self.following:
            return
        issue = self.input_freshness_issue()
        if issue is None:
            if self.following and self.motion_queue:
                points = [xy(p.pose) for p in self.motion_queue[0].poses]
                error = distance_to_route(xy(self.odom.pose.pose), points)
                if error > self.tracking_error_limit:
                    self.cancel_goal(self.follow_handle, "tracking_deviation")
                    self.follow_failed("tracking_deviation", cross_track_error=round(error, 3))
            return
        self.epoch += 1
        self.cancel_goal(self.plan_handle, "stale_input_compute_path")
        self.cancel_goal(self.follow_handle, "stale_input_follow_path")
        self.plan_handle = self.follow_handle = None
        self.pending = self.following = False
        self.motion_queue = []
        self.last_plan_start = self.last_plan_endpoint = None
        self.report_stale_input(issue)

    def reset(self):
        self.recovery_attempts = 0
        self.normal_best = None
        self.epoch += 1
        self.motion_queue = []
        self.pending = False
        plan_handle = self.plan_handle
        self.plan_handle = None
        self.cancel_goal(plan_handle, "compute_path")
        self.candidates = []
        self.failure_reasons = []
        self.route = None
        self.latest_route = None
        self.route_endpoint = None
        self.blocked_endpoint = None
        self.blocked_route = None
        self.blocked_grid_hash = None
        self.blocked_until = 0.0
        self.last_plan_start = None
        self.last_plan_endpoint = None
        self.following = False
        if self.follow_handle is not None:
            self.cancel_goal(self.follow_handle, "follow_path")
            self.follow_handle = None

    def on_odom(self, message):
        self.odom = message
        self.odom_received_at = time.monotonic()

    def on_map(self, message):
        self.grid = message
        self.grid_received_at = time.monotonic()

    def input_freshness_issue(self):
        return freshness_issue(time.monotonic(), self.odom_received_at,
                               self.grid_received_at, self.odom_timeout,
                               self.costmap_timeout)

    def report_stale_input(self, issue):
        state, age = issue
        if self.last_status.get("status") == state:
            return
        details = {} if age is None else {"age_sec": round(age, 2)}
        self.status(state, **details)

    def cancel_goal(self, handle, label):
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warning("could not cancel %s goal: %s" % (label, exc))

    def on_query(self, message):
        query = " ".join(message.data.casefold().split())
        if not query or query == self.query:
            return
        self.query = query
        self.query_generation += 1
        self.reset()
        self.clear_memory_selection()
        self.memory_goal = self.expected = None
        self.memory_heading = None
        self.pending_destination = None
        self.query_reset_pending = True
        self.query_reset_inflight = False
        # GraphNav can explore frontiers before object triangulation publishes
        # a waypoint. Briefly suppress the preceding query's stale route.
        self.query_grace_until = time.monotonic() + 2.0
        self.status("awaiting_graph_route", query=query)
        self.retry_graph_reset()
        self.activate_pending_memory()

    def on_waypoint(self, message):
        point = xy(message.pose)
        if message.header.frame_id != "spot1/odom" or not all(map(math.isfinite, point)):
            return
        if self.memory_goal is not None and distance(point, self.memory_goal) > 1.0:
            return
        if self.expected is None or distance(self.expected, point) > 3.0:
            self.reset()
            self.expected = point
            self.query_grace_until = 0.0
            self.relay_destination(message)
            self.status("awaiting_graph_route", goal=list(point))

    def on_memory_goal(self, message):
        if self.memory_views or self.memory_selecting:
            return  # The query's candidate set owns selection; ignore legacy replay.
        point = xy(message.pose)
        heading = yaw(message.pose)
        if (message.header.frame_id != "spot1/odom"
                or not all(map(math.isfinite, (*point, heading)))):
            return
        if self.memory_goal is not None and distance(self.memory_goal, point) < 1.0:
            return
        self.reset()
        self.memory_goal = self.expected = point
        self.memory_heading = heading
        self.query_grace_until = 0.0
        if self.odom is not None and distance(xy(self.odom.pose.pose), point) < 1.0:
            self.status("already_at_goal", goal=list(point), requires_visual_confirmation=True)
            return
        # Make the remembered viewpoint an explicit GraphNav destination.
        # This retains its graph route and avoids a direct start-to-goal retry loop.
        self.relay_destination(message)
        self.status("awaiting_graph_route", goal=list(point))

    def on_route(self, message):
        if self.memory_selecting and self.memory_active is None:
            return
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if (self.query_reset_pending or stamp < self.minimum_route_stamp
                or time.monotonic() < self.query_grace_until or not message.poses
                or self.odom is None or self.grid is None):
            return
        if self.memory_goal is not None and distance(xy(self.odom.pose.pose), self.memory_goal) < 1.0:
            return
        if message.header.frame_id != self.grid.header.frame_id:
            self.status("frame_mismatch", route_frame=message.header.frame_id,
                        costmap_frame=self.grid.header.frame_id)
            return
        endpoint = xy(message.poses[-1].pose)
        if (message.header.frame_id != self.odom.header.frame_id
                or any(not all(map(math.isfinite, xy(p.pose))) for p in message.poses)):
            self.status("invalid_graph_route")
            return
        # Accept nearby graph endpoints, but reject another task's destination.
        if self.expected is not None and distance(endpoint, self.expected) > 7.0:
            return  # Old graph path from the preceding query or frontier goal.
        self.latest_route = message
        if self.following or self.pending:
            return  # Finish the current short segment before selecting another.
        if self.blocked_endpoint is not None and distance(endpoint, self.blocked_endpoint) < 4.0:
            points = [xy(p.pose) for p in message.poses]
            changed_route = (self.blocked_route is not None and
                             any(distance_to_route(p, self.blocked_route) > 3.0
                                 for p in points))
            changed_map = (time.monotonic() >= self.blocked_until and
                           hash(tuple(self.grid.data)) != self.blocked_grid_hash)
            if not changed_route and not changed_map:
                return
            if not changed_route and self.recovery_attempts >= 3:
                return
            if changed_route:
                self.recovery_attempts = 0
            else:
                self.recovery_attempts += 1
            self.blocked_endpoint = None
            self.last_plan_start = None
            self.last_plan_endpoint = None
        start = xy(self.odom.pose.pose)
        if (self.last_plan_start is not None and self.last_plan_endpoint is not None
                and distance(start, self.last_plan_start) < 2.0
                and distance(endpoint, self.last_plan_endpoint) < 4.0):
            return
        self.route = message
        if self.route_endpoint is not None and distance(endpoint, self.route_endpoint) >= 4.0:
            self.recovery_attempts = 0
        self.route_endpoint = endpoint
        self.start_segment()

    def current_route_points(self):
        points = [xy(p.pose) for p in self.route.poses]
        # GraphNav's 6 m goal radius can end before the remembered viewpoint.
        # Finish that approach with the same collision-checked local planner.
        if (self.expected is not None and distance(points[-1], self.expected) <= 7.0
                and distance(points[-1], self.expected) > 1e-6):
            points.append(self.expected)
        return remaining_route(xy(self.odom.pose.pose), points)

    def start_segment(self):
        if (self.query_reset_pending or self.odom is None or self.grid is None
                or self.route is None or self.pending or self.following):
            return
        stale = self.input_freshness_issue()
        if stale:
            self.report_stale_input(stale)
            return
        start = xy(self.odom.pose.pose)
        endpoint = self.expected or xy(self.route.poses[-1].pose)
        self.last_plan_start = start
        self.last_plan_endpoint = endpoint
        if distance(start, endpoint) < 1.0 and not self.memory_selecting:
            self.status("route_done", goal=list(endpoint),
                        requires_visual_confirmation=self.memory_goal is not None)
            return
        blocked = footprint_blockers(self.grid, start, yaw(self.odom.pose.pose))
        if blocked:
            self.block_current_route()
            self.status("start_footprint_blocked", goal=list(endpoint),
                        blocked_samples=blocked)
            return
        points = self.current_route_points()
        candidates = route_candidates(
            start, points, self.grid, maximum=self.max_segment,
            limit=self.max_candidates)
        self.candidates = []
        for candidate in candidates:
            if self.memory_goal is not None and distance(candidate[:2], self.memory_goal) < 1e-6:
                self.candidates.extend(self.memory_terminal_candidates(start, candidate[:2]))
                continue
            self.candidates.append(candidate)
            x, y, heading, arc = candidate
            # A coarse graph corner is a position constraint, not a required
            # vehicle orientation. Its outgoing edge can force a Dubins loop
            # at a short segment endpoint. Also try the approach bearing;
            # Memory endpoints evaluate observation and arrival headings separately.
            bearing = math.atan2(y - start[1], x - start[0])
            difference = math.atan2(math.sin(bearing - heading), math.cos(bearing - heading))
            if abs(difference) > 0.1:
                self.candidates.append((x, y, bearing, arc))
        if self.memory_selecting:
            self.memory_full_check = distance(start, endpoint) <= self.max_segment
            if self.memory_full_check:
                # A short prefix cannot prove that the remembered viewpoint is reachable.
                self.candidates = self.memory_terminal_candidates(start, endpoint)
            self.memory_deadline = time.monotonic() + 25.0
        self.candidate_index = 0
        self.failure_reasons = []
        self.normal_best = None
        if not self.candidates:
            if self.memory_candidates_exhausted():
                return
            self.block_current_route()
            self.status("no_free_graph_candidate", goal=list(endpoint))
            return
        if not self.plan_client.wait_for_server(timeout_sec=0.1):
            self.status("planner_unavailable")
            return
        self.try_candidate()

    def try_candidate(self):
        if self.candidate_index >= len(self.candidates):
            if self.memory_candidates_exhausted():
                return
            self.block_current_route()
            self.status("no_vehicle_path", goal=list(self.route_endpoint),
                        candidates=len(self.candidates), reasons=self.failure_reasons,
                        candidate_goals=[list(c[:3]) for c in self.candidates],
                        graph_route=[list(p) for p in self.current_route_points()])
            return
        x, y, heading, _ = self.candidates[self.candidate_index]
        pose = PoseStamped()
        pose.header.frame_id = self.grid.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(heading / 2)
        pose.pose.orientation.w = math.cos(heading / 2)
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.use_start = False
        goal.planner_id = self.planner_id
        epoch = self.epoch
        index = self.candidate_index
        self.pending = True
        try:
            future = self.plan_client.send_goal_async(goal)
        except Exception as exc:
            self.candidate_failed("planner_send_error:" + str(exc)[:80])
            return
        future.add_done_callback(lambda f: self.on_plan_accepted(f, epoch, index))

    def candidate_failed(self, reason):
        self.pending = False
        self.plan_handle = None
        self.failure_reasons.append(reason)
        if self.normal_best is not None:
            path, target, _ = self.normal_best
            self.normal_best = None
            self.execute_checked_path(path, target)
            return
        self.candidate_index += 1
        self.try_candidate()

    def on_plan_accepted(self, future, epoch, index):
        try:
            handle = future.result()
        except Exception as exc:
            if epoch == self.epoch:
                self.candidate_failed("planner_error:" + str(exc)[:80])
            return
        if epoch != self.epoch:
            if handle is not None and handle.accepted:
                self.cancel_goal(handle, "stale_compute_path")
            return
        if handle is None or not handle.accepted:
            self.candidate_failed("planner_rejected")
            return
        self.plan_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_plan_result(f, epoch, index))

    def on_plan_result(self, future, epoch, index):
        if epoch != self.epoch or index != self.candidate_index:
            return
        self.plan_handle = None
        stale = self.input_freshness_issue()
        if stale:
            self.pending = False
            self.last_plan_start = None
            self.last_plan_endpoint = None
            self.report_stale_input(stale)
            return
        try:
            result = future.result()
            path = result.result.path
        except Exception as exc:
            self.candidate_failed("planner_error:" + str(exc)[:80])
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.candidate_failed("empty_path")
            return
        if path.header.frame_id != self.grid.header.frame_id:
            self.candidate_failed("path_frame_mismatch")
            return
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        target = self.candidates[index][:2]
        route = self.current_route_points()
        reason = validate_vehicle_path(points, headings,
                                       [xy(self.odom.pose.pose)] + route,
                                       self.grid, target,
                                       minimum_turning_radius=self.minimum_turning_radius,
                                       heading_quantization_bins=int(self.get_parameter("heading_quantization_bins").value),
                                       start=xy(self.odom.pose.pose),
                                       target_heading=(self.candidates[index][2]
                                           if self.memory_goal is not None
                                           and distance(target, self.memory_goal) < 1e-6 else None))
        if reason:
            self.candidate_failed(reason)
            return
        self.full_path_pub.publish(path)
        if self.memory_selecting:
            self.record_memory_path(path, points, headings)
            return
        if self.memory_goal is None:
            cost = maneuver_cost(points)
            if self.normal_best is not None:
                if self.normal_best[2] < cost:
                    path, target, cost = self.normal_best
                self.normal_best = None
            elif (index+1 < len(self.candidates)
                  and distance(self.candidates[index+1][:2], target) < 1e-6):
                # Compare only the two headings at this endpoint. Do not run
                # every shorter candidate once a feasible endpoint exists.
                self.normal_best = (path, target, cost)
                self.candidate_index += 1
                self.try_candidate()
                return
        self.execute_checked_path(path, target)

    def execute_checked_path(self, path, target):
        # A map can change while the alternative heading is being planned.
        self.pending = False
        stale = self.input_freshness_issue()
        if stale:
            self.normal_best = None
            self.report_stale_input(stale)
            return
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        reason = validate_vehicle_path(points, headings,
            [xy(self.odom.pose.pose)]+self.current_route_points(), self.grid, target,
            minimum_turning_radius=self.minimum_turning_radius,
            heading_quantization_bins=int(self.get_parameter("heading_quantization_bins").value),
            start=xy(self.odom.pose.pose))
        if reason:
            self.block_current_route()
            self.status("path_changed_before_execution", reason=reason)
            return
        self.pending = False
        if not self.follow_enabled:
            self.path_pub.publish(path)
            self.status("planned", goal=list(target), poses=len(path.poses),
                        planner_id=self.planner_id, graph_goal=list(self.route_endpoint))
            return
        # Keep full-path validation and memory scoring; split execution alone.
        # RPP prunes nearest points before detecting its own reversing cusps.
        self.motion_queue = []
        for first, last in motion_sections(points):
            section = deepcopy(path)
            section.poses = section.poses[first:last + 1]
            self.motion_queue.append(section)
        self.send_motion_segment()

    def send_motion_segment(self):
        if not self.motion_queue:
            return
        stale = self.input_freshness_issue()
        if stale:
            self.motion_queue = []
            self.last_plan_start = self.last_plan_endpoint = None
            self.report_stale_input(stale)
            return
        path = self.motion_queue[0]
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        target = points[-1]
        reason = validate_vehicle_path(points, headings,
            [xy(self.odom.pose.pose)] + self.current_route_points(), self.grid, target,
            minimum_turning_radius=self.minimum_turning_radius,
            heading_quantization_bins=int(self.get_parameter("heading_quantization_bins").value),
            start=xy(self.odom.pose.pose))
        if reason:
            self.follow_failed("motion_section_invalid", detail=reason)
            return
        if not self.follow_client.wait_for_server(timeout_sec=0.1):
            self.motion_queue = []
            self.status("controller_unavailable")
            return
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        self.status("planned", goal=list(target), poses=len(path.poses),
                    planner_id=self.planner_id, graph_goal=list(self.route_endpoint),
                    motion_sections_remaining=len(self.motion_queue))
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = "FollowPath"
        follow_goal.goal_checker_id = ("maneuver_goal_checker"
                                      if len(self.motion_queue) > 1 else "goal_checker")
        epoch = self.epoch
        self.following = True
        try:
            future = self.follow_client.send_goal_async(follow_goal)
        except Exception as exc:
            self.follow_failed("controller_send_error", detail=str(exc)[:80])
            return
        future.add_done_callback(
            lambda f: self.on_follow_accepted(f, epoch, target))

    def on_follow_accepted(self, future, epoch, target):
        try:
            handle = future.result()
        except Exception:
            handle = None
        if epoch != self.epoch:
            if handle is not None and handle.accepted:
                self.cancel_goal(handle, "stale_follow_path")
            return
        if handle is None or not handle.accepted:
            self.follow_failed("controller_rejected")
            return
        self.follow_handle = handle
        self.status("following", goal=list(target), graph_goal=list(self.route_endpoint))
        handle.get_result_async().add_done_callback(
            lambda f: self.on_follow_result(f, epoch, target))

    def on_follow_result(self, future, epoch, target):
        if epoch != self.epoch:
            return
        self.follow_handle = None
        self.following = False
        try:
            status = future.result().status
        except Exception:
            status = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            if len(self.motion_queue) > 1:
                stale = self.input_freshness_issue()
                if stale:
                    self.motion_queue = []
                    self.last_plan_start = self.last_plan_endpoint = None
                    self.report_stale_input(stale)
                    return
                cusp = self.motion_queue[0].poses[-1].pose
                if not maneuver_reached(xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                                        xy(cusp), yaw(cusp)):
                    self.follow_failed("cusp_not_reached", goal=list(xy(cusp)))
                    return
                self.motion_queue.pop(0)
                self.status("maneuver_done", goal=list(target),
                            motion_sections_remaining=len(self.motion_queue))
                self.send_motion_segment()  # Recheck against the current map.
                return
            self.motion_queue = []
            self.blocked_endpoint = None
            self.status("segment_done", goal=list(target))
            self.recovery_attempts = 0
            if self.latest_route is not None:
                self.route = self.latest_route
                self.route_endpoint = xy(self.route.poses[-1].pose)
            self.start_segment()
        else:
            self.follow_failed("follow_aborted", action_status=status)

    def follow_failed(self, reason, **details):
        self.epoch += 1
        self.motion_queue = []
        self.following = False
        self.follow_handle = None
        # A controller failure (stalled vehicle, rejected action, or missed
        # cusp) does not prove the remembered viewpoint is unreachable.
        # Retain it and back off this route. Only failed planning checks may
        # reject a memory candidate and release its GraphNav destination.
        self.block_current_route()
        self.status("follow_failed", reason=reason,
                    graph_goal=list(self.route_endpoint) if self.route_endpoint else None,
                    **details)

    def block_current_route(self):
        self.blocked_endpoint = self.route_endpoint
        self.blocked_route = ([xy(p.pose) for p in self.route.poses]
                              if self.route is not None else None)
        self.blocked_grid_hash = hash(tuple(self.grid.data)) if self.grid else None
        self.blocked_until = time.monotonic() + 20.0


def main(args=None):
    rclpy.init(args=args)
    node = GraphPathBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
