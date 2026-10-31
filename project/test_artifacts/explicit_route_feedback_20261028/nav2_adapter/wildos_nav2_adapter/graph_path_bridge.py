"""Follow the original GraphNav route using checked, car-like Nav2 segments."""
from __future__ import annotations

import json
import math
import time
from collections import deque
from copy import deepcopy
from threading import Lock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .graph_route import (cell_cost, distance, distance_to_route, footprint_blockers,
                          point_blockers, freshness_issue, maneuver_cost, remaining_route,
                          route_candidates, forward_arc_arrival_heading,
                          validate_vehicle_path, validate_local_prefix,
                          validate_ideal_path, validate_ideal_local_prefix)
from .input_quality import measurement_age, stamp_ns, valid_pose, valid_grid
from .continuous_path import ContinuousPath, trim_moving_path
from .execution_feedback import ExecutionFeedback
from .execution_recovery import ExecutionRecovery
from .exploration_cost import ExplorationCostComparison
from .target_observation import TargetObservation
from .visible_approach import VisibleTargetApproach
from .confirmed_approach import ConfirmedTargetApproach
from .recall_candidate_policy import RecallCandidatePolicy
from .navigation_transactions import NavigationTransactions
from .sensor_geometry import SBR_CAMERA_OFFSET, observation_clearance, geometry_restart_reason
from .motion_path import (MANEUVER_COMPLETION_POSITION_TOLERANCE_M, ACKERMANN_CUSP_POSITION_TOLERANCE_M,
                          NORMAL_GOAL_CHECKER_POSITION_TOLERANCE_M, maneuver_reached,
                          motion_sections, initial_maneuver_already_reached,
                          diagnose_maneuver_join, first_section_retreat_cost)
from .tracking_preview import preview_tracking
from .arrival_criteria import (OBJECT_SURFACE_ARRIVAL_DISTANCE_M,
                               BODY_OBJECT_ARRIVAL_DISTANCE_M, vehicle_body_distance,
                               within_body_arrival,
                               STANDOFF_ARRIVAL_TOLERANCE_M, memory_region_metrics,
                               effective_object_arrival_distance, planar_distance)

OBJECT_CONFIRMATION_STANDOFF_TOLERANCE_M = STANDOFF_ARRIVAL_TOLERANCE_M
NEAREST_SIGNAL_SWITCH_MARGIN_M = 2.0
NEAREST_SIGNAL_GOAL_PROGRESS_M = 3.0
NEAREST_SIGNAL_STANDOFF_M = 4.0


def is_traffic_light_query(query):
    return query in ('traffic light', 'traffic lights', 'traffic signal')


def xy(pose):
    return (float(pose.position.x), float(pose.position.y))


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class GraphPathBridge(ExplorationCostComparison, NavigationTransactions, ContinuousPath, ExecutionFeedback, ExecutionRecovery, TargetObservation, VisibleTargetApproach, ConfirmedTargetApproach, RecallCandidatePolicy, Node):
    def __init__(self):
        super().__init__("wildos_graph_path_bridge")
        self.declare_parameter("follow_path", False)
        self.declare_parameter("upstream_target_approach", False)
        self.upstream_target_approach = bool(self.get_parameter("upstream_target_approach").value)
        self.declare_parameter("carrier_model", "ideal_point")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("ideal_path_corridor", 8.0)
        self.declare_parameter("max_segment_length", 24.0)
        self.declare_parameter("max_candidates", 10)
        self.declare_parameter("max_motion_sections", 6)
        self.declare_parameter("max_consecutive_empty_paths", 4)
        self.declare_parameter("minimum_turning_radius", 0.0)
        self.declare_parameter("heading_quantization_bins", 72)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("costmap_timeout_sec", 3.0)
        self.declare_parameter("local_costmap_timeout_sec", 1.5)
        self.declare_parameter("compute_path_timeout_sec", 5.0)
        self.declare_parameter("tracking_error_limit", 1.0)
        self.declare_parameter("traffic_light_arrival_distance", BODY_OBJECT_ARRIVAL_DISTANCE_M)
        self.declare_parameter("object_arrival_distance", BODY_OBJECT_ARRIVAL_DISTANCE_M)
        self.declare_parameter("camera_offset", list(SBR_CAMERA_OFFSET))
        self.declare_parameter("minimum_camera_target_range", 1.0)
        observation_clearance(self.get_parameter('camera_offset').value,
            float(self.get_parameter('minimum_camera_target_range').value),
            STANDOFF_ARRIVAL_TOLERANCE_M)
        self.follow_enabled = bool(self.get_parameter("follow_path").value)
        self.carrier_model = str(self.get_parameter("carrier_model").value).strip().lower()
        if self.carrier_model not in ("ideal_point", "ackermann"):
            raise ValueError("carrier_model must be ideal_point or ackermann")
        self.declare_parameter('require_source_control', self.carrier_model == 'ackermann')
        self.require_source_control = bool(self.get_parameter('require_source_control').value)
        configured_planner = str(self.get_parameter("planner_id").value).strip()
        self.planner_id = configured_planner or (
            "GridBased" if self.carrier_model == "ideal_point" else "CarLike")
        self.ideal_path_corridor = float(self.get_parameter("ideal_path_corridor").value)
        self.max_segment = float(self.get_parameter("max_segment_length").value)
        self.max_candidates = int(self.get_parameter("max_candidates").value)
        self.max_motion_sections = int(self.get_parameter("max_motion_sections").value)
        self.max_consecutive_empty_paths = int(
            self.get_parameter("max_consecutive_empty_paths").value)
        self.empty_path_streak = 0
        self.empty_path_streak_epoch = 0
        self.empty_path_streak_route = None
        self.minimum_turning_radius = float(
            self.get_parameter("minimum_turning_radius").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.costmap_timeout = float(self.get_parameter("costmap_timeout_sec").value)
        self.local_costmap_timeout = float(self.get_parameter("local_costmap_timeout_sec").value)
        self.compute_path_timeout = float(self.get_parameter("compute_path_timeout_sec").value)
        self.tracking_error_limit = float(self.get_parameter("tracking_error_limit").value)
        if (not math.isfinite(self.max_segment) or self.max_segment < 4 or self.max_candidates < 1
                or self.max_motion_sections < 1 or self.max_motion_sections > 64
                or self.max_consecutive_empty_paths < 1
                or self.max_consecutive_empty_paths > 32
                or (self.carrier_model == "ackermann" and
                    (not math.isfinite(self.minimum_turning_radius)
                     or self.minimum_turning_radius <= 0))
                or not math.isfinite(self.ideal_path_corridor) or self.ideal_path_corridor <= 0
                or not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0
                or not math.isfinite(self.costmap_timeout) or self.costmap_timeout <= 0
                or not math.isfinite(self.local_costmap_timeout) or self.local_costmap_timeout <= 0
                or not math.isfinite(self.compute_path_timeout) or self.compute_path_timeout <= 0
                or not math.isfinite(self.tracking_error_limit) or self.tracking_error_limit <= 0):
            raise ValueError("invalid graph segment parameters")

        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.graph_reset_client = self.create_client(Trigger, "/spot1/graphnav_planner/release_goal")
        self.graph_search_reset_client = self.create_client(Trigger, "/spot1/graphnav_planner/clear_goal")
        self.follow_client = (ActionClient(self, FollowPath, "/follow_path")
                              if self.follow_enabled else None)
        self.path_pub = self.create_publisher(Path, "/wildos/planned_path", 10)
        self.full_path_pub = self.create_publisher(Path, "/wildos/full_planned_path", 10)
        self.status_pub = self.create_publisher(String, "/wildos/plan_status", 10)
        self.instance_release_pub = self.create_publisher(String, '/spot1/object_search/instance_release', 10)
        self.create_subscription(String, '/spot1/object_search/instance_release_ack', self.on_instance_release_ack, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, "/spot1/graphnav_destination", 10)
        self.route_rejection_pub = self.create_publisher(PoseStamped,
            "/spot1/graphnav_planner/rejected_route_endpoint", 10)
        self.route_observation_pub = self.create_publisher(PoseStamped,
            "/spot1/graphnav_planner/observed_route_endpoint", 10)
        self.create_subscription(Path, "/spot1/graphnav_planner/path", self.on_route, 10)
        self.create_subscription(PoseStamped,
            "/spot1/raw_object_waypoint" if self.upstream_target_approach else "/spot1/imgnav_waypoint",
            self.on_waypoint, 10)
        self.create_subscription(PoseStamped, "/spot1/remembered_object_goal", self.on_memory_goal, 10)
        self.create_subscription(String, "/spot1/object_search/query", self.on_query, 10)
        self.init_visible_approach()
        self.create_subscription(String, '/spot1/visible_target_approach', self.on_visible_approach, 10)
        self.create_timer(.25, self.activate_visible_approach)
        self.odom_input_lock = Lock()
        self.odom_input_pending = deque(maxlen=32)
        self.odom_input_latest_stamp = 0
        self.odom_input_group = MutuallyExclusiveCallbackGroup()
        self.odom_input_guard = self.create_guard_condition(self.consume_pending_odometry)
        self.create_subscription(Odometry, "/spot1/odom", self.receive_odometry,
                                 qos_profile_sensor_data, callback_group=self.odom_input_group)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_map, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_local_map, 1)
        self.create_timer(1.0, self.repeat_status)
        self.create_timer(2.0, self.retry_when_planner_starts)
        self.transaction_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.25, self.monitor_inputs, clock=self.transaction_clock)
        self.create_timer(0.5, self.retry_graph_reset, clock=self.transaction_clock)

        self.grid = self.odom = self.route = None
        self.latest_route = None
        self.route_complete = True
        self.require_route_status = True
        self.route_metadata = {}
        self.retired_explorations = {}
        self.waiting_routes = {}
        self.route_diagnostic_log_at = float('-inf')
        self.route_accept_log_at = float('-inf')
        self.create_subscription(String, "/spot1/graphnav_planner/route_status", self.on_route_status, 10)
        self.create_subscription(String, "/spot1/graphnav_planner/retired_exploration", self.on_exploration_retired, 10)
        self.create_subscription(String, "/spot1/object_confirmation", self.on_object_confirmation, 10)
        self.query_started_ns = 0
        self.query_origin = None
        self.nearest_signal_position = None
        self.nearest_signal_distance = None
        self.nearest_signal_standoff = None
        self.object_found = False
        self.active_object_confirmation = None
        self.init_confirmed_approach()
        self.latest_route_complete = True
        self.grid_received_at = self.odom_received_at = None
        # Keep receipt cadence separate from the measurement timestamp age.
        # Costmap headers can legitimately lag by one publication period while
        # messages are arriving normally; receipt time is used only to yield
        # between successive planner candidates.
        self.grid_arrived_at = None
        self.local_grid = None
        self.local_grid_received_at = None
        self.expected = self.memory_goal = None
        self.memory_heading = None
        self.query_grace_until = 0.0
        self.query = None
        self.query_generation = 0
        self.query_reset_pending = self.query_reset_inflight = False
        self.pending_destination = None
        self.minimum_route_stamp = 0
        self.epoch = 0
        self.plan_request_serial = 0
        self.active_plan_serial = None
        self.pending = False
        self.candidate_waiting_for_map = False
        self.plan_handle = None
        self.plan_request_started_at = None
        self.follow_handle = None
        self.following = False
        self.motion_queue = []
        self.route_endpoint = None
        self.blocked_endpoint = None
        self.blocked_route = None
        self.blocked_grid_hash = None
        self.blocked_until = 0.0
        self.recovery_attempts = 0
        self.recovery_progress = None
        self.last_plan_start = None
        self.last_plan_endpoint = None
        self.candidates = []
        self.candidate_index = 0
        self.failure_reasons = []
        self.normal_best = None
        # Bounded no-path memory, scoped to one GraphNav exploration identity,
        # one query and a 2 m start region. New owners, a new query or a moved
        # vehicle retry the goal.
        self.failed_goal_scope = None
        self.failed_goals = {}
        self.failed_goal_grid_message = None
        self.failed_goal_grid_signature = None
        self.active_plan_start = None
        self.active_plan_failure_context = None
        self.lookback_active = False
        self.lookback_started_at = 0.
        self.lookback_last_at = 0.
        self.lookback_last_pose = None
        self.last_status = {"status": "idle", "source": "graphnav"}
        self.init_observation()
        self.init_execution_feedback()
        self.init_execution_recovery()
        self.init_continuous_path()
        self.init_navigation_transactions()
        self.init_exploration_cost()
        self.init_memory_selection()
        self.declare_strategy_parameters()
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info("GraphNav route bridge started, carrier=%s planner=%s follow=%s" %
                               (self.carrier_model, self.planner_id, self.follow_enabled))

    def declare_strategy_parameters(self):
        """Declare immutable strategy settings before installing runtime guards."""
        pass

    def status(self, state, **details):
        metadata = self.exploration_metadata(self.route)
        self.last_status = {"schema_version": 2,
                            "stamp_ns": self.get_clock().now().nanoseconds,
                            "status": state, "query": self.query,
                            "query_generation": self.query_generation,
                            "carrier_model": self.carrier_model,
                            "minimum_turning_radius_m": (self.minimum_turning_radius
                                if self.carrier_model == "ackermann" else 0.0),
                            "planner_id": self.planner_id,
                            "source": ("memory_via_graphnav" if self.memory_goal else
                                       "visible_target_via_graphnav" if self.visible_approach_active else "graphnav"),
                            "exploration_id": metadata.get('exploration_id') if metadata else None,
                            "exploration_goal": metadata.get('goal') if metadata else None,
                            "replay_inputs": {
                                "navigation_epoch": self.epoch,
                                "odom_stamp_ns": stamp_ns(self.odom) if self.odom is not None else None,
                                "global_costmap_stamp_ns": stamp_ns(self.grid) if self.grid is not None else None,
                                "local_costmap_stamp_ns": stamp_ns(self.local_grid) if self.local_grid is not None else None,
                                "route_stamp_ns": stamp_ns(self.route) if self.route is not None else None},
                            **details}
        self.status_pub.publish(String(data=json.dumps(self.last_status)))
        self.get_logger().info("plan status: " + json.dumps(self.last_status))

    def repeat_status(self):
        self.status_pub.publish(String(data=json.dumps(self.last_status)))

    def retry_when_planner_starts(self):
        self.consume_pending_odometry()
        if self.follow_release is not None:
            self.monitor_follow_transactions()
            return
        # Recovery must not depend on GraphNav publishing another path. Use
        # only the already accepted destination, fresh inputs and bounded
        # backoff; start_segment performs all collision checks again.
        if (self.blocked_endpoint is not None and self.last_status.get("status")
                in ("follow_failed", "recovery_failed", "stale_odom", "stale_costmap", "stale_local_costmap",
                    "memory_locally_blocked",
                        "path_changed_before_execution", "object_visible_not_arrived",
                        "confirmation_before_standoff", "confirmed_standoff_update_deferred",
                        "confirmed_other_instance_deferred", "confirmation_outside_memory_region",
                        "execution_unavailable")):
            if (not self.follow_enabled or self.query_reset_pending or self.follow_quarantine
                    or self.route is None or self.pending
                    or self.following or self.input_freshness_issue() is not None
                    or self.local_freshness_issue() is not None
                    or self.execution_transport_issue() is not None
                    or time.monotonic() < self.blocked_until
                    or not self.plan_client.wait_for_server(timeout_sec=0.0)
                    or (self.follow_enabled
                        and not self.follow_client.wait_for_server(timeout_sec=0.0))):
                return
            if self.recovery_attempts >= 3:
                # Exhaustion belongs to the accepted destination. It must not
                # wait for another GraphNav publication to request retirement.
                if self.fail_confirmed_approach('route_recovery_exhausted',
                                                retry_count=self.recovery_attempts):
                    return
                if self.reject_exploration_route():
                    self.status('route_recovery_exhausted', retry_count=self.recovery_attempts)
                return
            endpoint = xy(self.route.poses[-1].pose) if self.route.poses else None
            if endpoint is None or (getattr(self, 'route_complete', True) and self.expected is not None
                    and distance(endpoint, self.expected) > 7.0):
                return
            if self.last_status.get('status') != 'execution_unavailable':
                self.recovery_attempts += 1
            self.last_plan_start = self.last_plan_endpoint = None
            self.start_segment()
            return
        if (self.last_status.get("status") in (
                "planner_unavailable", "controller_unavailable", "stale_odom", "stale_costmap", "stale_local_costmap",
                "execution_ready", "planning_only", "confirmed_standoff_update_deferred", "execution_unavailable")
                and self.blocked_endpoint is None
                and not self.query_reset_pending
                and self.route is not None and not self.pending and not self.following
                and self.input_freshness_issue() is None
                and self.execution_transport_issue() is None
                and (not self.follow_enabled or self.local_freshness_issue() is None)
                and self.plan_client.wait_for_server(timeout_sec=0.0)
                and (not self.follow_enabled
                     or self.follow_client.wait_for_server(timeout_sec=0.0))):
            self.start_segment()

    def on_parameters(self, parameters):
        reason = geometry_restart_reason(parameters, self.get_parameter)
        if reason:
            return SetParametersResult(successful=False, reason=reason)
        follow = self.follow_enabled
        compute_path_timeout = self.compute_path_timeout
        for parameter in parameters:
            if parameter.name == "follow_path":
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(successful=False, reason="follow_path must be bool")
                follow = parameter.value
            elif parameter.name == "compute_path_timeout_sec":
                try:
                    compute_path_timeout = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False,
                        reason="compute_path_timeout_sec must be a positive number")
                if not math.isfinite(compute_path_timeout) or compute_path_timeout <= 0:
                    return SetParametersResult(successful=False,
                        reason="compute_path_timeout_sec must be a positive number")
            elif parameter.name != "use_sim_time":
                return SetParametersResult(successful=False,
                    reason="restart bridge to change " + parameter.name)
        self.compute_path_timeout = compute_path_timeout
        if follow != self.follow_enabled:
            self.retire_follow_transactions()
            self.stall_anchor = None
            self.finish_execution_feedback(False, 'execution_mode_changed')
            self.epoch += 1
            self.cancel_goal(self.plan_handle, "execution_mode_compute_path")
            self.cancel_goal(self.follow_handle, "execution_mode_follow_path")
            self.plan_handle = self.follow_handle = None
            self.plan_request_started_at = None
            self.pending = self.following = False
            self.candidate_waiting_for_map = False
            self.motion_queue = []
            self.retired_motion = None
            self.prepared_extension = None
            self.extending_path = self.replacement_pending = False
            self.follow_enabled = follow
            if follow and self.follow_client is None:
                self.follow_client = ActionClient(self, FollowPath, "/follow_path")
            self.last_plan_start = self.last_plan_endpoint = None
            self.status("execution_ready" if follow else "planning_only")
        return SetParametersResult(successful=True)

    def pose_blockers(self, grid, position, heading=0.0):
        if self.carrier_model == "ideal_point":
            return point_blockers(grid, position)
        return footprint_blockers(grid, position, heading)

    def vehicle_pose_feasible(self, position, heading, tolerance=1.0):
        """Can the planner finish at/near this graph pose with the real body?

        The planner tolerance allows the returned path to stop short of the
        requested endpoint. Rejecting a goal solely because its exact footprint
        collides would discard usable samples (093254 candidate 5 succeeded
        from a nearby pose). Search the same bounded neighbourhood instead.
        """
        if not self.pose_blockers(self.grid, position, heading):
            return True
        step = .2
        limit = int(round(tolerance / step))
        for dx in range(-limit, limit + 1):
            for dy in range(-limit, limit + 1):
                offset = (dx * step, dy * step)
                if offset == (0., 0.) or math.hypot(*offset) > tolerance + 1e-9:
                    continue
                if not self.pose_blockers(
                        self.grid, (position[0] + offset[0], position[1] + offset[1]), heading):
                    return True
        return False

    def failed_goal_context(self, start_heading=None):
        """An empty search is reusable only for the same vehicle/map geometry."""
        grid = self.grid
        if grid is not self.failed_goal_grid_message:
            self.failed_goal_grid_message = grid
            if grid is None:
                self.failed_goal_grid_signature = None
            else:
                info, origin = grid.info, grid.info.origin
                self.failed_goal_grid_signature = (
                    grid.header.frame_id, info.width, info.height, info.resolution,
                    origin.position.x, origin.position.y, origin.position.z,
                    origin.orientation.x, origin.orientation.y,
                    origin.orientation.z, origin.orientation.w, hash(tuple(grid.data)))
        if start_heading is None:
            start_heading = yaw(self.odom.pose.pose) if self.odom is not None else 0.
        bins = int(self.get_parameter('heading_quantization_bins').value)
        start_bin = (int(round(math.remainder(start_heading, 2*math.pi)
                               * bins / (2*math.pi))) % bins
                     if self.carrier_model == 'ackermann' else None)
        return self.carrier_model, start_bin, self.failed_goal_grid_signature

    def failed_goal_key(self, start, goal, heading, *, context=None):
        # 0.5 m goal/heading buckets and a 2 m start region. A moved vehicle, a
        # new query or a new exploration owner must be allowed to retry a
        # previously failed endpoint; this is not a time-based or permanent
        # blacklist. (on_query clears the cache explicitly.)
        return (context if context is not None else self.failed_goal_context(),
                int(round(goal[0] / .5)), int(round(goal[1] / .5)),
                int(round(heading / .5)),
                int(round(start[0] / 2.)), int(round(start[1] / 2.)))

    def sync_failed_goal_scope(self, identity):
        if identity != self.failed_goal_scope:
            self.failed_goal_scope = identity
            self.failed_goals.clear()

    def goal_recently_failed(self, start, goal, heading):
        return self.failed_goal_key(start, goal, heading) in self.failed_goals

    def remember_failed_goal(self, start, goal, heading, *, context=None):
        current = self.failed_goal_context()
        # The planner's exact internal map version is not observable here.
        # If our map changed during the request, do not label either map with
        # that failure. A header-only republication keeps the same signature.
        if context is not None and context[-1] != current[-1]:
            return
        key = self.failed_goal_key(start, goal, heading, context=context)
        self.failed_goals.pop(key, None)
        self.failed_goals[key] = True
        while len(self.failed_goals) > 128:
            self.failed_goals.pop(next(iter(self.failed_goals)))

    def validate_path(self, points, headings, route, grid, target, *, start=None,
                      target_heading=None):
        if self.carrier_model == "ideal_point":
            return validate_ideal_path(points, headings, route, grid, target,
                corridor=self.ideal_path_corridor, start=start,
                target_heading=target_heading)
        return validate_vehicle_path(points, headings, route, grid, target,
            minimum_turning_radius=self.minimum_turning_radius,
            heading_quantization_bins=int(self.get_parameter("heading_quantization_bins").value),
            start=start, target_heading=target_heading)

    def validate_local_path(self, points, headings, grid, start, start_heading):
        if self.carrier_model == "ideal_point":
            return validate_ideal_local_prefix(points, grid, start)
        return validate_local_prefix(points, headings, grid, start, start_heading)

    def motion_completion_contract(self, points, headings, section_count):
        target=points[-1]
        observation_heading=self.exploration_observation_heading()
        observing=(observation_heading is not None and self.route_endpoint is not None
            and distance(target,self.route_endpoint)<.25
            and abs(math.remainder(headings[-1]-observation_heading,2*math.pi))<=.2)
        explicit=(self.expected is not None and distance(target,self.expected)<.25)
        cusp=self.carrier_model=='ackermann' and section_count>1
        checker=('cusp_goal_checker' if cusp else 'maneuver_goal_checker'
            if section_count>1 or self.recovering or observing or explicit else 'goal_checker')
        position=(ACKERMANN_CUSP_POSITION_TOLERANCE_M if cusp else
            MANEUVER_COMPLETION_POSITION_TOLERANCE_M if checker=='maneuver_goal_checker'
            else NORMAL_GOAL_CHECKER_POSITION_TOLERANCE_M)
        return checker,position,.7 if checker=='goal_checker' else .35

    def tracking_issue(self, points, headings, section_count):
        if self.carrier_model!='ackermann': return None
        _,position_tolerance,heading_tolerance=self.motion_completion_contract(points,headings,section_count)
        try:
            result=preview_tracking(points,headings,xy(self.odom.pose.pose),yaw(self.odom.pose.pose),
                self.minimum_turning_radius,position_tolerance=position_tolerance,
                heading_tolerance=heading_tolerance)
        except ValueError:
            return 'invalid_tracking_preview_input'
        return result.reason

    def initial_tracking_issue(self, points, headings):
        if self.carrier_model!='ackermann': return None
        sections=motion_sections(points)
        if not sections: return 'tracking_section_empty'
        first,last=sections[0]
        return self.tracking_issue(points[first:last+1],headings[first:last+1],len(sections))

    def retry_graph_reset(self):
        if not self.graph_reset_may_dispatch():
            return
        if self.instance_release_pending is not None:
            self.retry_instance_release()
            return
        client = self.graph_search_reset_client if self.graph_reset_search else self.graph_reset_client
        operation = 'reset_search' if self.graph_reset_search else 'release_goal'
        if not client.service_is_ready():
            if self.last_status.get("status") != "graph_reset_unavailable":
                self.status("graph_reset_unavailable", query=self.query, operation=operation)
            return
        generation = self.query_generation
        self.graph_reset_serial += 1
        serial = self.graph_reset_serial
        self.graph_reset_current_serial = serial
        fence = self.confirmation_not_before_ns
        self.graph_reset_attempts += 1
        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            self.graph_reset_retry_at = time.monotonic()+1.
            self.status("graph_reset_error", detail=str(exc)[:80])
            return
        self.query_reset_inflight = True
        self.graph_reset_request = dict(serial=serial,started_at=time.monotonic(),operation=operation)
        self.graph_reset_waiting[serial] = future
        self.add_navigation_done_callback(future,
            lambda f: self.on_graph_reset_done(f, generation, fence, serial))

    def on_graph_reset_done(self, future, generation, confirmation_fence=None, serial=None):
        if serial is not None:
            if serial not in self.graph_reset_waiting:
                return  # A consumed response has no authority over a new goal.
            try:
                response_success = future.result().success is True
            except Exception:
                # A transport exception cannot prove that a queued clear
                # request has finished executing on the server.
                response_success = None
            if response_success is not None:
                self.graph_reset_waiting.pop(serial)
            current = (serial == self.graph_reset_current_serial
                       and (generation,confirmation_fence) == self.graph_reset_scope())
            if current:
                self.graph_reset_request = None
                self.query_reset_inflight = False
                if response_success is True:
                    self.graph_reset_success = self.graph_reset_scope()
                else:
                    self.graph_reset_retry_at = time.monotonic()+1.
                    self.status('graph_reset_error',query=self.query)
            if not self.finish_graph_reset_barrier():
                return
            # Completion uses the current intent, never the old response scope.
            generation,confirmation_fence = self.graph_reset_scope()
        if (generation != self.query_generation or self.instance_release_pending is not None
                or (confirmation_fence is not None
                    and confirmation_fence != self.confirmation_not_before_ns)):
            return
        self.query_reset_inflight = False
        try:
            success = serial is not None or future.result().success
        except Exception:
            success = False
        if not success:
            self.status("graph_reset_error", query=self.query)
            return
        self.query_reset_pending = False
        self.minimum_route_stamp = self.get_clock().now().nanoseconds
        if self.object_found:
            return
        if self.pending_destination is not None:
            goal, self.pending_destination = self.pending_destination, None
            self.relay_destination(goal)
        if self.memory_goal is not None and self.odom is not None and distance(
                xy(self.odom.pose.pose), self.memory_goal) < 1.0:
            self.status("already_at_goal", goal=list(self.memory_goal), requires_visual_confirmation=True)
            self.await_object_observation()
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
        self.consume_pending_odometry()
        if self.check_visible_reobserve_deadline():
            return
        if self.check_visible_approach_deadline():
            return
        if self.hold_revision:
            self.publish_execution_hold(self.follow_hold)
        if self.follow_quarantine:
            self.monitor_follow_transactions()
            return
        if self.follow_release is not None:
            self.monitor_follow_transactions()
            return
        if self.monitor_frontier_choice():
            return
        if self.resume_planning_escape():
            return
        if not self.pending and not self.following:
            return
        issue = self.input_freshness_issue()
        if issue is None and self.following:
            issue = self.local_freshness_issue()
        transport_issue = self.execution_transport_issue()
        if issue is None and transport_issue is not None:
            self.execution_transport_unavailable(transport_issue)
            return
        if issue is None:
            if self.monitor_follow_transactions():
                return
            if (self.plan_request_started_at is not None
                    and time.monotonic() - self.plan_request_started_at >= self.compute_path_timeout):
                self.recover_timed_out_plan()
                return
            # Pure pursuit can request extra curvature while correcting a
            # small error on a valid arc. The actuator bounds that request;
            # saturation alone does not prove the path is infeasible. Monitor
            # measured tracking here and let Nav2's progress checker detect a
            # stalled maneuver instead of repeatedly canceling healthy motion.
            if self.following and self.motion_queue:
                points = [xy(p.pose) for p in self.motion_queue[0].poses]
                error = distance_to_route(xy(self.odom.pose.pose), points)
                if error > self.tracking_error_limit:
                    self.cancel_goal(self.follow_handle, "tracking_deviation")
                    self.follow_failed("tracking_deviation", cross_track_error=round(error, 3))
                    return
                stalled = self.execution_stall()
                if stalled is not None:
                    self.cancel_goal(self.follow_handle, 'execution_stalled')
                    self.follow_failed('execution_stalled', **stalled)
                    return
            self.maybe_extend_path()
            return
        self.finish_execution_feedback(False, 'stale_input')
        self.retire_follow_transactions()
        self.epoch += 1
        self.cancel_goal(self.plan_handle, "stale_input_compute_path")
        self.cancel_goal(self.follow_handle, "stale_input_follow_path")
        self.plan_handle = self.follow_handle = None
        self.plan_request_started_at = None
        self.pending = self.following = False
        self.candidate_waiting_for_map = False
        self.motion_queue = []
        self.retired_motion = None
        self.prepared_extension = None
        self.extending_path = self.replacement_pending = False
        self.recovering = False
        self.escape_path = None
        self.clear_exploration_comparison()
        self.last_plan_start = self.last_plan_endpoint = None
        self.report_stale_input(issue)

    def reset(self, preserve_execution_corridor=False):
        self.consume_pending_odometry()
        self.retired_motion = None
        self.clear_exploration_comparison()
        corridor = ((self.escape_trace, self.escape_failures, self.escape_attempt_position,
                     self.planning_escape_failures,self.deferred_planning_escape) if preserve_execution_corridor else None)
        self.retire_follow_transactions()
        self.init_execution_recovery()
        if corridor is not None:
            (self.escape_trace, self.escape_failures, self.escape_attempt_position,
             self.planning_escape_failures,self.deferred_planning_escape) = corridor
        self.deferred_follow_result = None
        self.finish_execution_feedback(False, 'navigation_reset')
        self.extending_path = self.replacement_pending = False
        self.prepared_extension = None
        self.recovery_attempts = 0
        self.recovery_progress = None
        self.normal_best = None
        self.stall_anchor = None
        self.active_plan_start = self.active_plan_failure_context = None
        self.epoch += 1
        self.motion_queue = []
        self.pending = False
        self.candidate_waiting_for_map = False
        plan_handle = self.plan_handle
        self.plan_handle = None
        self.plan_request_started_at = None
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

    def receive_odometry(self, message):
        """Receive independently; only the navigation group applies the poses."""
        if message.header.frame_id != 'spot1/odom' or not valid_pose(message.pose.pose):
            return
        with self.odom_input_lock:
            previous=max(self.odom_input_latest_stamp,
                         stamp_ns(self.odom) if self.odom is not None else 0)
            age=measurement_age(message,self.get_clock().now().nanoseconds,
                                self.odom_timeout,previous,future_tolerance_sec=.05)
            if age is None:
                return
            self.odom_input_latest_stamp=stamp_ns(message)
            self.odom_input_pending.append(message)
        self.odom_input_guard.trigger()

    def consume_pending_odometry(self):
        # Applying the bounded batch here retains motion/view history in the
        # navigation group. Ingress never changes path, query or action state.
        with self.odom_input_lock:
            pending=list(self.odom_input_pending)
            self.odom_input_pending.clear()
        for message in pending:
            self.on_odom(message)  # Recheck source age; receipt cannot renew it.

    def on_odom(self, message):
        if message.header.frame_id != 'spot1/odom' or not valid_pose(message.pose.pose):
            return
        age = measurement_age(message, self.get_clock().now().nanoseconds,
                              self.odom_timeout, stamp_ns(self.odom) if self.odom else None,
                              future_tolerance_sec=0.05)
        if age is None:
            return
        self.observe_execution_motion(self.odom, message)
        self.odom = message
        self.odom_received_at = time.monotonic() - age
        if self.query is not None and self.query_origin is None:
            self.query_origin = xy(message.pose.pose)
        self.remember_driven_pose(message)

    def on_map(self, message):
        if not valid_grid(message, 'spot1/odom'):
            return
        age = measurement_age(message, self.get_clock().now().nanoseconds,
                              self.costmap_timeout, stamp_ns(self.grid) if self.grid else None)
        if age is None:
            return
        self.grid = message
        self.grid_received_at = time.monotonic() - age
        self.grid_arrived_at = time.monotonic()
        if self.candidate_waiting_for_map and self.pending:
            self.candidate_waiting_for_map = False
            stale = self.input_freshness_issue()
            if stale:
                self.pending = False
                self.report_stale_input(stale)
                return
            self.try_candidate()
        self.retry_cleared_start_footprint()

    def retry_cleared_start_footprint(self):
        """Resume the same route when a fresh map clears a blocked vehicle pose.

        A blocked start normally gets a 20 s backoff. Costmaps can clear the
        vehicle footprint much sooner, so wait for fresh, mutually consistent
        inputs and let start_segment repeat its complete planning/execution
        checks instead of waiting for a timer or another GraphNav route.
        """
        status = self.last_status
        if (status.get('status') != 'start_footprint_blocked'
                or status.get('navigation_epoch') != self.epoch
                or status.get('query_generation') != self.query_generation
                or self.blocked_endpoint is None or self.route is None
                or not self.route.poses or self.route_endpoint is None
                or distance(self.route_endpoint, self.blocked_endpoint) > .5):
            return False
        if (not self.follow_enabled or self.query_reset_pending
                or self.instance_release_pending is not None
                or self.follow_quarantine or self.follow_release is not None
                or self.follow_request is not None or self.follow_active is not None
                or self.follow_hold or self.pending or self.following
                or self.recovering or self.replacement_pending or self.extending_path
                or self.frontier_choice is not None
                or self.confirmed_approach_failure is not None
                or self.execution_recovery_owns_motion()):
            return False
        if self.input_freshness_issue() is not None or self.local_freshness_issue() is not None:
            return False
        if self.execution_transport_issue() is not None:
            return False
        if (self.plan_client is None or self.follow_client is None
                or not self.plan_client.wait_for_server(timeout_sec=0.0)
                or not self.follow_client.wait_for_server(timeout_sec=0.0)):
            return False

        start = xy(self.odom.pose.pose)
        heading = yaw(self.odom.pose.pose)
        if (self.pose_blockers(self.grid, start, heading)
                or self.pose_blockers(self.local_grid, start, heading)):
            return False

        reported_goal = status.get('goal')
        current_goal = (self.expected or self.exploration_destination()
                        or xy(self.route.poses[-1].pose))
        if (not isinstance(reported_goal, (list, tuple)) or len(reported_goal) != 2
                or distance(tuple(reported_goal), current_goal) > .25):
            return False

        goal = tuple(current_goal)
        self.blocked_endpoint = self.blocked_route = self.blocked_grid_hash = None
        self.blocked_until = 0.0
        self.last_plan_start = self.last_plan_endpoint = None
        self.status('blocked_start_cleared', goal=list(goal),
                    trigger='fresh_costmap_footprint_clear')
        self.start_segment()
        return True

    def input_freshness_issue(self):
        return freshness_issue(time.monotonic(), self.odom_received_at,
                               self.grid_received_at, self.odom_timeout,
                               self.costmap_timeout)

    def on_local_map(self, message):
        if not valid_grid(message, 'spot1/odom'):
            return
        age = measurement_age(message, self.get_clock().now().nanoseconds,
                              self.local_costmap_timeout,
                              stamp_ns(self.local_grid) if self.local_grid else None)
        if age is None:
            return
        self.local_grid = message
        self.local_grid_received_at = time.monotonic()-age

    def local_freshness_issue(self):
        received = self.local_grid_received_at
        age = time.monotonic()-received if received is not None else None
        if self.local_grid is None or age is None or age < 0 or age > self.local_costmap_timeout:
            return ('stale_local_costmap', age)
        return None

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
            self.get_logger().warning("Cancelling %s action, epoch=%d pending=%s following=%s" %
                                      (label, self.epoch, self.pending, self.following))
            handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warning("could not cancel %s goal: %s" % (label, exc))

    def recover_timed_out_plan(self):
        old_epoch, old_index = self.epoch, self.candidate_index
        handle = self.plan_handle
        self.plan_handle = None
        self.plan_request_started_at = None
        self.candidate_waiting_for_map = False
        self.pending = False
        # Retire only this planner transaction. The accepted FollowPath still
        # belongs to the navigation epoch and must deliver its terminal result.
        self.active_plan_serial = None
        self.cancel_goal(handle, "compute_path_timeout")
        self.get_logger().warning(
            "ComputePath timed out: sent_epoch=%d candidate=%d timeout_sec=%.2f" %
            (old_epoch, old_index, self.compute_path_timeout))

        if self.extending_path:
            self.extension_failed('planner_timeout')
            return

        current_endpoint = self.route_endpoint
        newest = self.latest_route
        if (newest is not None and newest.poses and current_endpoint is not None
                and distance(xy(newest.poses[-1].pose), current_endpoint) >= 4.0):
            self.route = newest
            self.route_complete = self.latest_route_complete
            self.route_endpoint = xy(newest.poses[-1].pose)
            self.last_plan_start = self.last_plan_endpoint = None
            self.sync_recovery_progress()
            self.status("planner_timeout_using_latest_route",
                        previous_candidate=old_index, goal=list(self.route_endpoint))
            self.start_segment()
            return
        self.candidate_failed("planner_timeout")

    def route_diagnostic(self, reason, *, stamp=0, age=None):
        # GraphNav can republish the same route many times per second. Keep
        # enough runtime evidence to distinguish stale/unsynchronized input
        # from a planner that never produced a route without flooding logs.
        now = time.monotonic()
        if now - self.route_diagnostic_log_at < 2.0:
            return
        self.route_diagnostic_log_at = now
        self.get_logger().warning("GraphNav route ignored: " + json.dumps({
            "reason": reason, "stamp_ns": stamp, "age_sec": age,
            "query": self.query, "query_generation": self.query_generation,
            "reset_pending": self.query_reset_pending,
            "route_grace_remaining_sec": round(max(0., self.query_grace_until-now), 3),
            "minimum_route_stamp_ns": self.minimum_route_stamp,
            "has_odom": self.odom is not None, "has_costmap": self.grid is not None,
            "costmap_frame": self.grid.header.frame_id if self.grid is not None else None,
            "pending": self.pending, "following": self.following,
            "follow_enabled": self.follow_enabled,
            "last_status": self.last_status.get('status'),
        }))

    def on_query(self, message):
        query = " ".join(message.data.casefold().split())
        if not query or query == self.query:
            return
        self.get_logger().info("New query resets navigation: %s -> %s, epoch=%d" %
                               (self.query, query, self.epoch))
        self.query = query
        self.init_visible_approach()
        self.init_confirmed_approach()
        self.query_generation += 1
        self.frontier_compared.clear()
        self.frontier_protocol_failures = 0
        self.object_found = False
        self.query_origin = (xy(self.odom.pose.pose)
            if self.odom is not None and self.odom_received_at is not None
            and time.monotonic() - self.odom_received_at <= self.odom_timeout else None)
        self.nearest_signal_position = None
        self.nearest_signal_distance = None
        self.nearest_signal_standoff = None
        self.active_object_confirmation = None
        self.init_observation()
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.route_metadata = {}
        self.waiting_routes = {}
        # Planner no-path memory belongs to the previous query's goals. It is
        # not time based, so a new query must clear it explicitly.
        self.failed_goals.clear()
        self.failed_goal_scope = None
        self.reset()
        self.clear_memory_selection()
        self.memory_goal = self.expected = None
        self.memory_heading = None
        self.pending_destination = None
        self.request_graph_reset(reset_search=True)
        # GraphNav can explore frontiers before object triangulation publishes
        # a waypoint. Briefly suppress the preceding query's stale route.
        self.query_grace_until = time.monotonic() + 2.0
        self.status("awaiting_graph_route", query=query)
        self.retry_graph_reset()
        self.activate_pending_memory()

    def on_waypoint(self, message, confirmed_first=False, confirmed_observation=False):
        if (self.upstream_target_approach and self.active_object_confirmation is not None
                and not confirmed_observation):
            return  # An unlabelled raw point cannot replace a confirmed target.
        if self.upstream_target_approach and self.memory_selecting:
            return
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if (self.object_found or self.instance_release_pending is not None
                or stamp < max(self.query_started_ns, self.confirmation_not_before_ns) or stamp > now_ns
                or now_ns - stamp > 5000000000):
            return
        point = xy(message.pose)
        if message.header.frame_id != "spot1/odom" or not all(map(math.isfinite, point)):
            return
        if self.memory_goal is not None:
            # Unlabelled legacy waypoints cannot identify a remembered object.
            # Query-scoped confirmations can finish from any new observer pose.
            return
        if self.confirmed_route_goal is not None:
            return  # Validated confirmations own refinement of this route.
        if self.recently_unconfirmed(point):
            return
        if self.visible_approach_active or self.visible_reobserve_exploration:
            self.reset()
            self.init_visible_approach()
            self.expected = None
        if (is_traffic_light_query(self.query)
                and self.nearest_signal_position is None
                and self.query_origin is not None):
            # A query waypoint carries the approach bearing. Use its configured
            # 4 m standoff to establish the initial nearest-signal reference;
            # later confirmed instances may replace it only when genuinely
            # closer to the query origin.
            heading = yaw(message.pose)
            hinted_target = (point[0] + NEAREST_SIGNAL_STANDOFF_M * math.cos(heading),
                             point[1] + NEAREST_SIGNAL_STANDOFF_M * math.sin(heading))
            hinted_distance = distance(self.query_origin, hinted_target)
            if hinted_distance > NEAREST_SIGNAL_STANDOFF_M:
                self.nearest_signal_position = hinted_target
                self.nearest_signal_distance = hinted_distance
                self.nearest_signal_standoff = point
                self.status('nearest_signal_hint_latched',
                            selected_object=list(hinted_target),
                            selected_object_distance=hinted_distance,
                            standoff=list(point))
        if (is_traffic_light_query(self.query)
                and self.nearest_signal_position is not None
                and self.nearest_signal_standoff is not None
                and distance(point, self.nearest_signal_standoff) > 3.0):
            # A farther signal can produce another waypoint after the current
            # route ends. Keep the query's nearest confirmed instance latched.
            return
        if self.expected is not None and distance(self.expected, point) > 3.0:
            # A text query can match several lights/signs in one frame, so its
            # triangulated waypoint may jump between instances. Keep the
            # selected instance while its checked route is active and during
            # the short visual-arrival window; a new candidate may replace it
            # after the route finishes unconfirmed or planning becomes idle.
            if not confirmed_first and (self.following or self.pending or self.observation_wait_until is not None):
                return
        if self.expected is None or distance(self.expected, point) > 3.0:
            self.reset()
            self.observation_wait_until = None
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

    def promote_nearest_signal(self, data):
        """Latch the nearest supported signal and let its route replace staging."""
        if not is_traffic_light_query(self.query) or self.object_found:
            return False
        origin = self.query_origin
        if origin is None and self.odom is not None:
            origin = xy(self.odom.pose.pose)
            self.query_origin = origin
        if origin is None:
            return False
        object_position = tuple(map(float, data['object_position'][:2]))
        candidate_distance = distance(origin, object_position)
        same_target = (self.nearest_signal_position is not None
                       and distance(self.nearest_signal_position, object_position) <= 3.0)
        nearer_target = (self.nearest_signal_distance is None
                         or candidate_distance + NEAREST_SIGNAL_SWITCH_MARGIN_M
                            < self.nearest_signal_distance)
        if not same_target and not nearer_target:
            return False
        standoff = tuple(map(float, data['standoff']))
        if self.recently_unconfirmed(standoff):
            return False

        previous_target = self.nearest_signal_position
        previous_goal = self.expected
        selected_new_target = not same_target
        self.nearest_signal_position = object_position
        self.nearest_signal_distance = candidate_distance
        self.nearest_signal_standoff = standoff
        goal_progress = (previous_goal is None or
            distance(origin, standoff) + NEAREST_SIGNAL_GOAL_PROGRESS_M
                < distance(origin, previous_goal))
        if not (selected_new_target or goal_progress):
            return True
        if previous_goal is not None and distance(previous_goal, standoff) <= 3.0:
            return True

        goal = PoseStamped()
        goal.header.frame_id = 'spot1/odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = standoff
        goal.pose.position.z = float(self.odom.pose.pose.position.z)
        heading = math.atan2(object_position[1] - standoff[1],
                             object_position[0] - standoff[0])
        goal.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.orientation.w = math.cos(heading / 2.0)
        self.reset()
        self.observation_wait_until = None
        self.expected = standoff
        self.query_grace_until = 0.0
        self.relay_destination(goal)
        self.status('nearest_signal_route_preempted',
                    selected_object=list(object_position),
                    selected_object_distance=candidate_distance,
                    previous_object=list(previous_target) if previous_target else None,
                    previous_goal=list(previous_goal) if previous_goal else None,
                    goal=list(standoff))
        return True

    def on_object_confirmation(self, message):
        self.consume_pending_odometry()
        if self.object_found:
            return
        try:
            data = json.loads(message.data)
            now_ns = self.get_clock().now().nanoseconds
            stamp = int(data['observation_stamp_ns'])
            approach_refresh = data.get('approach_refresh') is True
            if self.upstream_target_approach and approach_refresh:
                return  # Retained geometry is not a new original 3D observation.
            if (data['query'] != self.query
                    or stamp < max(self.query_started_ns, self.confirmation_not_before_ns,
                                   self.confirmation_evidence_not_before_ns) or stamp > now_ns):
                return
            if data.get('evidence_rejected') is True:
                if (data.get('frame') != 'spot1/odom'
                        or data.get('localization_valid') is not False
                        or now_ns-stamp > 5_000_000_000
                        or data.get('reason') not in ('ambiguous_depth_surfaces',
                            'instance_mismatch', 'current_observation_outside_cluster')):
                    return
                anchor = self.active_object_confirmation or {}
                instance_id = data.get('instance_id', '')
                if (not isinstance(instance_id, str)
                        or stamp < anchor.get('observation_stamp_ns', 0)
                        or (instance_id and anchor.get('instance_id')
                            and instance_id != anchor['instance_id'])):
                    return
                # Keep the accepted path and its timer ownership. Only revoke
                # success/refresh evidence; endpoint observation and the
                # existing release/ACK transaction handle bounded recovery.
                # Depth ambiguity revokes only 3D/arrival evidence. The mask's
                # bearing at the same acquisition time remains valid guidance.
                # Keep this fence separate from release/ACK, which revokes all
                # evidence for an abandoned instance, including bearing hints.
                self.confirmation_evidence_not_before_ns = stamp+1
                if instance_id:
                    self.upstream_instance_id = instance_id
                deferred = self.deferred_confirmed_approach
                if deferred and deferred.get('observation_stamp_ns', 0) <= stamp:
                    self.deferred_confirmed_approach = None
                self.status('target_evidence_rejected', query=self.query,
                            reason=data['reason'], observation_stamp_ns=stamp)
                if not self.upstream_target_approach and self.replacement_requires_release(data):
                    self.status('target_instance_superseded', query=self.query,
                                instance_id=instance_id,
                                replacement_candidate=data['replacement_candidate'])
                    # Do not plan to this candidate or label it as arrived.
                    # Stop/release the old owner first; ACK fences require new
                    # observations before the next instance can be confirmed.
                    self.release_unconfirmed_target(reason='instance_superseded')
                return
            if self.memory_selecting or self.instance_release_pending is not None:
                # Candidate comparison is not a selected destination. Negative
                # evidence still establishes the fence for delayed confirmations.
                return
            anchor = self.active_object_confirmation
            if (self.upstream_target_approach and anchor and anchor.get('instance_id')
                    and data.get('instance_id') != anchor['instance_id']):
                return
            if approach_refresh:
                anchor = self.active_object_confirmation
                update_stamp = int(data.get('approach_update_stamp_ns', 0))
                base_stamp = int(data.get('base_observation_stamp_ns', -1))
                update_age = now_ns - update_stamp
                if (anchor is None or anchor.get('query') != self.query
                        or base_stamp != anchor.get('observation_stamp_ns')
                        or stamp != anchor.get('observation_stamp_ns')
                        or not -50000000 <= update_age <= 5000000000
                        or planar_distance(data.get('object_position'),
                                           anchor.get('object_position')) > 1.0):
                    return
            elif now_ns - stamp > 5000000000:
                return
            if data['frame'] != 'spot1/odom' or data.get('search_confirmed') is not True:
                return
            quality = data.get('quality')
            if (data.get('localization_valid') is not True
                    or not isinstance(quality, dict)
                    or not math.isfinite(quality.get('confidence', 0.))
                    or quality.get('confidence', 0.) < .5
                    or quality.get('association_validated') is not True):
                return
            if (len(data['object_position']) != 3 or len(data['observer_position']) != 3
                    or not all(map(math.isfinite, [*data['object_position'], *data['observer_position'], data['radius']]))
                    or not 0 <= data['radius'] <= 3.
                    or not isinstance(data['inliers'], int) or data['inliers'] < 3
                    or not isinstance(data['observations'], int)
                    or data['observations'] < data['inliers']):
                return
            standoff = data.get('standoff')
            valid_standoff = (isinstance(standoff, (list, tuple)) and len(standoff) == 2
                              and all(map(math.isfinite, standoff)))
            navigation_heading=data.get('standoff_navigation_heading')
            if (navigation_heading is not None and (data.get('standoff_clamped') is not True
                    or not isinstance(navigation_heading,(int,float))
                    or not math.isfinite(navigation_heading))):
                return
            if (self.odom is None or self.odom_received_at is None
                    or time.monotonic() - self.odom_received_at > self.odom_timeout):
                self.status('confirmation_waiting_for_vehicle_pose', query=self.query)
                return
            if (is_traffic_light_query(self.query)
                    and not self.promote_nearest_signal(data)):
                # Do not let a farther confirmed signal finish the query after
                # the route has latched a nearer instance.
                return
            vehicle_position = xy(self.odom.pose.pose)
            vehicle_heading = yaw(self.odom.pose.pose)
            reference_distance = planar_distance(vehicle_position, data['object_position'][:2])
            object_distance = vehicle_body_distance(vehicle_position, vehicle_heading,
                data['object_position'], self.carrier_model)
            arrival_parameter = ('traffic_light_arrival_distance'
                                 if is_traffic_light_query(self.query)
                                 else 'object_arrival_distance')
            arrival_distance = float(self.get_parameter(arrival_parameter).value)
            if not math.isfinite(arrival_distance) or arrival_distance <= 0:
                return
            effective_arrival_distance = min(arrival_distance, BODY_OBJECT_ARRIVAL_DISTANCE_M)
            if not math.isfinite(effective_arrival_distance):
                return
            arrived = within_body_arrival(object_distance, effective_arrival_distance)
            if not valid_standoff and not arrived and not self.upstream_target_approach:
                self.status('confirmed_target_waiting_for_approach', query=self.query,
                            object_distance_m=object_distance,
                            reference_point_distance_m=reference_distance,
                            approach_goal_ready=False)
                return
            if not approach_refresh and isinstance(data.get('instance_id'), str):
                # Record only producer identity, even when its geometry is
                # outside a memory region and cannot be accepted as arrival.
                self.upstream_instance_id = data['instance_id']
            memory_distance = None
            memory_tolerance = None
            memory_anchor = None
            if self.memory_active and self.memory_active.get('object_position'):
                memory_anchor = self.memory_active.get('confirmation_anchor')
                region_view = self.memory_active
                if (isinstance(memory_anchor, dict)
                        and memory_anchor.get('object_position')):
                    # A passive memory point is a coarse seed. Once fresh
                    # query-scoped geometry matches it, use that live target
                    # estimate as the local association anchor. Large objects
                    # can expose different facade patches several metres apart
                    # as the vehicle approaches; comparing every patch to the
                    # original passive point can reject the same target.
                    region_view = dict(
                        self.memory_active,
                        object_position=memory_anchor['object_position'],
                        object_radius=memory_anchor.get('object_radius', 0.0),
                        evidence='query_confirmation_anchor')
                memory_distance, memory_tolerance = memory_region_metrics(
                    data['object_position'][:2], data['radius'], region_view)
                if (memory_distance > memory_tolerance
                        and self.memory_active.get('evidence') == 'passive_semantic_landmarks'):
                    self.status('confirmation_outside_memory_region', query=self.query,
                                memory_region=region_view['object_position'][:2],
                                observed_position=data['object_position'][:2],
                                distance=memory_distance, tolerance=memory_tolerance)
                    return
                if (memory_anchor is None
                        and self.memory_active.get('evidence') == 'passive_semantic_landmarks'):
                    # Latch the first geometrically supported match to the
                    # remembered region. Subsequent nearby observations can
                    # refine different visible patches without switching to a
                    # different passive landmark instance.
                    memory_anchor = self.memory_active['confirmation_anchor'] = {
                        'object_position': data['object_position'][:2],
                        'object_radius': data['radius'],
                        'observation_stamp_ns': data['observation_stamp_ns']}
            if (not self.confirmed_instance_matches(data)
                    and self.confirmed_route_busy()):
                self.status('confirmed_other_instance_deferred', query=self.query,
                            selected_object=list(self.confirmed_route_object),
                            observed_position=data['object_position'][:2])
                return
            first_confirmed = self.active_object_confirmation is None
            if not approach_refresh:
                self.init_visible_approach()
                self.active_object_confirmation = {
                    'query': self.query,
                    'observation_stamp_ns': stamp,
                    'object_position': data['object_position'][:2],
                    'instance_id': data.get('instance_id', ''),
                }
            if self.upstream_target_approach and not arrived:
                # The original triangulator publishes this same world point.
                # A matching recalled landmark can hand off its viewpoint to
                # the live target without changing the selected recall record.
                self.memory_goal = None
                self.memory_arrival_heading = None
                goal = PoseStamped()
                goal.header.frame_id = data['frame']
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, data['object_position'])
                goal.pose.orientation.w = 1.0
                self.on_waypoint(goal, confirmed_first=first_confirmed, confirmed_observation=True)
                return
            standoff_distance = (planar_distance(vehicle_position, standoff)
                                 if valid_standoff else None)
            if approach_refresh and arrived:
                # A carried object coordinate may advance navigation, but it
                # cannot be used as fresh arrival evidence after its source
                # observation has aged out.
                self.status('approach_refresh_waiting_for_fresh_confirmation',
                            query=self.query,
                            object_distance_m=object_distance,
                            arrival_distance_m=effective_arrival_distance,
                            standoff=list(standoff) if valid_standoff else None)
                return
            if (not arrived and valid_standoff
                    and standoff_distance > OBJECT_CONFIRMATION_STANDOFF_TOLERANCE_M):
                if not self.route_confirmed_target(data, vehicle_position):
                    self.status('confirmation_before_standoff', query=self.query,
                                vehicle_position=list(vehicle_position), standoff=standoff,
                                distance=standoff_distance,
                                object_distance_m=object_distance,
                                reference_point_distance_m=reference_distance,
                                tolerance=OBJECT_CONFIRMATION_STANDOFF_TOLERANCE_M)
                return
            if not arrived:
                # A nearby observation pose is not a successful object search.
                # Keep following the selected safe standoff. In particular,
                # do not switch queries while still outside the tighter
                # standoff tolerance: the nominal map-clamped route may be the
                # only collision-checked way to get closer.
                self.status('object_visible_not_arrived', query=self.query,
                            vehicle_position=list(vehicle_position),
                            object_position=data['object_position'][:2],
                            distance=object_distance,
                            reference_point_distance_m=reference_distance,
                            arrival_distance=effective_arrival_distance,
                            configured_arrival_distance=arrival_distance,
                            standoff_clamped=data.get('standoff_clamped'),
                            standoff_object_distance_m=data.get('standoff_object_distance_m'),
                            standoff_distance_m=standoff_distance,
                            standoff_tolerance_m=OBJECT_CONFIRMATION_STANDOFF_TOLERANCE_M)
                return
        except (ValueError, KeyError, TypeError):
            return
        memory_view_id = self.memory_active.get('id') if self.memory_active else None
        memory_view_position = self.memory_active.get('object_position') if self.memory_active else None
        memory_view_radius = self.memory_active.get('object_radius') if self.memory_active else None
        memory_evidence = self.memory_active.get('evidence') if self.memory_active else None
        self.object_found = True
        self.active_object_confirmation = None
        self.reset()
        self.memory_selecting = False
        self.pending_destination = None
        self.request_graph_reset()
        self.retry_graph_reset()
        self.status('object_found', query=self.query, object_position=data['object_position'],
                    arrival_basis='vehicle_body_clearance',
                    observer_position=data['observer_position'], confirmation=data,
                    vehicle_position=list(vehicle_position), vehicle_heading=vehicle_heading,
                    reference_point_distance_m=reference_distance,
                    standoff=list(standoff) if valid_standoff else None,
                    standoff_distance_m=standoff_distance,
                    standoff_tolerance_m=OBJECT_CONFIRMATION_STANDOFF_TOLERANCE_M,
                    object_distance_m=object_distance,
                    object_arrival_tolerance_m=effective_arrival_distance,
                    configured_object_arrival_tolerance_m=arrival_distance,
                    memory_region_distance_m=memory_distance,
                    memory_region_tolerance_m=memory_tolerance,
                    memory_confirmation_anchor=(
                        memory_anchor.get('object_position')
                        if isinstance(memory_anchor, dict) else None),
                    memory_view_id=memory_view_id,
                    memory_object_position=memory_view_position,
                    memory_object_radius=memory_view_radius,
                    memory_evidence=memory_evidence)

    def on_route_status(self, message):
        try:
            data = json.loads(message.data)
            stamp = int(data['stamp_ns'])
            age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
            if (stamp <= 0 or not -.2 <= age <= 5.
                    or not isinstance(data['complete'], bool) or len(data['goal']) != 2):
                return
            if not all(map(math.isfinite, data['goal'])):
                return
            identity = data.get('exploration_id')
            if identity is not None and (not isinstance(identity, str) or not identity
                                         or len(identity) > 128):
                return
        except (ValueError, KeyError, TypeError, OverflowError):
            return
        self.route_metadata[stamp] = data
        active_stamps = {stamp_ns(p) for p in (self.route, self.latest_route) if p is not None}
        for old in sorted(self.route_metadata)[:-32]:
            if old not in active_stamps:
                self.route_metadata.pop(old, None)
        pending = self.waiting_routes.pop(stamp, None)
        if pending is not None:
            self.on_route(pending)

    def on_exploration_retired(self, message):
        try:
            data = json.loads(message.data)
            identity, stamp = data['exploration_id'], int(data['stamp_ns'])
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            if (not isinstance(identity, str) or not identity or len(identity) > 128
                    or stamp < max(self.query_started_ns, self.minimum_route_stamp)
                    or stamp <= 0 or not 0. <= age <= 5.):
                return
        except (ValueError, KeyError, TypeError, OverflowError):
            return
        self.retired_explorations[identity] = stamp
        for old in sorted(self.retired_explorations, key=self.retired_explorations.get)[:-64]:
            self.retired_explorations.pop(old, None)
        metadata = self.exploration_metadata(self.route)
        if self.retain_motion_for_retired_intent(data, metadata):
            return
        if metadata is not None and metadata['exploration_id'] == identity:
            if (data.get('reason') in ('frontier_resolved', 'frontier_missing')
                    and self.retire_background_exploration(identity)):
                return
            same_frame = data.get('reason') in (
                'observation_completed', 'vehicle_route_rejected',
                'graph_path_unavailable', 'destination_backoff', 'vehicle_cost_selection',
                'frontier_resolved', 'frontier_missing')
            if self.recovering and same_frame:
                # Escape motion owns a measured physical corridor, separately
                # from the rejected exploration destination that triggered it.
                latest = self.exploration_metadata(self.latest_route)
                if latest is not None and latest['exploration_id'] == identity:
                    self.latest_route = None
                self.status('recovery_started', retired_exploration_id=identity,
                            retirement_reason=data.get('reason'))
                return
            if (data.get('reason') in ('frontier_resolved', 'frontier_missing')
                    and self.finish_current_exploration_section(identity)):
                return
            # Reuse the cancellation quarantine: clearing local state is not
            # proof that an accepted controller action has stopped.
            choice = self.frontier_choice_for_retirement(data)
            replacement = self.replacement_route_for_retirement(identity)
            self.reset(preserve_execution_corridor=same_frame)
            if choice is not None:
                choice['epoch'] = self.epoch
                self.frontier_choice = choice
            self.status('exploration_retired', exploration_id=identity,
                        retirement_reason=data.get('reason'))
            if replacement is not None:
                # reset() retires the old action and clears old planning state.
                # Re-admit the newer owner through the normal route contract;
                # start_segment remains fenced by follow_quarantine until its
                # terminal result and measured stop barrier are observed.
                self.on_route(replacement)

    def replacement_route_for_retirement(self, retired_identity):
        """Snapshot a fresh, already-published route owned by another intent.

        A frontier_resolved retirement can race the new owner's route callback.
        Keep only the route message across reset; never carry its old action,
        candidate queue, or execution authorization across the owner boundary.
        """
        route = self.latest_route
        if (route is None or self.query_reset_pending or self.object_found
                or self.memory_selecting or self.expected is not None
                or self.memory_goal is not None or self.confirmed_approach_failure is not None
                or (self.visible_reobserve_until is not None
                    and not self.visible_reobserve_exploration)):
            return None
        metadata = self.exploration_metadata(route)
        if metadata is None:
            return None
        identity = metadata.get('exploration_id')
        stamp = stamp_ns(route)
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        if (not identity or identity == retired_identity
                or identity in self.retired_explorations
                or stamp < max(self.query_started_ns, self.minimum_route_stamp)
                or stamp <= 0 or not -.2 <= age <= 5.
                or not route.poses or self.odom is None or self.grid is None
                or route.header.frame_id != self.odom.header.frame_id
                or route.header.frame_id != self.grid.header.frame_id
                or any(not all(map(math.isfinite, xy(p.pose))) for p in route.poses)):
            return None
        return deepcopy(route)

    def on_route(self, message):
        self.consume_pending_odometry()
        if self.object_found:
            return
        if self.confirmed_approach_failure is not None:
            return  # Old-goal publications cannot bypass observation recovery.
        if (self.visible_reobserve_until is not None
                and not self.visible_reobserve_exploration):
            self.route_diagnostic('visible_target_reobserving')
            return
        if self.memory_selecting and self.memory_active is None:
            return
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if stamp <= 0 or not -.2 <= age <= 5.:
            self.route_diagnostic("stale_or_future_path", stamp=stamp, age=round(age, 3))
            return
        if self.query_reset_pending:
            self.route_diagnostic("query_reset_pending", stamp=stamp, age=round(age, 3))
            return
        if stamp < self.minimum_route_stamp:
            self.route_diagnostic("before_query_reset_stamp", stamp=stamp, age=round(age, 3))
            return
        if time.monotonic() < self.query_grace_until:
            self.route_diagnostic("query_grace", stamp=stamp, age=round(age, 3))
            return
        if not message.poses:
            self.route_diagnostic("empty_path", stamp=stamp, age=round(age, 3))
            return
        if self.odom is None or self.grid is None:
            self.route_diagnostic("missing_odom_or_costmap", stamp=stamp, age=round(age, 3))
            return
        if message.header.frame_id != self.grid.header.frame_id:
            self.route_diagnostic("costmap_frame_mismatch", stamp=stamp, age=round(age, 3))
            self.status("frame_mismatch", route_frame=message.header.frame_id,
                        costmap_frame=self.grid.header.frame_id)
            return
        endpoint = xy(message.poses[-1].pose)
        if (message.header.frame_id != self.odom.header.frame_id
                or any(not all(map(math.isfinite, xy(p.pose))) for p in message.poses)):
            self.route_diagnostic("invalid_frame_or_pose", stamp=stamp, age=round(age, 3))
            self.status("invalid_graph_route")
            return
        metadata = getattr(self, 'route_metadata', {}).get(stamp)
        if getattr(self, 'require_route_status', False) and metadata is None:
            self.route_diagnostic("missing_matching_route_status", stamp=stamp, age=round(age, 3))
            self.waiting_routes[stamp] = message
            for old in sorted(self.waiting_routes)[:-8]:
                self.waiting_routes.pop(old, None)
            return
        if metadata is not None:
            if metadata.get('exploration_id') in self.retired_explorations:
                self.route_diagnostic('retired_exploration_path', stamp=stamp, age=round(age, 3))
                return
            if self.expected is not None and distance(metadata['goal'], self.expected) > 0.25:
                return
            incoming_complete = metadata['complete']
        else:
            incoming_complete = True
            if self.expected is not None and distance(endpoint, self.expected) > 7.0:
                return
            if (self.visible_goal_refresh_pending and self.expected is not None
                    and distance(endpoint,self.expected)>.25):
                return
        self.latest_route = message
        self.latest_route_complete = incoming_complete
        self.visible_goal_refresh_pending = False
        selected_choice = self.frontier_choice
        if self.wait_for_frontier_choice(message, metadata):
            return
        if time.monotonic() - self.route_accept_log_at >= 2.0:
            self.route_accept_log_at = time.monotonic()
            self.get_logger().info("GraphNav route accepted: " + json.dumps({
                "stamp_ns": stamp, "age_sec": round(age, 3), "goal": list(endpoint),
                "complete": incoming_complete,
                "metadata_mode": metadata.get('mode') if metadata is not None else "legacy",
                "pending": self.pending, "following": self.following,
                "follow_enabled": self.follow_enabled,
            }))
        if self.following or self.pending:
            self.route_diagnostic("route_deferred_while_planning_or_following",
                                  stamp=stamp, age=round(age, 3))
            return  # Finish the current short segment before selecting another.
        self.sync_recovery_progress(message)
        if self.recovery_attempts >= 3:
            if self.fail_confirmed_approach('route_recovery_exhausted',
                                            retry_count=self.recovery_attempts):
                return
            self.reject_exploration_route()
            self.status('route_recovery_exhausted', retry_count=self.recovery_attempts)
            return
        if self.blocked_endpoint is not None and (
                self.exploration_metadata(message) is not None
                or distance(endpoint, self.blocked_endpoint) < 4.0):
            # sync_recovery_progress already released blocks for new owners.
            # A partial graph endpoint moving does not change an exploration ID.
            points = [xy(p.pose) for p in message.poses]
            changed_route = (self.blocked_route is not None and
                             any(distance_to_route(p, self.blocked_route) > 3.0
                                 for p in points))
            changed_map = (time.monotonic() >= self.blocked_until and
                           hash(tuple(self.grid.data)) != self.blocked_grid_hash)
            if not changed_route and not changed_map:
                return
            # A different polyline is another attempt at the same destination,
            # not proof of progress. Only transport recovery is uncharged, as
            # in the timer admission path; a new owner was reset above.
            if self.last_status.get('status') != 'execution_unavailable':
                self.recovery_attempts += 1
            self.blocked_endpoint = None
            self.last_plan_start = None
            self.last_plan_endpoint = None
        start = xy(self.odom.pose.pose)
        if (self.last_status.get('status') not in ('route_done', 'segment_done')
                and self.last_plan_start is not None and self.last_plan_endpoint is not None
                and distance(start, self.last_plan_start) < 2.0
                and distance(endpoint, self.last_plan_endpoint) < 4.0):
            return
        self.route = message
        self.route_complete = incoming_complete
        self.route_endpoint = endpoint
        if self.execute_frontier_choice_path(selected_choice, metadata):
            return
        self.start_segment()

    def sync_recovery_progress(self, route=None):
        """Bind retry credit to the destination, never a changing graph prefix."""
        route = self.route if route is None else route
        if (self.odom is None or route is None or not route.poses
                or self.input_freshness_issue() is not None):
            return None
        metadata = self.exploration_metadata(route)
        if metadata is not None:
            identity = ('exploration', metadata['exploration_id'])
            goal = tuple(metadata['goal'])
        else:
            identity = ('memory' if self.memory_goal is not None else
                        'object' if self.expected is not None else 'legacy',)
            goal = self.expected or self.memory_goal or xy(route.poses[-1].pose)
        scope = (self.query_generation, route.header.frame_id, identity)
        previous = self.recovery_progress
        if (previous is None or previous['scope'] != scope
                or (metadata is None and distance(goal, previous['goal']) >= 4.)):
            self.recovery_progress = dict(scope=scope, goal=tuple(goal),
                credited_distance=distance(xy(self.odom.pose.pose), goal))
            if previous is not None:
                self.recovery_attempts = 0
                self.blocked_endpoint = self.blocked_route = self.blocked_grid_hash = None
                self.blocked_until = 0.
                self.last_plan_start = self.last_plan_endpoint = None
        return self.recovery_progress

    def credit_recovery_progress(self):
        # Only an accepted, independently checked action result calls this.
        # Retreading a locally successful metre cannot credit it a second time.
        if self.input_freshness_issue() is not None:
            return
        progress = self.sync_recovery_progress()
        if progress is None:
            return
        remaining = distance(xy(self.odom.pose.pose), progress['goal'])
        if remaining <= progress['credited_distance'] - 1.:
            progress['credited_distance'] = remaining
            self.recovery_attempts = 0

    def exploration_metadata(self, route):
        if (route is None or self.expected is not None or self.memory_goal is not None
                or self.query_reset_pending):
            return None
        metadata = self.route_metadata.get(stamp_ns(route), {})
        identity = metadata.get('exploration_id')
        if not isinstance(identity, str) or not identity or len(identity) > 128:
            return None
        return metadata

    def exploration_destination(self):
        metadata = self.exploration_metadata(self.route)
        if metadata and metadata.get('complete'):
            return tuple(metadata['goal'])
        return None

    def acknowledge_exploration_observation(self):
        metadata = self.exploration_metadata(self.route)
        destination = self.exploration_destination()
        speed = (math.hypot(self.odom.twist.twist.linear.x,
                            self.odom.twist.twist.linear.y) if self.odom is not None else math.nan)
        if (not self.follow_enabled or not metadata or destination is None
                or self.following or self.pending or self.input_freshness_issue() is not None
                or distance(xy(self.odom.pose.pose), destination) >= 1.
                or not math.isfinite(speed) or speed > .15):
            return False
        heading = self.exploration_observation_heading()
        if heading is not None and abs(math.atan2(math.sin(yaw(self.odom.pose.pose)-heading),
                math.cos(yaw(self.odom.pose.pose)-heading))) > .35:
            return False
        route = self.route
        newest = self.exploration_metadata(self.latest_route)
        if (newest and newest['exploration_id'] == metadata['exploration_id']
                and stamp_ns(self.latest_route) >= stamp_ns(route)):
            route = self.latest_route
        age = (self.get_clock().now().nanoseconds - stamp_ns(route)) * 1e-9
        if (stamp_ns(route) < self.query_started_ns or not 0. <= age <= 5.
                or not route.poses):
            return False
        message = PoseStamped()
        message.header = deepcopy(route.header)
        message.pose = deepcopy(route.poses[-1].pose)
        self.route_observation_pub.publish(message)
        return True

    def current_route_points(self):
        if self.recovering and self.escape_path is not None:
            return [xy(p.pose) for p in self.escape_path.poses]
        if self.route is None:
            # A late ComputePath result can arrive after the route was cleared
            # (205615: AttributeError on None.poses killed the bridge). An
            # empty route is the honest input; callers already reject it.
            return []
        points = [xy(p.pose) for p in self.route.poses]
        # GraphNav's 6 m goal radius can end before the remembered viewpoint.
        # Finish that approach with the same collision-checked local planner.
        if (getattr(self, 'route_complete', True) and self.expected is not None and distance(points[-1], self.expected) <= 7.0
                and distance(points[-1], self.expected) > 1e-6):
            points.append(self.expected)
        destination = self.exploration_destination()
        if destination is not None and distance(points[-1], destination) > 1e-6:
            points.append(destination)
        return remaining_route(xy(self.odom.pose.pose), points)

    def exploration_observation_heading(self):
        if self.confirmed_route_goal is not None and self.confirmed_route_object is not None:
            if self.confirmed_route_heading is not None:
                return self.confirmed_route_heading
            target = self.live_observation_target or self.confirmed_route_object
            endpoint = self.confirmed_route_goal
            return math.atan2(target[1]-endpoint[1], target[0]-endpoint[0])
        if self.expected is not None or self.memory_goal is not None or self.route is None:
            return None
        metadata = self.route_metadata.get(stamp_ns(self.route), {})
        value = metadata.get('observation_heading')
        if metadata.get('complete') and isinstance(value, (int, float)) and math.isfinite(value):
            return float(value)
        return None

    def extension_candidates(self, candidates, old_target):
        """Avoid redundant extensions of the same finite controller action."""
        return [c for c in candidates if distance(c[:2], old_target) >= 4.]

    def start_segment(self, continuous=False):
        self.consume_pending_odometry()
        if self.follow_release is not None:
            return
        if self.visible_approach_active and self.visible_goal_refresh_pending:
            return  # A completed old leg cannot plan against the old intention.
        if (self.follow_quarantine or self.frontier_choice is not None
                or self.confirmed_approach_failure is not None):
            return
        if (self.recovering or self.query_reset_pending or self.odom is None or self.grid is None
                or self.route is None or self.pending or self.replacement_pending
                or (self.following and not continuous)):
            if continuous:
                self.extension_failed('extension_context_changed')
            return
        metadata = self.exploration_metadata(self.route)
        if metadata is not None and metadata['exploration_id'] in self.retired_explorations:
            self.status('awaiting_graph_route', retired_exploration_id=metadata['exploration_id'])
            return
        stale = self.input_freshness_issue()
        if stale:
            if continuous:
                self.extending_path = False
            self.report_stale_input(stale)
            return
        transport_issue = self.execution_transport_issue()
        if transport_issue is not None:
            self.execution_transport_unavailable(transport_issue)
            return
        start = xy(self.odom.pose.pose)
        endpoint = (self.expected if getattr(self, 'route_complete', True) else None) or self.exploration_destination() or xy(self.route.poses[-1].pose)
        self.sync_recovery_progress()
        self.last_plan_start = start
        self.last_plan_endpoint = endpoint
        observation_heading = self.exploration_observation_heading()
        observation_aligned = (observation_heading is None or abs(math.atan2(
            math.sin(yaw(self.odom.pose.pose)-observation_heading),
            math.cos(yaw(self.odom.pose.pose)-observation_heading))) <= .35)
        endpoint_tolerance = (OBJECT_CONFIRMATION_STANDOFF_TOLERANCE_M
                              if self.confirmed_route_goal is not None else 1.0)
        if distance(start, endpoint) < endpoint_tolerance and not self.memory_selecting and observation_aligned:
            if continuous:
                self.extension_failed('extension_endpoint_reached')
                return
            self.status("route_done", goal=list(endpoint),
                        requires_visual_confirmation=self.expected is not None)
            self.acknowledge_exploration_observation()
            self.await_object_observation()
            return
        blocked = self.pose_blockers(self.grid, start, yaw(self.odom.pose.pose))
        if blocked:
            if continuous:
                self.extension_failed('start_footprint_blocked')
                return
            self.block_current_route()
            self.status("start_footprint_blocked", goal=list(endpoint),
                        blocked_samples=blocked, navigation_epoch=self.epoch)
            return
        points = self.current_route_points()
        metadata = self.exploration_metadata(self.route)
        self.sync_failed_goal_scope(metadata.get('exploration_id') if metadata else None)
        if (self.lookback_active
                and time.monotonic()-self.lookback_started_at > 15.):
            self.release_visible_approach('side_lookback_timeout')
            return
        candidates = route_candidates(
            start, points, self.grid, maximum=self.max_segment,
            limit=self.max_candidates)
        # Memory selection deliberately probes remembered views, including
        # blocked ones, so its own rejection/switch logic must still see them.
        filter_route = (not self.memory_selecting and self.memory_goal is None
                        and self.grid is not None and self.route is not None
                        and self.grid.header.frame_id == self.route.header.frame_id)
        route_skipped = 0
        cached_skipped = 0
        route_infeasible = []
        self.candidates = []
        if observation_heading is not None and distance(start, endpoint) <= self.max_segment:
            self.candidates.append((*endpoint, observation_heading, distance(start, endpoint)))
        if self.confirmed_route_goal is not None and distance(start, endpoint) <= self.max_segment:
            heading = self.exploration_observation_heading()
            self.candidates.append((*endpoint, heading, distance(start, endpoint)))
        for candidate in candidates:
            if self.memory_goal is not None and distance(candidate[:2], self.expected) < 1e-6:
                self.candidates.extend(self.memory_terminal_candidates(start, candidate[:2]))
                continue
            x, y, heading, arc = candidate
            if filter_route and self.goal_recently_failed(start, (x, y), heading):
                cached_skipped += 1
            elif filter_route and not self.vehicle_pose_feasible((x, y), heading):
                route_skipped += 1
                route_infeasible.append(candidate)
            else:
                self.candidates.append(candidate)
            # A coarse graph corner is a position constraint, not a required
            # vehicle orientation. Its outgoing edge can force a Dubins loop
            # at a short segment endpoint. Also try the approach bearing;
            # Memory endpoints evaluate observation and arrival headings separately.
            bearing = math.atan2(y - start[1], x - start[0])
            difference = math.atan2(math.sin(bearing - heading), math.cos(bearing - heading))
            if abs(difference) > 0.1:
                alternative = (x, y, bearing, arc)
                if filter_route and self.goal_recently_failed(start, (x, y), bearing):
                    cached_skipped += 1
                elif filter_route and not self.vehicle_pose_feasible((x, y), bearing):
                    route_skipped += 1
                    route_infeasible.append(alternative)
                else:
                    self.candidates.append(alternative)
        if cached_skipped:
            self.get_logger().info(
                "Skipped %d cached failed route candidates" % cached_skipped)
        if route_skipped:
            self.get_logger().info(
                "Skipped %d route candidates with no clear goal pose" % route_skipped)
        if not self.candidates and route_infeasible:
            # Never strand the vehicle at a start/corner where every sample
            # failed the endpoint pose pre-filter. Retry those candidates; the
            # planner's collision/curvature checks still decide admission.
            # Cached planner failures stay excluded: re-planning the exact
            # (owner, start region, goal) triples that already returned no path
            # only repeats the search this cache exists to avoid.
            self.candidates = list(route_infeasible)
            self.status('route_filter_fallback', rejected=len(route_infeasible))
        if self.memory_selecting:
            self.memory_full_check = getattr(self, 'route_complete', True) and distance(start, endpoint) <= self.max_segment
            if self.memory_full_check:
                # A short prefix cannot prove that the remembered viewpoint is reachable.
                self.candidates = self.memory_terminal_candidates(start, endpoint)
            self.memory_deadline = time.monotonic() + 25.0
        if continuous and self.motion_queue:
            old_target = xy(self.motion_queue[0].poses[-1].pose)
            self.candidates = self.extension_candidates(self.candidates, old_target)
        self.begin_exploration_comparison()
        self.candidate_index = 0
        self.failure_reasons = []
        self.normal_best = None
        if not self.candidates:
            # A visible target is only an observation step, not a GraphNav
            # route commitment. If the graph has no free sample at that point,
            # still try one bounded local ComputePath to the accepted point;
            # the normal collision/curvature/freshness checks remain active.
            if (self.visible_approach_active and self.expected is not None
                    and distance(self.expected, endpoint) <= 2.0):
                heading = math.atan2(self.expected[1]-start[1],
                                     self.expected[0]-start[0])
                self.candidates = [(*self.expected, heading,
                                    distance(start, self.expected))]
                if not self.plan_client.wait_for_server(timeout_sec=0.1):
                    self.status("planner_unavailable")
                    return
                self.status('visible_target_local_fallback',
                            goal=list(self.expected),
                            requires_visual_confirmation=True)
                self.try_candidate()
                return
            if self.extending_path:
                self.extension_failed('no_extension_candidate')
                return
            if self.memory_candidates_exhausted():
                return
            if self.fail_confirmed_approach('no_free_graph_candidate'):
                return
            self.block_current_route()
            self.reject_exploration_route()
            self.status("no_free_graph_candidate", goal=list(endpoint))
            return
        if not self.plan_client.wait_for_server(timeout_sec=0.1):
            if self.extending_path:
                self.extension_failed('planner_unavailable')
                return
            self.status("planner_unavailable")
            return
        self.try_candidate()

    def try_candidate(self):
        if self.frontier_comparison is not None and not self.frontier_comparison_current():
            self.finish_frontier_comparison(aborted=True)
            return
        if self.candidate_index >= len(self.candidates):
            if self.extending_path:
                self.extension_failed(list(self.failure_reasons))
                return
            if self.memory_candidates_exhausted():
                return
            if self.try_planning_escape():
                return
            # Transport/protocol failures do not prove a viewpoint unusable.
            # Actual empty/invalid vehicle paths may hand it to observation
            # recovery; each alternative still traverses GraphNav and Smac.
            if (self.failure_reasons and not any(
                    r.startswith('planner_') or r == 'path_frame_mismatch'
                    for r in self.failure_reasons)
                    and self.fail_confirmed_approach('no_vehicle_path',
                                                    reasons=list(self.failure_reasons))):
                return
            if (self.visible_approach_active and self.failure_reasons and not any(
                    r.startswith('planner_') or r=='path_frame_mismatch'
                    for r in self.failure_reasons)):
                self.release_visible_approach('visible_target_no_vehicle_path')
                return
            self.block_current_route()
            self.reject_exploration_route()
            self.status("no_vehicle_path",
                        goal=(list(self.route_endpoint)
                              if self.route_endpoint is not None else None),
                        candidates=len(self.candidates), reasons=self.failure_reasons,
                        candidate_goals=[list(c[:3]) for c in self.candidates],
                        graph_route=[list(p) for p in self.current_route_points()])
            return
        # Smac holds the costmap mutex while searching. Back-to-back failed
        # candidates can starve its publication thread, so yield until a new
        # costmap message arrives after one normal publication interval. Use
        # local receipt time here: grid_received_at tracks header age for the
        # independent input freshness check and may already be ~1 s old when a
        # new costmap message is delivered.
        if self.grid_arrived_at is None or time.monotonic() - self.grid_arrived_at > .75:
            self.pending = True
            self.candidate_waiting_for_map = True
            self.status('waiting_for_costmap_update', candidate_index=self.candidate_index)
            return
        self.candidate_waiting_for_map = False
        x, y, heading, _ = self.candidates[self.candidate_index]
        pose = PoseStamped()
        pose.header.frame_id = self.grid.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(heading / 2)
        pose.pose.orientation.w = math.cos(heading / 2)
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.use_start = False
        if self.frontier_comparison is not None:
            goal.use_start = True
            goal.start.header = deepcopy(self.frontier_comparison['header'])
            goal.start.header.stamp = self.get_clock().now().to_msg()
            goal.start.pose = deepcopy(self.frontier_comparison['origin'])
        goal.planner_id = self.planner_id
        epoch = self.epoch
        index = self.candidate_index
        self.plan_request_serial += 1
        serial = self.active_plan_serial = self.plan_request_serial
        self.pending = True
        self.plan_request_started_at = time.monotonic()
        planned_start = goal.start.pose if goal.use_start else self.odom.pose.pose
        self.active_plan_start = xy(planned_start)
        self.active_plan_failure_context = self.failed_goal_context(yaw(planned_start))
        try:
            self.get_logger().info("ComputePath request sent: epoch=%d candidate=%d request=%d goal=(%.2f, %.2f)" %
                                   (epoch, index, serial, x, y))
            future = self.plan_client.send_goal_async(goal)
        except Exception as exc:
            self.candidate_failed("planner_send_error:" + str(exc)[:80])
            return
        # Map-wait admission has finished. Publish before registering the
        # response callback: an already-completed future must leave its real
        # result state visible, rather than being overwritten by "planning".
        self.status('planning', candidate_index=index, request_serial=serial,
                    background=bool(self.following),
                    requested_goal_pose=[float(x),float(y),float(heading)],
                    requested_start_pose=([*xy(goal.start.pose),yaw(goal.start.pose)]
                                          if goal.use_start else None),
                    bridge_odom_pose=[*xy(self.odom.pose.pose),yaw(self.odom.pose.pose)],
                    planner_use_start=bool(goal.use_start))
        self.add_navigation_done_callback(future,
            lambda f: self.on_plan_accepted(f, epoch, index, serial))

    def reserve_short_route_probe(self):
        """Use a remaining failure slot for a geometrically different prefix.

        Normal candidate order stays intact. Only an already failing, stopped
        exploration may probe a short prefix before retiring its route. A
        reversible car can reach a point behind it without turning its body
        to face that point; required observation headings remain directional.
        """
        if (self.carrier_model != 'ackermann' or not self.follow_enabled
                or self.following or self.extending_path or self.recovering
                or self.expected is not None or self.memory_goal is not None
                or self.memory_selecting or self.memory_active is not None
                or self.query_reset_pending or self.follow_quarantine
                or self.follow_release is not None
                or self.input_freshness_issue() is not None):
            return False
        start = xy(self.odom.pose.pose)
        current_heading = yaw(self.odom.pose.pose)
        observation_heading = self.exploration_observation_heading()
        observation_endpoint = self.exploration_destination() or self.route_endpoint
        probes = []
        for x, y, heading, arc in self.candidates:
            if (observation_heading is not None and observation_endpoint is not None
                    and distance((x, y), observation_endpoint) < 1e-6):
                continue
            behind = ((x-start[0])*math.cos(current_heading)
                      + (y-start[1])*math.sin(current_heading) < 0.)
            if behind and math.cos(heading-current_heading) < 0.:
                heading = math.remainder(heading+math.pi, 2*math.pi)
            if any(distance((x, y), previous[:2]) < 1e-6
                   and abs(math.remainder(heading-previous[2], 2*math.pi)) < 1e-6
                   for previous in self.candidates[:self.candidate_index+1]):
                continue
            if (self.goal_recently_failed(start, (x, y), heading)
                    or not self.vehicle_pose_feasible((x, y), heading)):
                continue
            probes.append((x, y, heading, arc))
        if not probes:
            return False
        probe = min(probes, key=lambda p: (p[3], abs(math.remainder(p[2]-current_heading, 2*math.pi))))
        self.candidates.insert(self.candidate_index+1, probe)
        self.status('short_route_probe', candidate_goal=list(probe[:3]),
                    failed_attempts=self.empty_path_streak)
        return True

    def candidate_failed(self, reason):
        self.get_logger().warning("ComputePath candidate failed: epoch=%d candidate=%d reason=%s" %
                                  (self.epoch, self.candidate_index, reason))
        if (reason == 'empty_path' and self.active_plan_start is not None
                and self.candidate_index < len(self.candidates)):
            goal = self.candidates[self.candidate_index]
            self.remember_failed_goal(self.active_plan_start, goal[:2], goal[2],
                                      context=self.active_plan_failure_context)
        if reason == 'empty_path':
            # Scope the streak to the admitted route as well as the epoch: a
            # stale streak must not retire a fresh owner's first failure.
            route_key = ((self.route.header.frame_id, stamp_ns(self.route))
                         if self.route is not None else None)
            if (self.empty_path_streak_epoch != self.epoch
                    or self.empty_path_streak_route != route_key):
                self.empty_path_streak = 0
                self.empty_path_streak_epoch = self.epoch
                self.empty_path_streak_route = route_key
            self.empty_path_streak += 1
        else:
            self.empty_path_streak = 0
        self.pending = False
        self.plan_handle = None
        self.plan_request_started_at = None
        self.active_plan_serial = None
        if self.frontier_comparison is not None:
            self.advance_frontier_comparison(reason)
            return
        self.failure_reasons.append(reason)
        if (reason == 'empty_path'
                and self.empty_path_streak >= self.max_consecutive_empty_paths
                and self.follow_enabled and self.frontier_comparison is None
                and self.expected is None and self.memory_goal is None
                and self.memory_active is None and not self.memory_selecting):
            # 183339 epoch 4: 19 consecutive empty_path candidates burned
            # ~33 s while the graph route was blocked. A few independent
            # failures are enough to retire this route instead of retrying
            # every sample; no collision or freshness check is relaxed.
            self.empty_path_streak = 0
            self.motion_queue = []
            self.pending = False
            self.block_current_route()
            self.reject_exploration_route()
            self.status("no_vehicle_path",
                        goal=(list(self.route_endpoint)
                              if self.route_endpoint is not None else None),
                        candidates=len(self.candidates),
                        reasons=list(self.failure_reasons),
                        candidate_goals=[list(c[:3]) for c in self.candidates])
            return
        if self.normal_best is not None:
            path, target, _ = self.normal_best
            self.normal_best = None
            self.execute_checked_path(path, target)
            return
        if (reason == 'empty_path'
                and 0 < self.empty_path_streak < self.max_consecutive_empty_paths
                and (self.empty_path_streak == self.max_consecutive_empty_paths-1
                     or self.candidate_index+1 >= len(self.candidates))):
            self.reserve_short_route_probe()
        self.candidate_index += 1
        self.try_candidate()

    def current_plan_request(self, epoch, index, serial):
        # Legacy direct callback fixtures omit serial; dispatched requests
        # always carry it, including retries at the same candidate index.
        return (epoch == self.epoch and index == self.candidate_index
                and (serial is None or serial == self.active_plan_serial))

    def on_plan_accepted(self, future, epoch, index, serial=None):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error("ComputePath goal response failed: epoch=%d candidate=%d error=%s" %
                                    (epoch, index, str(exc)[:100]))
            if self.current_plan_request(epoch, index, serial):
                self.candidate_failed("planner_error:" + str(exc)[:80])
            return
        if not self.current_plan_request(epoch, index, serial):
            self.get_logger().warning("Late ComputePath response: sent_epoch=%d current_epoch=%d candidate=%d request=%s active=%s accepted=%s" %
                                      (epoch, self.epoch, index, serial, self.active_plan_serial,
                                       bool(handle is not None and handle.accepted)))
            if handle is not None and handle.accepted:
                self.cancel_goal(handle, "stale_compute_path")
            return
        if handle is not None and handle is self.plan_handle:
            return  # Repeated acceptance must not register another result callback.
        if handle is None or not handle.accepted:
            self.get_logger().warning("ComputePath goal rejected: epoch=%d candidate=%d" % (epoch, index))
            self.candidate_failed("planner_rejected")
            return
        self.get_logger().info("ComputePath goal accepted: epoch=%d candidate=%d" % (epoch, index))
        self.plan_handle = handle
        self.add_navigation_done_callback(handle.get_result_async(),
            lambda f: self.on_plan_result(f, epoch, index, serial))

    def on_plan_result(self, future, epoch, index, serial=None):
        if not self.current_plan_request(epoch, index, serial):
            self.get_logger().warning("Late ComputePath result ignored: sent=(%d,%d,%s) current=(%d,%d,%s)" %
                                      (epoch, index, serial, self.epoch, self.candidate_index, self.active_plan_serial))
            return
        self.consume_pending_odometry()
        self.active_plan_serial = None
        self.plan_handle = None
        self.plan_request_started_at = None
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
            self.get_logger().error("ComputePath result failed: epoch=%d candidate=%d error=%s" %
                                    (epoch, index, str(exc)[:100]))
            self.candidate_failed("planner_error:" + str(exc)[:80])
            return
        self.get_logger().info("ComputePath result: epoch=%d candidate=%d status=%d poses=%d" %
                               (epoch, index, result.status, len(path.poses)))
        if result.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.candidate_failed("empty_path")
            return
        if path.header.frame_id != self.grid.header.frame_id:
            self.candidate_failed("path_frame_mismatch")
            return
        if self.frontier_comparison is not None:
            self.record_frontier_cost(path)
            return
        if self.extending_path:
            path = trim_moving_path(path, xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                allow_heading_change=self.carrier_model == "ideal_point")
            if path is None:
                self.candidate_failed('extension_start_moved')
                return
        if self.route is None:
            # The request's route was cleared while the planner was running.
            # Its path has no graph owner to validate against; discard it
            # instead of treating the response as current progress.
            self.candidate_failed('route_cleared')
            return
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        target = self.candidates[index][:2]
        route = self.current_route_points()
        if self.visible_approach_active and self.expected is not None:
            # The local observation fallback is validated against its own
            # bounded segment; the graph may not contain the target point.
            route = [xy(self.odom.pose.pose), tuple(self.expected)]
        reason = self.validate_path(points, headings,
            [xy(self.odom.pose.pose)] + route, self.grid, target,
            start=xy(self.odom.pose.pose),
            target_heading=(self.candidates[index][2]
                if self.memory_goal is not None
                and distance(target, self.expected) < 1e-6 else None))
        if (self.carrier_model == "ackermann" and reason is None
                and initial_maneuver_already_reached(
                points, headings, xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                position_tolerance=ACKERMANN_CUSP_POSITION_TOLERANCE_M)):
            reason = 'maneuver_already_within_tolerance'
        reason=reason or self.exploration_motion_issue(points)
        reason=reason or self.initial_tracking_issue(points,headings)
        if reason:
            self.candidate_failed(reason)
            return
        validated_path = path
        self.full_path_pub.publish(path)
        # A valid path proves at least one route sample is reachable; a later
        # failure starts a new streak rather than retiring the route.
        self.empty_path_streak = 0
        if self.memory_selecting:
            self.record_memory_path(path, points, headings)
            return
        observation_heading = self.exploration_observation_heading()
        observation_endpoint = self.exploration_destination() or self.route_endpoint
        observation_candidate = (observation_heading is not None and observation_endpoint is not None
            and distance(target, observation_endpoint) < 1e-6
            and abs(math.atan2(math.sin(self.candidates[index][2]-observation_heading),
                              math.cos(self.candidates[index][2]-observation_heading))) < 1e-6)
        if self.memory_goal is None and not observation_candidate:
            path_length = sum(distance(a, b) for a, b in zip(points, points[1:]))
            motion_cost = (maneuver_cost(points) if self.carrier_model == "ackermann"
                           else path_length)
            retreat_cost = (0.25 * first_section_retreat_cost(points, target)
                            if self.carrier_model == 'ackermann' else 0.0)
            cost = (motion_cost + retreat_cost
                    + .1 * sum(distance_to_route(p, route) for p in points) / len(points))
            if self.normal_best is not None:
                if self.normal_best[2] < cost:
                    path, target, cost = self.normal_best
                self.normal_best = None
            elif (self.should_compare_endpoint_headings(path)
                  and index+1 < len(self.candidates)
                  and distance(self.candidates[index+1][:2], target) < 1e-6):
                # Compare only the two headings at this endpoint. Do not run
                # every shorter candidate once a feasible endpoint exists.
                self.normal_best = (path, target, cost)
                self.candidate_index += 1
                self.try_candidate()
                return
            if self.compare_curvature_heading(path, target, cost):
                return
        if path is validated_path and route == self.current_route_points():
            # This synchronous result callback owns the navigation state group.
            # Its map/pose/route cannot change between the successful checks
            # above and this handoff. Do not repeat the whole-path work while
            # odometry callbacks wait; execution still checks its actual segment.
            self.dispatch_validated_path(path, target, points, headings)
        else:
            # An earlier heading winner or a visible-target fallback checked
            # against a different corridor still needs the execution checks.
            self.execute_checked_path(path, target)

    def prepare_intent_change(self):
        return False

    def retain_motion_for_retired_intent(self, data, metadata):
        return False

    def release_continuous_visible_intent(self, reason):
        return False

    def should_compare_endpoint_headings(self, path):
        return True

    def compare_curvature_heading(self, path, target, cost):
        """Probe one arc-compatible yaw only after the normal choice needs shifts.

        Keep that validated choice as fallback. Explicit observation/recall poses
        and moving replacement actions keep their existing orientation contract.
        """
        if (self.carrier_model != 'ackermann' or not self.follow_enabled
                or self.extending_path or self.memory_goal is not None
                or self.memory_selecting or self.confirmed_route_goal is not None
                or self.following or self.follow_active is not None):
            return False
        points = [xy(p.pose) for p in path.poses]
        if len(points) < 2 or len(motion_sections(points)) <= 1:
            return False
        heading = forward_arc_arrival_heading(points[0], yaw(path.poses[0].pose),
                                              target, self.minimum_turning_radius)
        if (heading is None or any(distance(c[:2], target) < 1e-6
                and abs(math.remainder(c[2]-heading, 2*math.pi)) <= .1
                for c in self.candidates)
                or self.goal_recently_failed(points[0], target, heading)
                or not self.vehicle_pose_feasible(target, heading)):
            return False
        candidate = (*target, heading, self.candidates[self.candidate_index][3])
        self.candidates.insert(self.candidate_index+1, candidate)
        self.normal_best = (path, target, cost)
        self.candidate_index += 1
        self.status('curvature_heading_probe', goal=list(target),
                    requested_heading=heading, retained_sections=len(motion_sections(points)))
        self.try_candidate()
        return True

    def exploration_motion_issue(self, points):
        # Candidate comparison and execution must admit the same paths. A
        # route rejected at dispatch must not displace an executable frontier
        # or viewpoint during selection. Semantic/recall destinations do not
        # change the physical cost of stopping at every direction reversal.
        if (self.follow_enabled and self.carrier_model != "ideal_point"
                and not self.recovering
                and len(motion_sections(points)) > self.max_motion_sections):
            return "excessive_motion_sections"
        return None

    def execute_checked_path(self, path, target):
        self.consume_pending_odometry()
        # A map can change while the alternative heading is being planned.
        self.pending = False
        stale = self.input_freshness_issue()
        if stale:
            self.normal_best = None
            self.report_stale_input(stale)
            return
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        reason = self.validate_path(points, headings,
            [xy(self.odom.pose.pose)]+self.current_route_points(), self.grid, target,
            start=xy(self.odom.pose.pose))
        reason=reason or self.initial_tracking_issue(points,headings)
        if reason:
            if self.extending_path:
                self.extension_failed(reason)
                return
            # The candidate path failed current-map validation before execution.
            # Request a replacement exploration route immediately; keep only a
            # short bounded retry for explicit or remembered destinations.
            retry_delay = 2.0
            self.block_current_route(backoff=retry_delay)
            route_rejected = self.reject_exploration_route()
            self.status("path_changed_before_execution", reason=reason,
                        retry_after_sec=retry_delay,
                        route_rejected=route_rejected)
            return
        self.dispatch_validated_path(path, target, points, headings)

    def dispatch_validated_path(self, path, target, points, headings):
        """Synchronous handoff from whole-path checks to segment admission."""
        self.consume_pending_odometry()
        self.pending = False
        stale = self.input_freshness_issue()
        if stale:
            self.normal_best = None
            self.report_stale_input(stale)
            return
        if not self.follow_enabled:
            self.path_pub.publish(path)
            self.status("planned", goal=list(target), poses=len(path.poses),
                        planner_id=self.planner_id,
                        graph_goal=(list(self.route_endpoint)
                                    if self.route_endpoint is not None else None))
            return
        # Keep full-path validation and memory scoring; split execution alone.
        # RPP prunes nearest points before detecting its own reversing cusps.
        sections = self.execution_paths(path, points)
        motion_issue = self.exploration_motion_issue(points)
        if motion_issue:
            if self.extending_path:
                # A bad future route does not invalidate accepted motion.
                self.extension_failed(motion_issue)
                return
            self.motion_queue = []
            self.candidate_failed(motion_issue)
            return
        if self.extending_path:
            self.extending_path = False
            # Completion may precede the background planner result. Only an
            # action that is still running can consume a deferred extension or
            # be replaced. Otherwise dispatch through the normal actual-pose,
            # local-map and actuator checks below.
            if self.following:
                first_direction = (points[1][0]-points[0][0])*math.cos(headings[0]) + (points[1][1]-points[0][1])*math.sin(headings[0])
                if len(sections) != 1 or first_direction <= 0:
                    self.prepared_extension = (deepcopy(path), target)
                    return
                self.send_motion_segment(replacement=sections)
                return
        self.motion_queue = sections
        self.send_motion_segment()

    def execution_paths(self, path, points):
        if self.carrier_model == 'ideal_point':
            return [deepcopy(path)]
        sections=[]
        for first,last in motion_sections(points):
            section=deepcopy(path);section.poses=section.poses[first:last+1]
            sections.append(section)
        return sections

    def motion_join_geometry(self, points, headings, position_tolerance):
        return points,headings,position_tolerance

    def endpoint_may_complete_immediately(self, points):
        return True

    def send_motion_segment(self, replacement=None, after_maneuver=False):
        self.consume_pending_odometry()
        if (self.follow_quarantine or self.follow_release is not None or self.follow_request is not None
                or (replacement is None and self.follow_active is not None)):
            return False
        queue = replacement if replacement is not None else self.motion_queue
        if not queue:
            return False
        transport_issue = self.execution_transport_issue()
        if transport_issue is not None:
            self.execution_transport_unavailable(transport_issue)
            return False
        stale = self.input_freshness_issue() or self.local_freshness_issue()
        if stale:
            self.motion_queue = []
            self.last_plan_start = self.last_plan_endpoint = None
            self.report_stale_input(stale)
            return False
        path = queue[0]
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        target = points[-1]
        # Select once, before actual-pose feasibility checks. The short
        # endpoint prediction and the dispatched action share this contract.
        goal_checker_id,xy_tolerance,yaw_tolerance=self.motion_completion_contract(points,headings,len(queue))
        reason = ('start_footprint_blocked' if self.pose_blockers(
            self.grid, xy(self.odom.pose.pose), yaw(self.odom.pose.pose)) else None)
        reason = reason or self.validate_path(points, headings,
            [xy(self.odom.pose.pose)] + self.current_route_points(), self.grid, target,
            start=xy(self.odom.pose.pose))
        reason = reason or self.validate_local_path(points, headings, self.local_grid,
            xy(self.odom.pose.pose), yaw(self.odom.pose.pose))
        reason=reason or self.tracking_issue(points,headings,len(queue))
        if (self.carrier_model == "ackermann"
                and reason != 'start_footprint_blocked'):
            offset = distance(xy(self.odom.pose.pose), points[0])
            angle = abs(math.atan2(math.sin(yaw(self.odom.pose.pose)-headings[0]),
                                   math.cos(yaw(self.odom.pose.pose)-headings[0])))
            join_points,join_headings,join_tolerance=self.motion_join_geometry(points,headings,xy_tolerance)
            join, join_failure_reason = diagnose_maneuver_join(
                join_points, join_headings, xy(self.odom.pose.pose),
                yaw(self.odom.pose.pose), self.minimum_turning_radius,
                endpoint_position_tolerance=join_tolerance)
            if join is not None and reason is None:
                join_points = [p[:2] for p in join]
                join_headings = [p[2] for p in join]
                for grid in (self.grid, self.local_grid):
                    reason = validate_local_prefix(join_points, join_headings, grid,
                        xy(self.odom.pose.pose), yaw(self.odom.pose.pose), maximum=4.)
                    if reason:
                        break
            if reason or join is None:
                if not after_maneuver:
                    # Initial and replacement paths have the same actual-pose
                    # contract as a cusp, but retain their own failure owner.
                    if replacement is not None:
                        self.extension_failed(reason or 'actual_pose_cannot_join')
                    else:
                        self.follow_failed('motion_section_invalid',
                            detail=reason or 'actual_pose_cannot_join',
                            join_failure_reason=join_failure_reason)
                    return
                # Replan only when the settled axle cannot join this branch
                # with feasible steering and a clear swept body on both maps.
                self.motion_queue = []
                self.last_plan_start = self.last_plan_endpoint = None
                if self.recovery_attempts>=3:
                    self.block_current_route(backoff=8.)
                    self.reject_exploration_route()
                    self.status('handoff_recovery_exhausted',reason=reason or 'actual_cusp_pose_differs',
                                retry_count=self.recovery_attempts)
                    return
                self.recovery_attempts+=1
                self.status('replanning_after_maneuver', reason=reason or 'actual_cusp_pose_differs',
                            position_error=offset, heading_error=angle,
                            path_source_stamp_ns=stamp_ns(path),
                            rejected_section_goal=list(target),
                            join_failure_reason=(join_failure_reason if join is None else None),
                            retry_count=self.recovery_attempts)
                self.start_segment()
                return
        if reason:
            if replacement is not None:
                self.extension_failed(reason)
                return
            self.follow_failed("motion_section_invalid", detail=reason)
            return
        if not self.follow_client.wait_for_server(timeout_sec=0.1):
            if replacement is not None:
                self.extension_failed('controller_unavailable')
                return
            self.motion_queue = []
            self.status("controller_unavailable")
            return False
        source_path_stamp_ns=stamp_ns(path)
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        self.status("planned", goal=list(target), poses=len(path.poses),
                    planner_id=self.planner_id, graph_goal=list(self.route_endpoint) if self.route_endpoint is not None else None,
                    motion_sections_remaining=len(queue),
                    actual_start_pose=[*xy(self.odom.pose.pose),yaw(self.odom.pose.pose)],
                    actual_start_stamp_ns=(self.odom.header.stamp.sec*10**9+self.odom.header.stamp.nanosec),
                    path_source_stamp_ns=source_path_stamp_ns,
                    goal_checker_id=goal_checker_id,goal_position_tolerance_m=xy_tolerance)
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = "FollowPath"
        follow_goal.goal_checker_id = goal_checker_id
        position_error = distance(xy(self.odom.pose.pose), target)
        heading_error = abs(math.atan2(
            math.sin(yaw(self.odom.pose.pose) - headings[-1]),
            math.cos(yaw(self.odom.pose.pose) - headings[-1])))
        if (self.endpoint_may_complete_immediately(points) and
                position_error <= xy_tolerance and heading_error <= yaw_tolerance):
            # FollowPath would report success without moving when a short cusp
            # lands inside its own goal disk. Reject this candidate so the
            # next checked route can make real progress.
            self.motion_queue = []
            self.candidate_failed('section_goal_within_controller_tolerance')
            return
        epoch = self.epoch
        self.follow_request_serial += 1
        serial = self.follow_request_serial
        self.follow_request = dict(serial=serial,started_at=time.monotonic(),
            replacement=replacement is not None,target=target,heading=headings[-1],
            xy_tolerance=xy_tolerance,yaw_tolerance=yaw_tolerance,path_stamp_ns=stamp_ns(path))
        self.replacement_pending = replacement is not None
        if replacement is None:
            self.following = True
        try:
            future = self.follow_client.send_goal_async(follow_goal,
                feedback_callback=lambda m: self.on_follow_feedback(m,epoch,serial))
        except Exception as exc:
            self.follow_transaction_timeout('controller_send_error')
            return
        self.add_navigation_done_callback(future,
            lambda f: self.on_follow_accepted(f, epoch, target, serial, queue, replacement is not None))
        return True

    def on_follow_accepted(self, future, epoch, target, serial=None, queue=None, replacement=False):
        self.consume_pending_odometry()
        if serial is not None and self.quarantine_follow_acceptance(future,serial):
            return
        if (serial is not None and epoch == self.epoch
                and serial == self.active_follow_serial and self.follow_handle is not None):
            return  # Acceptance is consumed once; do not duplicate result callbacks.
        try:
            handle = future.result()
        except Exception:
            handle = None
        if epoch != self.epoch or (serial is not None and
                (self.follow_request is None or serial != self.follow_request['serial'])):
            if handle is not None and handle.accepted:
                self.cancel_goal(handle, "stale_follow_path")
            return
        self.replacement_pending = False
        if handle is None:
            self.follow_transaction_timeout('follow_acceptance_error')
            return
        transaction,self.follow_request = self.follow_request,None
        if not handle.accepted:
            deferred = self.deferred_follow_result
            self.deferred_follow_result = None
            if replacement and deferred is not None:
                self.on_follow_result(*deferred)
                return
            if replacement and self.following:
                self.extension_failed('controller_rejected')
                return
            self.follow_failed("controller_rejected")
            return
        self.follow_active = transaction
        if transaction is not None:
            transaction.update(handle=handle,feedback_at=time.monotonic(),endpoint_since=None)
        transport_issue = self.execution_transport_issue()
        if transport_issue is not None:
            self.follow_handle = handle
            self.execution_transport_unavailable(transport_issue)
            if serial is not None:
                self.quarantine_follow_acceptance(future,serial)
            return
        self.deferred_follow_result = None
        self.active_follow_serial = self.follow_request_serial if serial is None else serial
        if queue is not None:
            self.motion_queue = queue
        self.following = True
        self.follow_handle = handle
        if replacement:
            # Acceptance is the commit point for a new motion owner. A late
            # terminal from the superseded serial cannot finish the new action.
            self.retired_motion = None
        if not self.recovering:
            self.checkpoint_execution_feedback(self.motion_queue[0])
        self.status("following", goal=list(target), graph_goal=list(self.route_endpoint) if self.route_endpoint is not None else None)
        accepted_serial = self.active_follow_serial
        try:
            self.add_navigation_done_callback(handle.get_result_async(),
                lambda f: self.on_follow_result(f, epoch, target, accepted_serial))
        except Exception:
            self.follow_transaction_timeout('follow_result_request_error')

    def on_follow_result(self, future, epoch, target, serial=None):
        self.consume_pending_odometry()
        if serial is not None and self.quarantine_follow_result(future,serial):
            return
        if epoch != self.epoch or (serial is not None and serial != self.active_follow_serial):
            return
        if self.retired_motion is not None and (
                self.retired_motion['epoch'] != epoch
                or self.retired_motion['serial'] != serial):
            self.retired_motion = None
        transport_issue = self.execution_transport_issue()
        if transport_issue is not None:
            self.execution_transport_unavailable(transport_issue)
            if serial is not None:
                self.quarantine_follow_result(future,serial)
            return
        if self.replacement_pending:
            # Nav2 may complete/cancel the old goal before acknowledging its
            # replacement. Do not invalidate the new request in that gap.
            if self.deferred_follow_result is None:
                self.deferred_follow_result = (future, epoch, target, serial)
            return
        try:
            status = future.result().status
        except Exception:
            self.follow_transaction_timeout('follow_result_error')
            return
        if status not in (4,5,6):
            self.follow_transaction_timeout('follow_result_invalid')
            return
        if self.retired_motion is not None:
            self.retired_motion['terminal'] = True
        self.active_follow_serial = None
        self.follow_active = None
        self.follow_handle = None
        self.following = False
        if self.recovering:
            reached = (status == GoalStatus.STATUS_SUCCEEDED and self.input_freshness_issue() is None
                       and self.escape_path is not None and maneuver_reached(
                           xy(self.odom.pose.pose), yaw(self.odom.pose.pose), target,
                           yaw(self.escape_path.poses[-1].pose), position_tolerance=.6))
            self.finish_execution_escape(reached, 'escape_arrived' if reached else 'escape_action_failed',
                                         action_status=status)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if len(self.motion_queue) > 1:
                stale = self.input_freshness_issue()
                if stale:
                    self.motion_queue = []
                    self.retired_motion = None
                    self.last_plan_start = self.last_plan_endpoint = None
                    self.report_stale_input(stale)
                    return
                cusp = self.motion_queue[0].poses[-1].pose
                if not maneuver_reached(xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                                        xy(cusp), yaw(cusp),
                                        position_tolerance=(ACKERMANN_CUSP_POSITION_TOLERANCE_M
                                            if self.carrier_model == 'ackermann'
                                            else MANEUVER_COMPLETION_POSITION_TOLERANCE_M),
                                        heading_tolerance=.35):
                    self.follow_failed("cusp_not_reached", goal=list(xy(cusp)))
                    return
                self.finish_execution_feedback(True, 'maneuver_completed')
                if self.retired_motion is not None:
                    self.finish_retired_exploration_section(target)
                    return
                self.credit_recovery_progress()
                self.motion_queue.pop(0)
                self.status("maneuver_done", goal=list(target),
                            motion_sections_remaining=len(self.motion_queue))
                self.send_motion_segment(after_maneuver=True)
                return
            if self.input_freshness_issue() is not None or distance(xy(self.odom.pose.pose), target) > 1.5:
                self.follow_failed('action_success_without_arrival')
                return
            self.finish_execution_feedback(True, 'segment_completed')
            if self.retired_motion is not None:
                self.finish_retired_exploration_section(target)
                return
            self.motion_queue = []
            self.blocked_endpoint = None
            self.status("segment_done", goal=list(target))
            self.credit_recovery_progress()
            if self.replacement_pending or self.pending:
                return  # The in-flight plan still owns its source graph route.
            if self.latest_route is not None:
                self.route = self.latest_route
                self.route_complete = self.latest_route_complete
                self.route_endpoint = xy(self.route.poses[-1].pose)
            if self.prepared_extension is not None:
                path, endpoint = self.prepared_extension
                self.prepared_extension = None
                path = trim_moving_path(path, xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                    allow_heading_change=self.carrier_model == "ideal_point")
                if path is not None:
                    self.execute_checked_path(path, endpoint)
                    return
            self.start_segment()
        else:
            self.follow_failed("follow_aborted", action_status=status)

    def finish_retired_exploration_section(self, target):
        # Called only after the normal action identity and actual-arrival
        # checks. A retired frontier is never an observation success/ACK.
        motion = self.retired_motion
        identity = motion['identity']
        target_handoff = motion.get('owner_kind') == 'visible_target_timeout'
        latest = self.latest_route
        if (not self.retired_replacement_context() and not self.pending
                and self.frontier_choice is None and self.frontier_comparison is None):
            # Reverse preparation sections cannot start a moving replacement.
            # on_route therefore left the new owner in latest_route. Only now,
            # after the actual old terminal/arrival, admit that fresh owner so
            # its corridor can authorize a rechecked suffix. Selection/ACK or
            # an in-flight plan still finishes through its original pipeline.
            replacement = self.replacement_route_for_retirement(identity)
            if replacement is not None:
                self.route = replacement
                self.route_complete = self.exploration_metadata(replacement)['complete']
                self.route_endpoint = xy(replacement.poses[-1].pose)
                self.sync_recovery_progress(replacement)
        if self.retired_replacement_context():
            # An expired bearing-only target cannot donate its old suffix.
            retired_tail = None if target_handoff else self.retired_tail_candidate()
            # The old action has really terminated. Preserve an in-flight plan
            # for the new intention rather than canceling and computing twice.
            self.retired_motion = None
            self.motion_queue = []
            if target_handoff:
                self.status('visible_target_handoff_segment_finished',
                            retired_approach_id=identity, goal=list(target),
                            handoff_pending=self.pending)
            else:
                self.status('exploration_segment_finished', retired_exploration_id=identity,
                            goal=list(target), handoff_pending=self.pending)
            if self.pending and self.extending_path:
                return
            if self.prepared_extension is not None:
                path, endpoint = self.prepared_extension
                self.prepared_extension = None
                path = trim_moving_path(path, xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                                       allow_heading_change=self.carrier_model == 'ideal_point')
                if path is not None:
                    self.execute_checked_path(path, endpoint)
                    return
            if (retired_tail is not None and
                    self.try_retired_tail_candidate(retired_tail, identity)):
                return
        self.reset(preserve_execution_corridor=True)
        if target_handoff:
            self.status('visible_target_handoff_segment_finished',
                        retired_approach_id=identity, goal=list(target))
        else:
            self.status('exploration_segment_finished', retired_exploration_id=identity,
                        goal=list(target))
        if latest is not None:
            self.on_route(latest)  # Freshness/frame/query/retired-ID checks all apply.

    def follow_failed(self, reason, **details):
        if self.retired_motion is not None:
            # self.route may already be a background intention. Failure of
            # the still accepted retired action must not block/reject it or
            # train it as a failed traversal. Revoke motion through the usual
            # cancellation/standstill barrier, then re-admit the latest route.
            self.stop_retired_motion(reason, **details)
            return
        self.retired_motion = None
        self.retire_follow_transactions()
        was_recovering = self.recovering
        self.deferred_follow_result = None
        self.stall_anchor = None
        self.finish_execution_feedback(False, reason)
        self.epoch += 1
        self.motion_queue = []
        self.prepared_extension = None
        self.extending_path = self.replacement_pending = False
        self.pending = False
        self.candidate_waiting_for_map = False
        self.cancel_goal(self.plan_handle, 'follow_failed_compute_path')
        self.plan_handle = None
        self.following = False
        self.follow_handle = None
        if was_recovering:
            self.finish_execution_escape(False, reason, action_status=details.get('action_status'))
            return
        if self.try_execution_escape(reason, details):
            return
        if (self.recovery_attempts >= 2
                and reason in ('follow_aborted', 'motion_section_invalid',
                               'execution_stalled', 'tracking_deviation')
                and self.fail_confirmed_approach(reason, **details)):
            return
        measured_support_loss = (reason == 'execution_stalled' and details.get('stall_class')
                                 in ('wheel_airborne_spin', 'wheel_support_lost'))
        repeated_execution_failure = (reason in ('execution_stalled', 'tracking_deviation')
                                      and self.recovery_attempts >= 2)
        repeated_local_collision = (reason == 'motion_section_invalid'
                                    and details.get('detail') == 'local_footprint_collision'
                                    and self.recovery_attempts >= 2)
        repeated_exploration_abort = (
            reason == 'follow_aborted' and details.get('action_status') == GoalStatus.STATUS_ABORTED
            and self.recovery_attempts >= 2 and self.exploration_metadata(self.route) is not None)
        repeated_visible_collision = (
            reason == 'motion_section_invalid'
            and details.get('detail') == 'local_footprint_collision'
            and self.recovery_attempts >= 1
            and self.visible_approach_active
            and self.expected is not None
            and self.memory_goal is None
            and self.memory_active is None
            and self.confirmed_route_goal is None
            and not getattr(self, 'lookback_active', False))
        if repeated_visible_collision:
            # A bearing-only goal has no confirmed object pose to justify
            # retrying the same blocked approach for the full 30 s deadline.
            # After the first map refresh/replan also hits the local body
            # footprint, release this observation point and resume exploration.
            self.release_visible_approach('visible_target_path_blocked')
            return
        if ((measured_support_loss or repeated_execution_failure)
                and self.memory_goal is not None
                and self.retry_memory_after_execution_failure()):
            return
        if (measured_support_loss or repeated_execution_failure or repeated_local_collision
                or repeated_exploration_abort):
            # Repeated inability to execute a frontier is enough to try a
            # different exploration region, even when all tires have contact.
            # Reject a failed route exit, never the object or recall destination.
            # A local-only obstacle can invalidate a globally planned section.
            # Give transient observations the existing retries, then ask the
            # graph planner for another frontier before the budget is spent.
            self.reject_exploration_route()
        # A controller failure (stalled vehicle, rejected action, or missed
        # cusp) does not prove the remembered viewpoint is unreachable.
        # Retain it and back off this route. Only failed planning checks may
        # reject a memory candidate and release its GraphNav destination.
        # Fresh inputs can become usable well before the old fixed 20 s wait.
        # Keep a bounded 2/4/8 s recovery schedule and the existing 3 retries.
        retry_delay = min(8.0, 2.0 * 2**self.recovery_attempts)
        self.block_current_route(backoff=retry_delay)
        self.status("follow_failed", reason=reason,
                    retry_after_sec=retry_delay,
                    graph_goal=list(self.route_endpoint) if self.route_endpoint else None,
                    **details)

    def reject_exploration_route(self):
        # Postpone only an exact published route exit. An explicit partial
        # route can change its exit while retaining the confirmed object goal.
        explicit_partial = (self.upstream_target_approach and self.expected is not None
            and self.memory_goal is None and self.route is not None
            and (self.route_metadata.get(stamp_ns(self.route)) or {}).get('mode') == 'frontier'
            and distance((self.route_metadata.get(stamp_ns(self.route)) or {}).get('goal',self.expected),self.expected)<.25)
        if (not self.follow_enabled or (self.expected is not None and not explicit_partial) or self.memory_goal is not None
                or self.query_reset_pending or self.input_freshness_issue() is not None
                or self.route is None or not self.route.poses
                or stamp_ns(self.route) < self.query_started_ns):
            return False
        message = PoseStamped()
        route = self.route
        metadata = self.exploration_metadata(route)
        newest = self.exploration_metadata(self.latest_route)
        same_owner = ((metadata is None and newest is None) or
                      (metadata is not None and newest is not None and
                       metadata['exploration_id'] == newest['exploration_id']))
        if (not explicit_partial and self.latest_route is not None and self.latest_route.poses
                and same_owner
                and self.latest_route.header.frame_id == route.header.frame_id
                and (metadata is not None or
                     distance(xy(self.latest_route.poses[-1].pose), xy(route.poses[-1].pose)) < .25)
                and stamp_ns(self.latest_route) >= stamp_ns(route)):
            route = self.latest_route
        message.header = deepcopy(route.header)
        message.pose = deepcopy(route.poses[-1].pose)
        self.route_rejection_pub.publish(message)
        return True

    def block_current_route(self, backoff=20.0):
        self.blocked_endpoint = self.route_endpoint
        self.blocked_route = ([xy(p.pose) for p in self.route.poses]
                              if self.route is not None else None)
        self.blocked_grid_hash = hash(tuple(self.grid.data)) if self.grid else None
        self.blocked_until = time.monotonic() + backoff


def main(args=None):
    rclpy.init(args=args)
    node = GraphPathBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

