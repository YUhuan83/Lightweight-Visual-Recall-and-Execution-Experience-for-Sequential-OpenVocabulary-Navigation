"""Plan WildOS target poses with Nav2 and optionally follow the resulting path."""
from __future__ import annotations

import json
import math
import time

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from .planning import heading_changed, should_replan, should_retry_follow


def pose_yaw(message: PoseStamped) -> float | None:
    """Return a valid planar heading, or None for an unspecified quaternion."""
    q = message.pose.orientation
    values = (float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6:
        return None
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class PathPlannerBridge(Node):
    def __init__(self):
        super().__init__("wildos_path_planner_bridge")
        self.declare_parameter("goal_topic", "/wildos/goal_pose")
        # Only a replayed in-process memory target should override GraphNav.
        # /spot1/imgnav_waypoint also carries ordinary perception/frontier goals.
        self.declare_parameter("direct_goal_topic", "/spot1/remembered_object_goal")
        self.declare_parameter("query_topic", "/spot1/object_search/query")
        self.declare_parameter("action_name", "/compute_path_to_pose")
        # Keep the original graph-following planner for normal exploration.
        # Replayed memory goals use a curvature-constrained planner; never
        # fall back to NavFn for those direct vehicle maneuvers by default.
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("direct_planner_id", "CarLike")
        self.declare_parameter("direct_fallback_planner_id", "")
        self.declare_parameter("min_goal_distance", 1.0)
        self.declare_parameter("min_direct_heading_change", 0.35)
        self.declare_parameter("result_topic", "/wildos/planned_path")
        self.declare_parameter("status_topic", "/wildos/plan_status")
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.direct_planner_id = str(self.get_parameter("direct_planner_id").value)
        self.direct_fallback_planner_id = str(
            self.get_parameter("direct_fallback_planner_id").value)
        self.min_distance = float(self.get_parameter("min_goal_distance").value)
        self.min_direct_heading_change = float(
            self.get_parameter("min_direct_heading_change").value)
        if self.min_direct_heading_change <= 0:
            raise ValueError("min_direct_heading_change must be positive")
        self.path_pub = self.create_publisher(
            PathMsg, str(self.get_parameter("result_topic").value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.declare_parameter("follow_path", False)
        self.declare_parameter("follow_action", "/follow_path")
        self.declare_parameter("max_follow_retries", 2)
        self.declare_parameter("follow_retry_backoff", 1.0)
        self.declare_parameter("plan_failure_retry_delay", 5.0)
        self.declare_parameter("max_direct_plan_retries", 3)
        self.follow = bool(self.get_parameter("follow_path").value)
        self.max_follow_retries = int(self.get_parameter("max_follow_retries").value)
        self.follow_retry_backoff = float(
            self.get_parameter("follow_retry_backoff").value)
        self.plan_failure_retry_delay = float(
            self.get_parameter("plan_failure_retry_delay").value)
        self.max_direct_plan_retries = int(
            self.get_parameter("max_direct_plan_retries").value)
        if self.max_follow_retries < 0:
            raise ValueError("max_follow_retries must be >= 0")
        if self.follow_retry_backoff <= 0:
            raise ValueError("follow_retry_backoff must be positive")
        if self.plan_failure_retry_delay <= 0:
            raise ValueError("plan_failure_retry_delay must be positive")
        if self.max_direct_plan_retries < 1:
            raise ValueError("max_direct_plan_retries must be >= 1")
        self.follow_client = (
            ActionClient(self, FollowPath, str(self.get_parameter("follow_action").value))
            if self.follow else None)
        self.client = ActionClient(
            self, ComputePathToPose, str(self.get_parameter("action_name").value))
        self.last_goal = None
        self.last_direct_yaw = None
        self.last_attempt_goal = None
        self.plan_failure_retry_due = 0.0
        self.latest_goal_xy = None
        self.latest_goal_message = None
        self.latest_graph_goal_message = None
        self.pending = False
        self.follow_handle = None
        self.follow_generation = 0
        self.plan_epoch = 0
        self.active_query = None
        self.direct_mode = False
        self.direct_plan_failure_count = 0
        self.follow_retry_count = 0
        self.retry_goal_message = None
        self.retry_due = 0.0
        self.last_status = {"status": "idle", "goal": None, "poses": 0}
        self.create_subscription(
            PoseStamped, str(self.get_parameter("goal_topic").value),
            self.on_graph_goal, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("direct_goal_topic").value),
            self.on_direct_goal, 10)
        self.create_subscription(
            String, str(self.get_parameter("query_topic").value),
            self.on_search_query, 10)
        self.create_timer(1.0, self.republish_status)
        self.create_timer(0.2, self.retry_tick)
        mode = "planning and path following" if self.follow else "planning only"
        self.get_logger().info(
            f"WildOS -> Nav2 bridge started ({mode})")

    def publish_status(self, payload: dict) -> None:
        self.last_status = {"mode": "direct" if self.direct_mode else "graph", **payload}
        self.status_pub.publish(String(data=json.dumps(self.last_status)))
        self.get_logger().info("plan status: " + json.dumps(self.last_status))

    def republish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self.last_status)))

    def reset_route(self) -> None:
        """Invalidate outstanding action callbacks when the target source changes."""
        self.plan_epoch += 1
        self.pending = False
        if self.follow_handle is not None:
            self.follow_handle.cancel_goal_async()
            self.follow_handle = None
        self.follow_generation += 1
        self.last_goal = None
        self.last_direct_yaw = None
        self.last_attempt_goal = None
        self.plan_failure_retry_due = 0.0
        self.direct_plan_failure_count = 0
        self.latest_goal_xy = None
        self.latest_goal_message = None
        self.follow_retry_count = 0
        self.retry_goal_message = None
        self.retry_due = 0.0

    def on_search_query(self, message: String) -> None:
        query = " ".join(message.data.casefold().split())
        if not query or query == self.active_query:
            return
        self.active_query = query
        self.latest_graph_goal_message = None
        self.reset_route()
        self.direct_mode = False
        self.get_logger().info("Object-search target changed to %r" % query)

    def on_graph_goal(self, message: PoseStamped) -> None:
        self.latest_graph_goal_message = message
        if not self.direct_mode:
            self.on_goal(message)

    def on_direct_goal(self, message: PoseStamped) -> None:
        yaw = pose_yaw(message)
        goal_xy = (message.pose.position.x, message.pose.position.y)
        new_position = should_replan(
            self.latest_goal_xy, goal_xy, self.min_distance)
        new_heading = heading_changed(
            self.last_direct_yaw, yaw, self.min_direct_heading_change)
        if not self.direct_mode or new_position or new_heading:
            self.reset_route()
        self.direct_mode = True
        if new_position or new_heading or self.last_direct_yaw is None:
            self.last_direct_yaw = yaw
        self.on_goal(message)

    def on_goal(self, message: PoseStamped) -> None:
        goal_xy = (message.pose.position.x, message.pose.position.y)
        if should_replan(self.latest_goal_xy, goal_xy, self.min_distance):
            self.follow_retry_count = 0
            self.retry_goal_message = None
            self.retry_due = 0.0
        self.latest_goal_xy = goal_xy
        self.latest_goal_message = message
        if self.pending:
            return
        if self.retry_goal_message is not None:
            return
        if not should_replan(self.last_goal, goal_xy, self.min_distance):
            return
        if (not should_replan(self.last_attempt_goal, goal_xy, self.min_distance)
                and time.monotonic() < self.plan_failure_retry_due):
            return
        self.plan_failure_retry_due = 0.0
        self.last_attempt_goal = goal_xy
        self.get_logger().info("%s goal received: %.2f %.2f" % (
            "direct" if self.direct_mode else "graph", *goal_xy))
        if self.follow_handle is not None:
            self.follow_handle.cancel_goal_async()
            self.follow_handle = None
            self.follow_generation += 1
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.publish_status({"status": "planner_unavailable", "goal": list(goal_xy)})
            self.handle_plan_failure(goal_xy, "planner unavailable")
            return
        planner_id = (self.direct_planner_id if self.direct_mode
                      else self.planner_id)
        self.request_path(message, goal_xy, self.plan_epoch, planner_id)

    def request_path(self, message, goal_xy, epoch, planner_id):
        request = ComputePathToPose.Goal()
        request.use_start = False
        request.planner_id = planner_id
        request.goal = message
        self.pending = True
        future = self.client.send_goal_async(request)
        future.add_done_callback(
            lambda f: self.on_accepted(f, goal_xy, epoch, planner_id))

    def on_accepted(self, future, goal_xy, epoch, planner_id=None) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            if epoch == self.plan_epoch:
                self.pending = False
                self.publish_status({"status": "planner_error", "goal": list(goal_xy),
                                     "planner_id": planner_id,
                                     "detail": str(exc)[:200]})
                self.handle_plan_failure(goal_xy, str(exc))
            return
        if epoch != self.plan_epoch:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.pending = False
            if (self.latest_goal_xy is not None
                    and should_replan(goal_xy, self.latest_goal_xy,
                                      self.min_distance)):
                self.on_goal(self.latest_goal_message)
                return
            self.publish_status({"status": "goal_rejected", "goal": list(goal_xy)})
            self.handle_plan_failure(goal_xy, "planner rejected goal")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.on_result(f, goal_xy, epoch, planner_id))

    def on_result(self, future, goal_xy, epoch, planner_id=None) -> None:
        if epoch != self.plan_epoch:
            return
        self.pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.publish_status({"status": "planner_error", "goal": list(goal_xy),
                                 "planner_id": planner_id,
                                 "detail": str(exc)[:200]})
            self.handle_plan_failure(goal_xy, str(exc))
            return
        if (self.latest_goal_xy is not None
                and should_replan(goal_xy, self.latest_goal_xy, self.min_distance)):
            self.on_goal(self.latest_goal_message)
            return
        raw_status = None if result is None else getattr(result, "status", None)
        action_status = None if raw_status is None else int(raw_status)
        plan_result = getattr(result, "result", None) if result is not None else None
        path = getattr(plan_result, "path", None)
        poses = (list(path.poses) if path is not None
                 and action_status == GoalStatus.STATUS_SUCCEEDED else [])
        if not poses:
            fallback_id = self.direct_fallback_planner_id
            if (self.direct_mode and planner_id == self.direct_planner_id
                    and fallback_id and fallback_id != planner_id
                    and self.latest_goal_message is not None):
                self.get_logger().warning(
                    f"{planner_id} found no path for remembered target; "
                    f"retrying with {fallback_id}")
                fallback_xy = self.latest_goal_xy or goal_xy
                self.request_path(
                    self.latest_goal_message, fallback_xy, epoch, fallback_id)
                return
            failure = {"status": "empty_path", "goal": list(goal_xy),
                       "planner_id": planner_id, "action_status": action_status}
            error_code = getattr(plan_result, "error_code", None)
            error_msg = getattr(plan_result, "error_msg", None)
            if isinstance(error_code, int):
                failure["error_code"] = error_code
            if isinstance(error_msg, str) and error_msg:
                failure["detail"] = error_msg[:200]
            self.publish_status(failure)
            self.handle_plan_failure(
                goal_xy, str(failure.get("detail", "planner returned no path")))
            return
        self.plan_failure_retry_due = 0.0
        self.last_goal = goal_xy
        self.direct_plan_failure_count = 0
        out = PathMsg()
        out.header = path.header
        out.poses = poses
        self.path_pub.publish(out)
        self.publish_status({
            "status": "planned",
            "goal": [float(goal_xy[0]), float(goal_xy[1])],
            "poses": len(poses),
            "planner_id": planner_id,
        })
        if self.follow and self.follow_client is not None:
            if self.follow_client.wait_for_server(timeout_sec=2.0):
                follow_goal = FollowPath.Goal()
                follow_goal.path = out
                follow_goal.controller_id = "FollowPath"
                follow_goal.goal_checker_id = "goal_checker"
                future = self.follow_client.send_goal_async(follow_goal)
                future.add_done_callback(
                    lambda f: self.on_follow_accepted(
                        f, goal_xy, len(poses), epoch, planner_id))
            else:
                self.publish_status({"status": "controller_unavailable",
                                     "goal": list(goal_xy)})
                self.last_goal = None
                self.handle_plan_failure(goal_xy, "controller unavailable")

    def on_follow_accepted(self, future, goal_xy, poses, epoch, planner_id=None):
        try:
            handle = future.result()
        except Exception:
            if epoch == self.plan_epoch:
                self.schedule_follow_retry(goal_xy, status=None)
            return
        if epoch != self.plan_epoch:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.schedule_follow_retry(goal_xy, status=None)
            return
        if (self.latest_goal_xy is not None
                and should_replan(goal_xy, self.latest_goal_xy, self.min_distance)):
            handle.cancel_goal_async()
            return
        self.follow_generation += 1
        generation = self.follow_generation
        self.follow_handle = handle
        self.publish_status({
            "status": "following",
            "goal": [float(goal_xy[0]), float(goal_xy[1])],
            "poses": poses,
            "planner_id": planner_id,
        })
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.on_follow_result(
                f, goal_xy, generation, epoch, planner_id))

    def on_follow_result(self, future, goal_xy, generation, epoch, planner_id=None):
        if epoch != self.plan_epoch or generation != self.follow_generation:
            return
        self.follow_handle = None
        try:
            result = future.result()
        except Exception:
            result = None
        status = None if result is None else int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.follow_retry_count = 0
            self.retry_goal_message = None
            self.publish_status({"status": "follow_done", "goal": list(goal_xy),
                                 "action_status": status})
        else:
            self.schedule_follow_retry(goal_xy, status)

    def schedule_follow_retry(self, goal_xy, status):
        if (self.latest_goal_xy is None
                or should_replan(goal_xy, self.latest_goal_xy, self.min_distance)):
            return
        retry = should_retry_follow(
            status, self.follow_retry_count, self.max_follow_retries,
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED)
        if not retry:
            self.publish_status({"status": "follow_failed", "goal": list(goal_xy),
                                 "action_status": status,
                                 "retries": self.follow_retry_count})
            self.last_goal = None
            if self.direct_mode:
                self.abandon_direct_goal(goal_xy, "FollowPath retries exhausted")
            else:
                self.schedule_plan_retry()
            return
        self.follow_retry_count += 1
        # Clear goal de-duplication so Nav2 plans again from the current pose.
        self.last_goal = None
        self.retry_goal_message = self.latest_goal_message
        delay = self.follow_retry_backoff * self.follow_retry_count
        self.retry_due = time.monotonic() + delay
        self.publish_status({"status": "follow_retry_scheduled", "goal": list(goal_xy),
                             "action_status": status,
                             "retry": self.follow_retry_count,
                             "retry_in_seconds": delay})

    def schedule_plan_retry(self):
        """Retry a failed target slowly, while allowing a changed target at once."""
        self.plan_failure_retry_due = (
            time.monotonic() + self.plan_failure_retry_delay)

    def handle_plan_failure(self, goal_xy, reason):
        if self.direct_mode:
            self.direct_plan_failure_count += 1
            if self.direct_plan_failure_count >= self.max_direct_plan_retries:
                self.abandon_direct_goal(goal_xy, reason)
                return
        self.schedule_plan_retry()

    def abandon_direct_goal(self, goal_xy, reason):
        failures = self.direct_plan_failure_count
        graph_goal = self.latest_graph_goal_message
        self.get_logger().warning(
            f"abandoning remembered goal {goal_xy} after {failures} failed "
            f"planning/follow attempts: {reason}")
        self.reset_route()
        self.direct_mode = False
        self.publish_status({"status": "direct_goal_abandoned",
                             "goal": [float(goal_xy[0]), float(goal_xy[1])],
                             "retries": failures,
                             "detail": str(reason)[:200]})
        if graph_goal is not None:
            self.on_goal(graph_goal)

    def retry_tick(self):
        now = time.monotonic()
        if self.pending:
            return
        if self.retry_goal_message is not None:
            if now >= self.retry_due:
                message = self.retry_goal_message
                self.retry_goal_message = None
                self.retry_due = 0.0
                self.on_goal(message)
            return
        if (self.plan_failure_retry_due and now >= self.plan_failure_retry_due
                and self.latest_goal_message is not None):
            self.plan_failure_retry_due = 0.0
            self.on_goal(self.latest_goal_message)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
