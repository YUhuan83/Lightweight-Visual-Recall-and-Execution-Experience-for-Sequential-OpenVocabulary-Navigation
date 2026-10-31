"""Bounded escape over recently measured driving, separate from target memory."""
from collections import deque
import math
import time
from .graph_route import distance, distance_to_route


def reverse_escape(trace, position, heading, now, minimum_radius):
    """Fit a short reverse arc wholly within the recently driven corridor."""
    if (minimum_radius <= 0 or not all(map(math.isfinite, (*position, heading, now, minimum_radius)))):
        return None
    recent = [p for p in trace if 0 <= now-p[3] <= 60.]
    if len(recent) < 4 or distance(position, recent[-1][:2]) > .8:
        return None
    corridor = [p[:2] for p in recent]+[position]
    for old in reversed(recent):
        dx, dy = old[0]-position[0], old[1]-position[1]
        chord = math.hypot(dx, dy)
        if not 2. <= chord <= 4.:
            continue
        x = dx*math.cos(heading)+dy*math.sin(heading)
        y = -dx*math.sin(heading)+dy*math.cos(heading)
        if x >= 0:
            continue
        curvature = 2*y/(chord*chord)
        if abs(curvature) > 1/minimum_radius:
            continue
        turn = 2*math.atan2(-y, -x)
        error = math.atan2(math.sin(heading+turn-old[2]), math.cos(heading+turn-old[2]))
        if abs(error) > .4:
            continue
        length = turn/curvature if abs(curvature) > 1e-8 else x
        count = max(1, math.ceil(abs(length)/.2))
        poses = []
        for i in range(count+1):
            s = length*i/count
            angle = s*curvature
            lx = math.sin(angle)/curvature if abs(curvature) > 1e-8 else s
            ly = (1-math.cos(angle))/curvature if abs(curvature) > 1e-8 else 0.
            poses.append((position[0]+lx*math.cos(heading)-ly*math.sin(heading),
                          position[1]+lx*math.sin(heading)+ly*math.cos(heading), heading+angle))
        if all(distance_to_route(p[:2], corridor) <= .5 for p in poses):
            return poses
    return None


class ExecutionRecovery:
    def init_execution_recovery(self):
        self.escape_trace = deque(maxlen=96)
        self.escape_failures = deque(maxlen=8)
        self.planning_escape_failures = deque(maxlen=8)
        self.escape_attempt_position = None
        self.recovering = False
        self.escape_path = None
        self.deferred_execution_escape = None

    def remember_driven_pose(self, message):
        from .graph_path_bridge import xy, yaw
        now = time.monotonic()
        if (self.recovering or not self.following or self.control_status_at is None
                or now-self.control_status_at > 1. or not self.control_status.get('actuating')
                or self.control_status.get('target_speed', 0.) <= 0.):
            return
        point = xy(message.pose.pose)
        if self.escape_trace:
            step = distance(point, self.escape_trace[-1][:2])
            if step > 2.:
                self.escape_trace.clear()
            elif step < .3:
                return
        self.escape_trace.append((*point, yaw(message.pose.pose), now))

    def try_execution_escape(self, reason, details):
        from .graph_path_bridge import xy, yaw
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        if (getattr(self, 'carrier_model', 'ackermann') != 'ackermann'
                or reason != 'execution_stalled' or details.get('stall_class') not in
                ('actuated_no_path_progress', 'drive_limited_or_physical_contact',
                 'wheel_slip_or_contact_loss',
                 'wheel_airborne_spin', 'wheel_support_lost')
                or not self.follow_enabled or self.memory_goal is not None
                or self.query_reset_pending or self.odom is None
                or self.input_freshness_issue() is not None or self.local_freshness_issue() is not None):
            return False
        point = xy(self.odom.pose.pose)
        now = time.monotonic()
        self.escape_failures.append((*point, now))
        nearby = sum(0 <= now-p[2] <= 60. and distance(point, p[:2]) < 2.
                     for p in self.escape_failures)
        if nearby < 2 or (self.escape_attempt_position is not None
                         and distance(point, self.escape_attempt_position) < 3.):
            return False
        poses = reverse_escape(self.escape_trace, point, yaw(self.odom.pose.pose),
                               now, self.minimum_turning_radius)
        if poses is None:
            return False
        if self.follow_quarantine:
            # Keep an intent, never a prevalidated motion. The old action must
            # terminate and the vehicle settle before refitting at the actual
            # stopped pose and checking the current maps again.
            self.deferred_execution_escape = dict(scope=self.execution_escape_scope(),
                position=point, time=now)
            self.status('recovery_waiting_for_stop', failure_position=list(point))
            return True
        return self.start_measured_escape(point, poses)

    def execution_escape_scope(self):
        metadata = self.exploration_metadata(self.route)
        return (self.epoch, self.query_generation, self.odom.header.frame_id,
                metadata.get('exploration_id') if metadata else None,
                self.expected, self.memory_goal)

    def resume_execution_escape_after_stop(self):
        from .graph_path_bridge import xy, yaw
        intent, self.deferred_execution_escape = self.deferred_execution_escape, None
        if (intent is None or self.odom is None or not self.follow_enabled
                or self.query_reset_pending or self.memory_goal is not None
                or intent['scope'] != self.execution_escape_scope()
                or not 0. <= time.monotonic()-intent['time'] <= 60.
                or distance(xy(self.odom.pose.pose),intent['position']) > .8
                or self.input_freshness_issue() is not None
                or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None):
            return False
        speed=math.hypot(self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y)
        if not math.isfinite(speed) or speed > .15:
            return False
        point=xy(self.odom.pose.pose)
        poses=reverse_escape(self.escape_trace,point,yaw(self.odom.pose.pose),
                             time.monotonic(),self.minimum_turning_radius)
        return poses is not None and self.start_measured_escape(point,poses)

    def try_planning_escape(self):
        """Back out once when distinct destinations fail from the same pose.

        Empty planner results establish no terrain label. They can justify a
        bounded local recovery only over fresh, actually driven free space.
        """
        from .graph_path_bridge import xy, yaw
        if (getattr(self, 'carrier_model', 'ackermann') != 'ackermann'
                or not self.follow_enabled or self.following or self.pending
                or self.recovering or self.replacement_pending or self.follow_quarantine
                or self.memory_goal is not None or self.expected is not None or self.memory_selecting
                or self.query_reset_pending or self.odom is None
                or self.input_freshness_issue() is not None
                or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None
                or not self.failure_reasons
                or any(reason != 'empty_path' for reason in self.failure_reasons)):
            return False
        metadata = self.exploration_metadata(self.route)
        if metadata is None:
            return False
        point = xy(self.odom.pose.pose)
        speed = math.hypot(self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y)
        if not math.isfinite(speed) or speed > .15:
            return False
        now = time.monotonic()
        identity = metadata['exploration_id']
        self.planning_escape_failures.append((identity, *point, now))
        identities = {row[0] for row in self.planning_escape_failures
                      if 0 <= now-row[3] <= 60. and distance(point,row[1:3]) < 2.}
        if (len(identities) < 2 or (self.escape_attempt_position is not None
                and distance(point,self.escape_attempt_position) < 3.)):
            return False
        poses = reverse_escape(self.escape_trace,point,yaw(self.odom.pose.pose),
                               now,self.minimum_turning_radius)
        if poses is None:
            return False
        return self.start_measured_escape(point, poses)

    def start_measured_escape(self, point, poses):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        # A failure callback may have retired an action without receiving its
        # terminal result yet. The quarantine still owns motion in that gap.
        if (self.recovering or self.following or self.pending or self.replacement_pending
                or self.follow_quarantine or self.follow_request is not None
                or self.follow_active is not None):
            return False
        self.reject_exploration_route()
        path = Path()
        path.header.frame_id = self.odom.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, heading in poses:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(heading/2)
            pose.pose.orientation.w = math.cos(heading/2)
            path.poses.append(pose)
        previous_attempt = self.escape_attempt_position
        self.escape_attempt_position = point
        self.recovering = True
        self.escape_path = path
        self.blocked_endpoint = None
        self.motion_queue = [path]
        self.status('recovery_started', failure_position=list(point),
                    goal=list(poses[-1][:2]), measured_corridor=True)
        dispatched = self.send_motion_segment()
        if (dispatched is False and self.recovering and not self.following
                and self.follow_request is None and self.follow_active is None
                and not self.follow_quarantine):
            # Map/transport freshness or controller availability can change
            # at dispatch. No action owns this recovery; leave normal retries
            # usable rather than permanently blocking start_segment().
            self.recovering = False
            self.escape_path = None
            self.motion_queue = []
            self.escape_attempt_position = previous_attempt
        return True

    def finish_execution_escape(self, succeeded, reason, action_status=None):
        self.recovering = False
        self.escape_path = None
        self.deferred_execution_escape = None
        self.motion_queue = []
        self.escape_trace.clear()
        self.escape_failures.clear()
        self.planning_escape_failures.clear()
        self.last_plan_start = self.last_plan_endpoint = None
        self.status('recovery_done' if succeeded else 'recovery_failed', reason=reason)
        if succeeded:
            self.blocked_endpoint = None
            self.recovery_attempts = 0
            if self.latest_route is not None:
                self.route = self.latest_route
                self.route_complete = self.latest_route_complete
                self.route_endpoint = (float(self.route.poses[-1].pose.position.x),
                                       float(self.route.poses[-1].pose.position.y))
            self.start_segment()
        else:
            if self.latest_route is not None:
                self.route = self.latest_route
                self.route_complete = self.latest_route_complete
                self.route_endpoint = (float(self.route.poses[-1].pose.position.x),
                                       float(self.route.poses[-1].pose.position.y))
            # Follow the normal bounded retry path so the timer can resume the
            # newest GraphNav route after an escape action fails.
            self.follow_failed("follow_aborted", action_status=action_status,
                               recovery_reason=reason)
