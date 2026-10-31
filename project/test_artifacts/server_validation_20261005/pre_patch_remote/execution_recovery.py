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
        self.escape_attempt_position = None
        self.recovering = False
        self.escape_path = None

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
        if (reason != 'execution_stalled' or details.get('stall_class') not in
                ('drive_limited_or_physical_contact', 'wheel_slip_or_contact_loss')
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
        self.escape_attempt_position = point
        self.recovering = True
        self.escape_path = path
        self.blocked_endpoint = None
        self.motion_queue = [path]
        self.status('recovery_started', failure_position=list(point),
                    goal=list(poses[-1][:2]), measured_corridor=True)
        self.send_motion_segment()
        return True

    def finish_execution_escape(self, succeeded, reason):
        self.recovering = False
        self.escape_path = None
        self.motion_queue = []
        self.escape_trace.clear()
        self.escape_failures.clear()
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
            self.block_current_route(backoff=8.)
