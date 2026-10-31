"""Plan the next forward section while the accepted one is still running."""
from copy import deepcopy
import math
from .graph_route import distance, remaining_route

CONTINUOUS_HANDOFF_DISTANCE_M = 8.


def trim_moving_path(path, current, heading, *, allow_heading_change=False):
    """Prune the executed prefix; preserve heading checks for car-like motion."""
    from .motion_path import motion_sections
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    if len(points) < 2:
        return None
    sections = motion_sections(points)
    last = sections[0][1]
    index = min(range(last+1), key=lambda i: distance(points[i], current))
    if distance(points[index], current) > 1. or len(points) - index < 2:
        return None
    q = path.poses[index].pose.orientation
    planned_heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    if (not allow_heading_change and
            abs(math.atan2(math.sin(planned_heading-heading), math.cos(planned_heading-heading))) > .4):
        return None
    output = deepcopy(path)
    output.poses = output.poses[index:]
    return output


class ContinuousPath:
    def init_continuous_path(self):
        self.retired_motion = None
        self.extending_path = False
        self.extension_retry_at = 0.
        self.prepared_extension = None
        self.replacement_pending = False
        self.follow_request_serial = 0
        self.active_follow_serial = None
        self.deferred_follow_result = None

    def maybe_extend_path(self):
        import time
        if (self.retired_motion is not None or getattr(self, 'recovering', False) or not self.follow_enabled or not self.following or self.follow_handle is None
                or self.pending or self.extending_path or self.replacement_pending
                or self.memory_selecting or len(self.motion_queue) != 1
                or time.monotonic() < self.extension_retry_at or self.prepared_extension is not None):
            return
        from .graph_path_bridge import xy
        from .motion_path import motion_sections
        path = self.motion_queue[0]
        points = [xy(p.pose) for p in path.poses]
        if len(motion_sections(points)) != 1:
            return
        # Preempt only forward sections. Reverse ends and cusps require a
        # stop, a heading check and the actuator's direction-change handshake.
        q = path.poses[0].pose.orientation
        h = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        edge = (points[1][0]-points[0][0], points[1][1]-points[0][1])
        if (getattr(self, 'carrier_model', 'ackermann') != 'ideal_point'
                and edge[0]*math.cos(h)+edge[1]*math.sin(h) <= 0):
            return
        remaining = remaining_route(xy(self.odom.pose.pose), points)
        length = sum(distance(a, b) for a, b in zip(remaining, remaining[1:]))
        if length > CONTINUOUS_HANDOFF_DISTANCE_M:
            return
        if self.latest_route is not None:
            self.route, self.route_complete = self.latest_route, self.latest_route_complete
            self.route_endpoint = xy(self.route.poses[-1].pose)
        graph_remaining = self.current_route_points()
        if sum(distance(a, b) for a, b in zip(graph_remaining, graph_remaining[1:])) < length+4.:
            return
        self.extending_path = True
        self.start_segment(continuous=True)

    def finish_current_exploration_section(self, identity):
        """Retire an intention without revoking a healthy bounded action.

        Keep the original action, epoch and completion contract. The old tail
        and extensions never execute; the next intention is read again through
        on_route only after this accepted section produces a valid terminal.
        """
        from .graph_path_bridge import xy, yaw
        if self.retired_motion is not None:
            return self.retired_motion['identity'] == identity
        if (not self.follow_enabled or not self.following or self.follow_handle is None
                or self.follow_active is None or self.active_follow_serial is None
                or self.follow_active['serial'] != self.active_follow_serial
                or self.follow_request is not None or self.follow_quarantine or self.follow_hold
                or self.pending or self.extending_path or self.replacement_pending
                or self.prepared_extension is not None or self.frontier_choice is not None
                or self.frontier_comparison is not None or not self.motion_queue
                or self.input_freshness_issue() is not None
                or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None):
            return False
        accepted = self.motion_queue[0]
        frame = self.odom.header.frame_id
        if (accepted.header.frame_id != frame or self.grid.header.frame_id != frame
                or self.local_grid.header.frame_id != frame):
            return False
        path = trim_moving_path(accepted, xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                               allow_heading_change=self.carrier_model == 'ideal_point')
        if path is None:
            return False
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        length = sum(distance(a, b) for a, b in zip(points, points[1:]))
        if length > CONTINUOUS_HANDOFF_DISTANCE_M:
            return False
        issue = self.validate_path(points, headings, points, self.grid, points[-1],
                                   start=xy(self.odom.pose.pose))
        issue = issue or self.validate_local_path(points, headings, self.local_grid,
                                   xy(self.odom.pose.pose), yaw(self.odom.pose.pose))
        if issue:
            return False
        self.retired_motion = dict(identity=identity, epoch=self.epoch,
                                  serial=self.active_follow_serial)
        self.active_plan_serial = None  # Fence completed planner responses independently.
        self.status('exploration_finishing_segment', retired_exploration_id=identity,
                    remaining_distance_m=length, goal=list(points[-1]))
        return True

    def extension_failed(self, reason):
        import time
        self.pending = self.extending_path = False
        self.normal_best = None
        self.extension_retry_at = time.monotonic() + 2.
        # A failed extension does not cancel the valid accepted action.
        self.status('following' if self.following else 'segment_done',
                    extension_rejected=reason)
        # Completion may have arrived while the extension request was pending.
        # Its handler deferred handoff; resume through the normal freshness and
        # actuator guards now, for timeout and ordinary candidate rejection.
        if not self.following and not self.replacement_pending:
            self.start_segment()
