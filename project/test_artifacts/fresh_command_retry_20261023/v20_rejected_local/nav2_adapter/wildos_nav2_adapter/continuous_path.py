"""Plan the next forward section while the accepted one is still running."""
from copy import deepcopy
import math
import time
from .graph_route import distance, remaining_route

CONTINUOUS_HANDOFF_DISTANCE_M = 8.
# Exploration route replacement can take longer than the generic 5 s action
# handshake. Keep a separate bounded lease for already-accepted forward motion.
CONTINUOUS_HANDOFF_LEASE_DISTANCE_M = 16.
CONTINUOUS_HANDOFF_LEASE_SEC = 15.
# A timed-out visual hint may finish its current checked segment. Longer
# borrowed motion remains forward-only and uses a much shorter target lease.
VISIBLE_TARGET_HANDOFF_DISTANCE_M = 5.
VISIBLE_TARGET_HANDOFF_SEC = 5.


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
        self.prepared_stop_plan = None
        self.replacement_pending = False
        self.follow_request_serial = 0
        self.active_follow_serial = None
        self.deferred_follow_result = None

    def stop_plan_scope(self):
        return (self.epoch, self.query_generation,
                tuple(self.expected) if self.expected is not None else None,
                tuple(self.memory_goal) if self.memory_goal is not None else None,
                self.follow_enabled)

    def resume_prepared_stop_plan(self):
        """Admit a computed path only after the existing physical stop barrier."""
        prepared = self.prepared_stop_plan
        if prepared is None:
            return False
        path, target, scope, route = prepared
        if (route is not self.route or scope != self.stop_plan_scope()
                or self.object_found or self.query_reset_pending):
            self.prepared_stop_plan = None
            return False
        if self.follow_quarantine or self.follow_release is not None:
            return True
        self.prepared_stop_plan = None
        if self.pending or self.following or self.follow_request is not None or self.recovering:
            return False
        stale = self.input_freshness_issue() or self.local_freshness_issue()
        if stale:
            self.report_stale_input(stale)
            return True
        issue = self.execution_transport_issue()
        if issue is not None:
            self.execution_transport_unavailable(issue)
            return True
        from .graph_path_bridge import xy, yaw
        path = trim_moving_path(path, xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                                allow_heading_change=self.carrier_model == 'ideal_point')
        if path is None:
            self.status('prepared_path_start_changed', goal=list(target))
            return False
        # All geometry, tracking and current map checks are repeated here.
        self.execute_checked_path(path, target)
        return True

    def maybe_extend_path(self):
        if self.retired_motion is not None:
            self.maybe_replace_retired_section()
            return
        if (getattr(self, 'recovering', False) or not self.follow_enabled or not self.following or self.follow_handle is None
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
        if self.advance_visible_goal() or self.visible_goal_refresh_pending:
            return  # Keep driving while GraphNav supplies the next intention.
        if self.latest_route is not None:
            self.route, self.route_complete = self.latest_route, self.latest_route_complete
            self.route_endpoint = xy(self.route.poses[-1].pose)
        graph_remaining = self.current_route_points()
        if sum(distance(a, b) for a, b in zip(graph_remaining, graph_remaining[1:])) < length+4.:
            return
        self.extending_path = True
        self.start_segment(continuous=True)

    def finish_current_exploration_section(self, identity, *, owner_kind='exploration'):
        """Retire an intention without revoking a healthy bounded action.

        Keep the accepted action/epoch while a new intention is planned. Short
        sections may finish; long forward sections borrow a distance/time lease.
        The old queue tail never executes after retirement.
        """
        from .graph_path_bridge import xy, yaw
        if self.retired_motion is not None:
            return self.retired_motion['identity'] == identity
        if (not self.follow_enabled or not self.following or self.follow_handle is None
                or self.follow_active is None or self.active_follow_serial is None
                or self.follow_active['serial'] != self.active_follow_serial
                or self.follow_request is not None or self.follow_quarantine or self.follow_hold
                or (self.pending and not self.extending_path) or self.replacement_pending
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
        edge = next(((b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])
                     if distance(a, b) > 1e-9), None)
        forward = (edge is not None and
                   edge[0]*math.cos(headings[0])+edge[1]*math.sin(headings[0]) > 0.)
        bounded = length > CONTINUOUS_HANDOFF_DISTANCE_M
        if bounded and not forward:
            return False
        issue = self.validate_path(points, headings, points, self.grid, points[-1],
                                   start=xy(self.odom.pose.pose))
        issue = issue or self.validate_local_path(points, headings, self.local_grid,
                                   xy(self.odom.pose.pose), yaw(self.odom.pose.pose))
        if issue:
            return False
        if self.extending_path:
            # Retiring an intention invalidates its future extension, not the
            # already accepted and revalidated controller section. A late
            # planner acceptance/result is fenced by its request serial.
            self.cancel_goal(self.plan_handle, 'retired_exploration_extension')
            self.plan_handle = self.active_plan_serial = self.plan_request_started_at = None
            self.pending = self.extending_path = self.candidate_waiting_for_map = False
            self.normal_best = self.prepared_extension = None
        target_handoff = owner_kind == 'visible_target_timeout'
        self.retired_motion = dict(identity=identity, owner_kind=owner_kind, epoch=self.epoch,
            serial=self.active_follow_serial, bounded=bounded, forward=forward,
            deadline=time.monotonic()+(VISIBLE_TARGET_HANDOFF_SEC if target_handoff
                                      else CONTINUOUS_HANDOFF_LEASE_SEC),
            distance_limit=(VISIBLE_TARGET_HANDOFF_DISTANCE_M if target_handoff
                            else CONTINUOUS_HANDOFF_LEASE_DISTANCE_M),
            last_position=xy(self.odom.pose.pose), travelled=0., route=self.route,
            route_complete=self.route_complete, route_endpoint=self.route_endpoint)
        self.active_plan_serial = None  # Fence completed planner responses independently.
        if target_handoff:
            self.status('visible_target_handoff_finishing_segment',
                        retired_approach_id=identity,
                        remaining_distance_m=length, goal=list(points[-1]))
        else:
            self.status('exploration_finishing_segment', retired_exploration_id=identity,
                        remaining_distance_m=length, goal=list(points[-1]))
        return True

    def retired_replacement_context(self):
        """A background plan can only target a fresh, live exploration owner."""
        from .input_quality import stamp_ns
        motion = self.retired_motion
        metadata = self.exploration_metadata(self.route)
        if motion is None or metadata is None or not self.route.poses:
            return False
        age = (self.get_clock().now().nanoseconds-stamp_ns(self.route))*1e-9
        return (metadata['exploration_id'] != motion['identity']
                and metadata['exploration_id'] not in self.retired_explorations
                and self.route.header.frame_id == motion['route'].header.frame_id
                and stamp_ns(self.route) >= self.minimum_route_stamp
                and not self.query_reset_pending and self.expected is None and self.memory_goal is None
                and 0. <= age <= 5.)

    def retired_tail_candidate(self):
        """Snapshot only the not-yet-executed, contiguous old route suffix."""
        from nav_msgs.msg import Path
        from .graph_path_bridge import xy
        queue = self.motion_queue[1:]
        if not queue or len(queue) > 3:
            return None
        frame = queue[0].header.frame_id
        poses = []
        for segment in queue:
            if segment.header.frame_id != frame or len(segment.poses) < 2:
                return None
            if poses and distance(xy(poses[-1].pose), xy(segment.poses[0].pose)) > .15:
                return None
            poses.extend(segment.poses[1:] if poses else segment.poses)
        if len(poses) < 2:
            return None
        path = Path()
        path.header = deepcopy(queue[0].header)
        path.poses = deepcopy(poses)
        length = sum(distance(xy(a.pose), xy(b.pose))
                     for a, b in zip(path.poses, path.poses[1:]))
        return path if length <= 20. else None

    def try_retired_tail_candidate(self, path, retired_identity):
        """Reuse a retired suffix only after admission against the new owner/maps."""
        from .graph_path_bridge import xy, yaw
        metadata = self.exploration_metadata(self.route)
        if (metadata is None
                or metadata['exploration_id'] == retired_identity
                or metadata['exploration_id'] in self.retired_explorations
                or self.input_freshness_issue() is not None
                or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None
                or self.expected is not None or self.memory_goal is not None
                or self.recovering or self.follow_quarantine or self.query_reset_pending
                or path.header.frame_id != self.grid.header.frame_id
                or self.local_grid.header.frame_id != path.header.frame_id):
            return False
        current = xy(self.odom.pose.pose)
        goal = metadata['goal'][:2]
        candidate = trim_moving_path(path, current, yaw(self.odom.pose.pose),
                                     allow_heading_change=self.carrier_model == 'ideal_point')
        if candidate is None:
            return False
        points = [xy(p.pose) for p in candidate.poses]
        headings = [yaw(p.pose) for p in candidate.poses]
        if len(points) < 2 or distance(current, goal) - distance(points[-1], goal) < 1.0:
            return False
        reason = self.validate_path(points, headings,
            [current] + self.current_route_points(), self.grid, points[-1], start=current)
        reason = reason or self.validate_local_path(points, headings, self.local_grid,
                                                    current, yaw(self.odom.pose.pose))
        reason = reason or self.exploration_motion_issue(points)
        reason = reason or self.initial_tracking_issue(points, headings)
        if reason:
            self.status('retired_tail_candidate_rejected', reason=reason,
                        exploration_id=metadata['exploration_id'])
            return False
        self.motion_queue = [candidate]
        self.status('retired_tail_candidate_revalidated',
                    exploration_id=metadata['exploration_id'], goal=list(points[-1]))
        self.execute_checked_path(candidate, points[-1])
        return True

    def stop_retired_motion(self, reason, **details):
        motion = self.retired_motion
        latest = self.latest_route
        # An observed terminal closes the action, not the physical stopping
        # interval. Keep the same standstill barrier as canceled actions.
        if motion is not None and motion.get('terminal'):
            self.follow_quarantine.setdefault(motion['serial'], dict(handle=None, terminal=True))
        self.finish_execution_feedback(False, reason)
        self.reset(preserve_execution_corridor=True)
        if motion is not None and motion.get('owner_kind') == 'visible_target_timeout':
            self.status('visible_target_handoff_stopped', reason=reason,
                        retired_approach_id=motion['identity'], **details)
        else:
            self.status('exploration_handoff_stopped', reason=reason,
                        retired_exploration_id=motion['identity'] if motion else None, **details)
        if latest is not None:
            self.on_route(latest)  # Existing freshness and action quarantine apply.

    def maybe_replace_retired_section(self):
        from .graph_path_bridge import xy, yaw
        motion = self.retired_motion
        if motion is None or not self.following:
            return
        if (self.follow_active is None or self.active_follow_serial != motion['serial']
                or self.follow_active['serial'] != motion['serial'] or not self.motion_queue
                or self.epoch != motion['epoch']):
            self.stop_retired_motion('motion_identity_changed')
            return
        current = xy(self.odom.pose.pose)
        motion['travelled'] += distance(current, motion['last_position'])
        motion['last_position'] = current
        distance_limit = motion.get('distance_limit', CONTINUOUS_HANDOFF_LEASE_DISTANCE_M)
        if motion['bounded'] and (motion['travelled'] >= distance_limit
                                 or time.monotonic() >= motion['deadline']):
            self.stop_retired_motion('motion_lease_exhausted')
            return
        if (self.input_freshness_issue() is not None or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None):
            self.stop_retired_motion('handoff_inputs_unavailable')
            return
        speed = math.hypot(self.odom.twist.twist.linear.x,
                           self.odom.twist.twist.linear.y)
        stationary = math.isfinite(speed) and speed <= .05
        accepted = trim_moving_path(
            self.motion_queue[0], current, yaw(self.odom.pose.pose),
            allow_heading_change=(self.carrier_model == 'ideal_point'
                                  or stationary))
        if accepted is None:
            self.stop_retired_motion('handoff_actual_pose_cannot_join')
            return
        points = [xy(p.pose) for p in accepted.poses]
        headings = [yaw(p.pose) for p in accepted.poses]
        # Repeat the existing swept-prefix checks on both latest maps while
        # borrowing motion. RPP still checks its actual bounded command arc.
        for grid in (self.grid, self.local_grid):
            if self.validate_local_path(points, headings, grid, current, yaw(self.odom.pose.pose)):
                self.stop_retired_motion('handoff_prefix_blocked')
                return
        if (not motion['forward'] or self.pending or self.extending_path or self.replacement_pending
                or self.follow_request is not None or self.follow_quarantine
                or self.prepared_extension is not None or time.monotonic() < self.extension_retry_at
                or self.latest_route is None):
            return
        self.route = self.latest_route
        self.route_complete = self.latest_route_complete
        self.route_endpoint = xy(self.route.poses[-1].pose)
        if not self.retired_replacement_context():
            return
        self.extending_path = True
        self.start_segment(continuous=True)

    def retire_background_exploration(self, identity):
        """Revoke only a not-yet-dispatched replacement intention, not motion."""
        motion = self.retired_motion
        if (motion is None or identity == motion['identity'] or not self.following
                or self.follow_request is not None or self.replacement_pending
                or self.follow_quarantine or self.active_follow_serial != motion['serial']):
            return False
        self.cancel_goal(self.plan_handle, 'retired_background_compute_path')
        self.plan_handle = self.active_plan_serial = self.plan_request_started_at = None
        self.pending = self.extending_path = self.candidate_waiting_for_map = False
        self.normal_best = self.prepared_extension = None
        self.route, self.route_complete = motion['route'], motion['route_complete']
        self.route_endpoint = motion['route_endpoint']
        metadata = self.exploration_metadata(self.latest_route)
        if metadata and metadata['exploration_id'] == identity:
            self.latest_route = None
        self.status('exploration_handoff_waiting', retired_exploration_id=identity)
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
