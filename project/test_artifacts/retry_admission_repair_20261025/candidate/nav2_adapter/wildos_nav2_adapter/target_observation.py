"""Bounded, car-like observation approaches around a remembered object region."""
import math
import time
import json
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .graph_route import distance
from .sensor_geometry import observation_clearance
from .arrival_criteria import STANDOFF_ARRIVAL_TOLERANCE_M
from .observation_pose import connected_free_cells, observation_pose_clear


def live_observation_approaches(target, current, minimum_radius, limit=4):
    """Create bounded new baselines around carried geometry, never success evidence."""
    if (len(target) < 2 or not all(map(math.isfinite, target))
            or not math.isfinite(minimum_radius) or minimum_radius <= 0):
        return []
    base = math.atan2(current[1]-target[1], current[0]-target[0])
    radius = max(minimum_radius, min(6., distance(current, target)))
    candidates = []
    for offset in (-.35, .35, -.65, .65):
        x, y = target[0]+radius*math.cos(base+offset), target[1]+radius*math.sin(base+offset)
        if distance((x, y), current) >= 1.1:
            candidates.append((x, y, math.atan2(target[1]-y, target[0]-x)))
    return candidates[:limit]


def observation_approaches(view, current, limit=6):
    target = view.get('object_position')
    if not target:
        return []  # A legacy observer pose is not an object position.
    origin = view['position']
    radius = max(6., min(18., distance(origin, target)))
    base = math.atan2(origin[1]-target[1], origin[0]-target[0])
    candidates = []
    for offset in (0., -.52, .52, -1.05, 1.05, math.pi):
        x, y = target[0]+radius*math.cos(base+offset), target[1]+radius*math.sin(base+offset)
        if distance((x, y), current) < 2.:
            continue
        candidates.append((x, y, math.atan2(target[1]-y, target[0]-x)))
    return sorted(candidates, key=lambda p: distance(p, current))[:limit]


class TargetObservation:
    def init_observation(self, preserve_instance=False):
        release = getattr(self, 'instance_release_pending', None) if preserve_instance else None
        fence = getattr(self, 'confirmation_not_before_ns', 0) if preserve_instance else 0
        evidence_fence = getattr(self, 'confirmation_evidence_not_before_ns', 0) if preserve_instance else 0
        instance = getattr(self, 'upstream_instance_id', '') if preserve_instance else ''
        self.observation_attempts = set()
        self.observation_wait_until = None
        self.observation_active = False
        self.unconfirmed_approaches = []
        self.live_observation_target = None
        self.live_observation_deadline = None
        self.instance_release_pending = release
        self.confirmation_not_before_ns = fence
        self.confirmation_evidence_not_before_ns = evidence_fence
        self.upstream_instance_id = instance

    def request_instance_release(self):
        if self.instance_release_pending is not None:
            return
        anchor = self.active_object_confirmation or {}
        now = self.get_clock().now().nanoseconds
        self.instance_release_pending = dict(query=self.query,
            instance_id=anchor.get('instance_id') or self.upstream_instance_id, request_stamp_ns=now,
            observation_fence_ns=now,
            navigation_failed=bool(self.failed_confirmed_goals),
            request_id=f'{self.query_generation}:{now}')
        self.confirmation_not_before_ns = now

    def retry_instance_release(self):
        request = self.instance_release_pending
        if request is not None:
            # Retries retain identity. Renew transport age only; do not change
            # the observation fence established when the instance was abandoned.
            payload = dict(request, request_stamp_ns=self.get_clock().now().nanoseconds)
            self.instance_release_pub.publish(String(data=json.dumps(payload)))

    def on_instance_release_ack(self, message):
        try:
            data = json.loads(message.data)
            request = self.instance_release_pending
            if (request is None or data.get('released') is not True
                    or any(data.get(k) != request[k] for k in ('query', 'request_id', 'instance_id'))):
                return
            ack_stamp = int(data['request_stamp_ns'])
            now = self.get_clock().now().nanoseconds
            if not request['request_stamp_ns'] <= ack_stamp <= now+50_000_000:
                return
            self.instance_release_pending = None
            self.upstream_instance_id = ''
            self.confirmation_not_before_ns = max(self.confirmation_not_before_ns,
                                                 ack_stamp)
            self.status('target_instance_released', query=self.query,
                        instance_id=request['instance_id'])
            self.retry_graph_reset()
        except (ValueError, TypeError, KeyError):
            return

    def start_live_observation(self):
        anchor = self.active_object_confirmation
        if anchor is None or not anchor.get('object_position'):
            return False
        from .graph_path_bridge import xy
        target = anchor['object_position']
        current = xy(self.odom.pose.pose)
        clearance = observation_clearance(self.get_parameter('camera_offset').value,
            float(self.get_parameter('minimum_camera_target_range').value),
            STANDOFF_ARRIVAL_TOLERANCE_M)
        options = live_observation_approaches(target, current, clearance)
        deferred = self.deferred_confirmed_approach
        if (self.confirmed_approach_failure is not None and deferred is not None
                and deferred.get('query') == self.query and self.confirmed_instance_matches(deferred)):
            # Radial progress is not reachability: after failure a sideways or
            # more distant view of this same instance may be the useful one.
            x, y = deferred['standoff']
            options.insert(0, (x, y, math.atan2(target[1]-y, target[0]-x)))
        connected=(connected_free_cells(self.grid,current)
                   if self.carrier_model!='ideal_point' else None)
        for x, y, heading in options:
            key = (round(x, 1), round(y, 1))
            if (key in self.observation_attempts or len(self.observation_attempts) >= 4
                    or self.failed_confirmed_view((x, y))):
                continue
            if self.pose_blockers(self.grid, (x, y), heading):
                continue
            if connected is not None and not observation_pose_clear(
                    self.grid,connected,(x,y),heading):
                continue
            self.observation_attempts.add(key)
            pose = PoseStamped()
            pose.header.frame_id = 'spot1/odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.position.z = self.odom.pose.pose.position.z
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            self.reset(preserve_execution_corridor=True)
            self.confirmed_approach_failure = None
            self.observation_active = True
            self.live_observation_target = tuple(target)
            self.live_observation_deadline = time.monotonic()+30.
            self.observation_wait_until = None
            self.expected = self.confirmed_route_goal = (x, y)
            if self.confirmed_route_object is None:
                self.confirmed_route_object = tuple(target[:2])
            self.confirmed_route_started_at = time.monotonic()
            self.deferred_confirmed_approach = None
            self.relay_destination(pose)
            self.status('target_observation_approach', goal=[x, y],
                        object_position=list(target), approaches_tried=len(self.observation_attempts),
                        requires_visual_confirmation=True)
            return True
        return False

    def recently_unconfirmed(self, point):
        now = time.monotonic()
        self.unconfirmed_approaches = [(p, deadline) for p, deadline in
                                      self.unconfirmed_approaches if deadline > now]
        return any(distance(point, p) < 5. for p, _ in self.unconfirmed_approaches)

    def await_object_observation(self):
        if self.object_found or self.expected is None:
            return
        if getattr(self, 'visible_approach_active', False):
            self.finish_visible_approach()
            return
        if self.consume_confirmed_approach():
            return
        if self.observation_wait_until is None:
            self.live_observation_deadline = None
            self.observation_wait_until = time.monotonic() + 5.
            self.status('awaiting_visual_confirmation', goal=list(self.expected),
                        requires_visual_confirmation=True,
                        approaches_tried=len(self.observation_attempts))

    def monitor_observation(self):
        self.consume_pending_odometry()
        if (self.object_found or self.query_reset_pending or self.execution_recovery_owns_motion()):
            return
        if (self.memory_active is None and self.confirmed_route_goal is not None
                and self.confirmed_route_started_at is not None and self.route is None
                and not self.pending and not self.following and not self.follow_quarantine
                and self.input_freshness_issue() is None
                and time.monotonic()-self.confirmed_route_started_at >= 10.):
            self.fail_confirmed_approach('graph_route_unavailable')
        if (not self.object_found and self.live_observation_deadline is not None
                and time.monotonic() >= self.live_observation_deadline):
            # Include blocked/no-route states in the bound. A failed viewpoint
            # must not hold the instance forever without reaching a waypoint.
            if not self.fail_confirmed_approach('observation_route_timeout'):
                self.reset(preserve_execution_corridor=True)
                self.live_observation_deadline = None
                self.observation_wait_until = time.monotonic()
        if (self.object_found or self.memory_selecting
                or self.pending or self.following or self.follow_quarantine
                or self.observation_wait_until is None
                or time.monotonic() < self.observation_wait_until
                or self.input_freshness_issue() is not None or self.query_reset_pending):
            return
        if self.memory_active is None:
            if self.start_live_observation():
                return
            # Arrival at a semantic waypoint proves neither the class nor a
            # successful search. Release its explicit graph goal if fresh
            # observations did not confirm it, and resume frontier selection.
            failed = self.expected
            abandoned = self.failed_confirmed_goals + ([failed] if failed is not None else [])
            if abandoned:
                self.unconfirmed_approaches.extend((p, time.monotonic()+60.) for p in abandoned)
                self.unconfirmed_approaches = self.unconfirmed_approaches[-16:]
            self.reset(preserve_execution_corridor=True)
            self.request_graph_reset()
            self.request_instance_release()
            self.init_confirmed_approach()
            self.init_visible_approach()
            self.nearest_signal_position = None
            self.nearest_signal_distance = None
            self.nearest_signal_standoff = None
            self.active_object_confirmation = None
            self.expected = None
            self.pending_destination = None
            self.observation_active = False
            self.live_observation_target = None
            self.live_observation_deadline = None
            self.observation_wait_until = None
            self.status('unconfirmed_target_release_pending',
                        goal=list(failed) if failed else None,
                        requires_visual_confirmation=True)
            self.retry_graph_reset()
            return
        from .graph_path_bridge import xy, yaw
        current = xy(self.odom.pose.pose)
        options = observation_approaches(self.memory_active, current)
        for x, y, heading in options:
            key = (round(x, 1), round(y, 1))
            if key in self.observation_attempts:
                continue
            self.observation_attempts.add(key)
            if self.pose_blockers(self.grid, (x, y), heading):
                continue
            self.reset()
            self.observation_active = True
            self.observation_wait_until = None
            self.expected = (x, y)
            self.memory_arrival_heading = heading
            pose = self.make_memory_pose(self.memory_active, heading)
            pose.pose.position.x, pose.pose.position.y = x, y
            self.relay_destination(pose)
            self.status('memory_observation_approach', goal=[x, y],
                        object_position=self.memory_active['object_position'],
                        approaches_tried=len(self.observation_attempts))
            return
        self.observation_wait_until = None
        if self.memory_active.get('evidence') == 'passive_semantic_landmarks':
            # An unconfirmed semantic match may be a false positive or no
            # longer visible. Try another historical region, then let normal
            # exploration continue; never hold an unverified goal forever.
            if self.reject_selected_memory('semantic_hint_unconfirmed'):
                return
        self.status('memory_observation_exhausted', goal=list(self.expected),
                    reason='no_additional_verified_observation_approach',
                    approaches_tried=len(self.observation_attempts),
                    requires_visual_confirmation=True)

    def observation_headings(self, endpoint):
        object_position = (self.memory_active.get('object_position') if self.memory_active
                           else self.live_observation_target)
        if not self.observation_active or not object_position:
            return None
        heading = math.atan2(object_position[1]-endpoint[1], object_position[0]-endpoint[0])
        # The front camera must face the target. The ideal point carrier can
        # rotate in place; the Ackermann profile checks heading through its path.
        fov = min(math.pi, max(.2, (self.memory_active or {}).get('horizontal_fov', 1.4)))
        return [heading, heading-fov/4, heading+fov/4]
