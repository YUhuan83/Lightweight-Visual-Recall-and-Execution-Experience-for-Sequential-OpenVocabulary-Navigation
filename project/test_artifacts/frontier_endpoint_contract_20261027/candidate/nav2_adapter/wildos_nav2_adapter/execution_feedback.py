"""Feedback belongs to an accepted Nav2 action with observed actuation."""
from copy import deepcopy
import json
import math
import threading
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from graphnav_msgs.msg import NavigationGraph, TraversalFeedback, KeyValue
from std_msgs.msg import String
from .graph_route import distance, remaining_route
from .input_quality import stamp_ns
from .source_control import receipt_issue, PROTOCOL


def path_progress(path, position):
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    total = sum(distance(a, b) for a, b in zip(points, points[1:]))
    remaining = remaining_route(position, points)
    return max(0., total - sum(distance(a, b) for a, b in zip(remaining, remaining[1:])))


def native_command_braking(value):
    """An acknowledged brake command, not an unsent fail-closed intent."""
    command = value.get('sent') or {}
    brake = command.get('brake') if isinstance(command, dict) else None
    return (value.get('actuator_ok') is True and value.get('control_acknowledged') is True
            and isinstance(brake, (int, float)) and not isinstance(brake, bool)
            and math.isfinite(brake) and brake >= .9)


class ExecutionFeedback:
    def init_execution_feedback(self):
        self.execution_attempt = None
        self.feedback_graph = None
        self.control_status = None
        self.control_status_at = None
        self.control_status_inactive_at = None
        self.native_status = None
        self.native_status_at = None
        self.execution_status_lock = threading.Lock()
        # Vehicle freshness is a control safety input. Give it its own callback
        # group so route scoring or graph callbacks cannot age it out.
        self.native_status_callback_group = MutuallyExclusiveCallbackGroup()
        self.stall_anchor = None
        self.declare_parameter('stall_timeout_sec', 5.0)
        self.declare_parameter('stall_progress_m', .2)
        self.declare_parameter('stall_progress_rad', .2)
        if (not math.isfinite(float(self.get_parameter('stall_timeout_sec').value))
                or float(self.get_parameter('stall_timeout_sec').value) <= 0
                or not math.isfinite(float(self.get_parameter('stall_progress_m').value))
                or float(self.get_parameter('stall_progress_m').value) <= 0
                or not math.isfinite(float(self.get_parameter('stall_progress_rad').value))
                or not 0 < float(self.get_parameter('stall_progress_rad').value) < math.pi):
            raise ValueError('stall thresholds must be finite and positive')
        self.execution_feedback_pub = self.create_publisher(TraversalFeedback, '/spot1/traversal_feedback', 10)
        self.execution_event_pub = self.create_publisher(String, '/wildos/execution_feedback', 10)
        self.declare_parameter('execution_anchor_graph_topic', '/spot1/execution_anchor_graph')
        self.create_subscription(NavigationGraph, self.get_parameter('execution_anchor_graph_topic').value,
                                 self.on_feedback_graph, 1)
        self.create_subscription(String, '/wildos/control_status', self.on_control_status, 10,
                                 callback_group=self.native_status_callback_group)
        self.create_subscription(String, '/wildos/native_control_status', self.on_native_status, 10,
                                 callback_group=self.native_status_callback_group)

    def native_status_snapshot(self):
        with self.execution_status_lock:
            return self.native_status, self.native_status_at

    def control_status_snapshot(self):
        with self.execution_status_lock:
            return self.control_status, self.control_status_at

    def on_native_status(self, message):
        try:
            value = json.loads(message.data)
            if (not isinstance(value, dict) or not isinstance(value.get('actual', {}), dict)
                    or not isinstance(value.get('wheel_contact') or {}, dict)
                    or not isinstance(value.get('sent') or {}, dict)):
                return
            wall_time = float(value['wall_time'])
            age = time.time() - wall_time
            speed = float(value['speed'])
            if (not 0 <= age <= 1. or isinstance(value['speed'], bool)
                    or not math.isfinite(speed)):
                return
            contact = value.get('wheel_contact') or {}
            if (not isinstance(contact.get('diagnosis') or {}, dict)
                    or not isinstance(contact.get('wall_time', 0.), (int, float))):
                return
            value['wall_time'] = wall_time
            value['speed'] = speed
            with self.execution_status_lock:
                if (self.native_status is not None
                        and wall_time <= self.native_status.get('wall_time', 0.)):
                    return
                self.native_status, self.native_status_at = value, time.monotonic()-age
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def execution_transport_issue(self, *, for_stop=False):
        """Physical execution requires fresh, acknowledged actuator evidence."""
        if self.carrier_model != 'ackermann' or (not self.follow_enabled and not for_stop):
            return None  # Read-only planning and the ideal carrier retain their own contracts.
        value, received_at = self.native_status_snapshot()
        if value is None or received_at is None:
            return 'native_status_missing'
        if value.get('actuator_ok') is not True:
            return 'native_actuator_failed'
        age = time.monotonic()-received_at
        sample_age = value.get('telemetry_age_sec')
        if (not math.isfinite(age) or age < 0 or age > .5
                or isinstance(sample_age, bool) or not isinstance(sample_age, (int,float))
                or not math.isfinite(sample_age) or sample_age < 0
                or sample_age+age > .5 or value.get('telemetry_ok') is not True):
            return 'native_telemetry_stale'
        if value.get('control_acknowledged') is not True:
            return 'native_control_unacknowledged'
        # Stop quarantine verifies acknowledged braking and measured sustained
        # standstill in its caller. A dead producer must not prevent that
        # evidence from closing an old action; new motion still needs a source.
        if self.require_source_control and not for_stop:
            receipt = value.get('source_receipt')
            if value.get('control_protocol') != PROTOCOL:
                return 'source_protocol_missing'
            issue = receipt_issue(receipt, time.time_ns())
            if issue:
                return issue
            # Bound the receipt with a steady clock too. Wall rollback must
            # not extend the original source lifetime after status reception.
            remaining = receipt['expires_wall_ns'] * 1e-9 - value['wall_time']
            if age >= remaining:
                return 'source_receipt_expired'
            source, source_at = self.control_status_snapshot()
            source = source or {}
            owner = source.get('source_control') or {}
            owner_matches = (isinstance(owner, dict)
                and owner.get('sender_id') == receipt['sender_id']
                and owner.get('generation') == receipt['generation'])
            # control_status is sampled before the native ACK is consumed on
            # the next controller tick. If both topics name the exact same
            # applied command, the Native receipt is the stronger ACK; tolerate
            # only this one-sequence status skew, never an owner/sequence change.
            receipt_confirms_owner = (owner_matches
                and type(owner.get('sequence')) is int
                and owner['sequence'] == receipt.get('sequence')
                and receipt.get('applied') is True)
            if (source_at is None or time.monotonic()-source_at > .5
                    or source.get('control_protocol') != PROTOCOL
                    or not owner_matches
                    or (source.get('source_ack_ready') is not True
                        and not receipt_confirms_owner)):
                return 'source_owner_unconfirmed'
        return None

    def execution_transport_unavailable(self, reason):
        """Invalidate action ownership without blaming or releasing a route."""
        self.retired_motion = None
        self.retire_follow_transactions()
        self.finish_execution_feedback(False, reason)
        self.epoch += 1
        self.cancel_goal(self.plan_handle, 'execution_transport_compute_path')
        self.cancel_goal(self.follow_handle, 'execution_transport_follow_path')
        self.plan_handle = self.follow_handle = None
        self.plan_request_started_at = None
        self.pending = self.following = False
        self.candidate_waiting_for_map = False
        self.motion_queue = []
        self.prepared_extension = None
        self.extending_path = self.replacement_pending = False
        self.deferred_follow_result = None
        self.recovering = False
        self.escape_path = None
        self.stall_anchor = None
        self.last_plan_start = self.last_plan_endpoint = None
        if (self.last_status.get('status') != 'execution_unavailable'
                or self.last_status.get('reason') != reason):
            self.status('execution_unavailable', reason=reason)

    def execution_stall(self):
        now = time.monotonic()
        control, control_at = self.control_status_snapshot()
        with self.execution_status_lock:
            inactive_at = self.control_status_inactive_at
        if (self.stall_anchor is not None and inactive_at is not None
                and inactive_at > self.stall_anchor[2]):
            self.stall_anchor = None
        if (not self.following or self.odom is None or control_at is None
                or now - control_at > 1. or not control.get('actuating')):
            self.stall_anchor = None
            return None
        if not self.motion_queue or not self.motion_queue[0].poses:
            self.stall_anchor = None
            return None
        native, native_at = self.native_status_snapshot()
        if (self.carrier_model == 'ackermann' and native is not None
                and native_at is not None and now - native_at <= 1.
                and native_command_braking(native)):
            # A fresh SDK receipt of full brakes ends the physical actuation
            # window even if upstream intent still says "tracking". Waiting
            # is not evidence that terrain or a learned graph edge failed.
            self.stall_anchor = None
            return None
        p = self.odom.pose.pose.position
        path = self.motion_queue[0]
        pure_rotation = (self.carrier_model == 'ideal_point'
            and abs(float(control.get('target_speed', 0.))) <= .01
            and abs(float(control.get('target_angular', 0.))) > .01)
        if pure_rotation:
            from .graph_path_bridge import yaw
            # RPP can align to the lookahead at the start of a curved path,
            # or to the final orientation on arrival. The bridge does not
            # know that internal target. Measure actual yaw in the commanded
            # direction; this is actuation evidence, never arrival evidence.
            progress = yaw(self.odom.pose.pose)
            direction = math.copysign(1., float(control['target_angular']))
            threshold = float(self.get_parameter('stall_progress_rad').value)
        else:
            progress = path_progress(path, (p.x, p.y))
            direction = 0.
            threshold = float(self.get_parameter('stall_progress_m').value)
        path_key = (id(path), stamp_ns(path), len(path.poses), pure_rotation, direction)
        if self.stall_anchor is None:
            self.stall_anchor = (path_key, progress, now)
            return None
        anchor_key, anchor_progress, anchor_time = self.stall_anchor
        if anchor_key != path_key:
            # Meters and radians cannot share a baseline or elapsed window.
            # Same-mode replans/direction reversals keep the existing timeout:
            # a fresh command/path alone is not physical progress.
            if anchor_key[3] != pure_rotation:
                self.stall_anchor = (path_key, progress, now)
                return None
            self.stall_anchor = (path_key, progress, anchor_time)
            anchor_progress = progress
        delta = progress - anchor_progress
        if pure_rotation:
            delta = math.atan2(math.sin(delta), math.cos(delta)) * direction
        if delta >= threshold:
            self.stall_anchor = (path_key, progress, now)
            return None
        duration = now - anchor_time
        if duration < float(self.get_parameter('stall_timeout_sec').value):
            return None
        classification = ('kinematic_pose_not_advancing'
                          if self.carrier_model == 'ideal_point'
                          else 'actuated_no_path_progress')
        native, native_at = self.native_status_snapshot()
        if native is not None and native_at is not None and now - native_at <= 1.:
            actual = native.get('actual') or {}
            wheelspeed = actual.get('wheelspeed')
            contact = native.get('wheel_contact') or {}
            diagnosis = contact.get('diagnosis') or {}
            telemetry_age = native.get('telemetry_age_sec')
            state_age = native.get('state_age_sec')
            if (native.get('telemetry_ok') is False
                    or (isinstance(telemetry_age, (int, float))
                        and math.isfinite(telemetry_age) and telemetry_age > .5)):
                classification = 'native_telemetry_stale'
            elif (native.get('state_ok') is False
                    or (isinstance(state_age, (int, float))
                        and math.isfinite(state_age) and state_age > .5)):
                classification = 'state_sensor_stale'
            elif native.get('watchdog_stop'):
                classification = 'actuator_watchdog'
            elif (0 <= time.time() - contact.get('wall_time', 0.) <= 1.5
                    and diagnosis.get('classification') in ('wheel_airborne_spin', 'wheel_support_lost')):
                classification = diagnosis['classification']
            elif (isinstance(wheelspeed, (int, float)) and math.isfinite(wheelspeed)
                    and abs(wheelspeed) > .3 and native['speed'] < .1):
                classification = 'wheel_slip_or_contact_loss'
        return dict(stall_class=classification, no_progress_sec=round(duration, 3),
                    progress_kind='heading_rad' if pure_rotation else 'path_m',
                    progress_delta=round(delta, 4), progress_threshold=threshold,
                    target_speed=control.get('target_speed'),
                    target_angular=control.get('target_angular'))

    def checkpoint_execution_feedback(self, path):
        """Keep evidence across a preemption; learn only a completed graph prefix."""
        attempt = self.execution_attempt
        if attempt is None:
            self.start_execution_feedback(path)
            return
        p = self.odom.pose.pose.position
        attempt['planned_distance_completed'] += max(0.,
            path_progress(attempt['path'], (p.x, p.y)) - attempt['path_start_progress'])
        graph = attempt['graph']
        if (graph is not None and graph.nodes and self.odom is not None
                and graph.header.frame_id == self.odom.header.frame_id):
            def near(p):
                valid = [n for n in graph.nodes if any(n.uuid.id) and
                    math.isfinite(n.pose.position.x) and math.isfinite(n.pose.position.y)]
                if not valid:
                    return None
                n = min(valid, key=lambda n: distance((p.x, p.y),
                    (n.pose.position.x, n.pose.position.y)))
                return n if distance((p.x, p.y), (n.pose.position.x, n.pose.position.y)) <= 1. else None
            a, b = near(attempt['start'].pose.pose.position), near(self.odom.pose.pose.position)
            if (a is not None and b is not None and list(a.uuid.id) != list(b.uuid.id)
                    and attempt['actuated'] and attempt['observed_distance'] >= .1
                    and attempt['generation'] == self.query_generation
                    and abs(attempt['start_ns'] - stamp_ns(graph)) <= 10 * 10**9
                    and self.input_freshness_issue() is None and not attempt['tracking_invalid']):
                attempt['path_start_progress'] = path_progress(attempt['path'], (p.x, p.y))
                self.finish_execution_feedback(True, 'traversed_prefix', partial=True)
                self.start_execution_feedback(path)
        # The still-running logical traversal keeps its original anchors and
        # start time, but subsequent motion must follow the newly accepted path.
        self.execution_attempt['path'] = deepcopy(path)
        self.execution_attempt['path_start_progress'] = path_progress(path, (p.x, p.y))

    def on_feedback_graph(self, graph):
        if graph.header.frame_id == 'spot1/odom' and graph.nodes:
            self.feedback_graph = graph

    def on_control_status(self, message):
        try:
            data = json.loads(message.data)
            # The wire timestamp is integer ROS nanoseconds; float conversion
            # would lose precision, and bool is an int subclass in Python.
            if (not isinstance(data, dict) or type(data.get('stamp_ns')) is not int):
                return
            age = (self.get_clock().now().nanoseconds - data['stamp_ns']) * 1e-9
            if not 0 <= age <= 1. or not isinstance(data['actuating'], bool):
                return
            for name in ('target_speed', 'target_angular'):
                value = data.get(name, 0.)  # Older physical status omits angular.
                if isinstance(value, bool):
                    return
                value = float(value)
                if not math.isfinite(value):
                    return
                data[name] = value
            with self.execution_status_lock:
                if (self.control_status is not None
                        and data['stamp_ns'] < self.control_status.get('stamp_ns', 0)):
                    return
                received_at = time.monotonic()
                if not data['actuating']:
                    self.control_status_inactive_at = received_at
                self.control_status, self.control_status_at = data, received_at
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def start_execution_feedback(self, path):
        # ROS subscriptions deliver a new message for each graph update. Keep
        # the received snapshot by reference: feedback reads its node anchors
        # only. Copying thousands of unused edges blocks this executor long
        # enough to miss odometry while a newly accepted action starts moving.
        self.execution_attempt = dict(start=deepcopy(self.odom), start_ns=self.get_clock().now().nanoseconds,
            path=deepcopy(path), query=self.query, generation=self.query_generation, actuated=False,
            graph=self.feedback_graph, observed_distance=0., tracking_invalid=False,
            planned_distance_completed=0., initial_path=deepcopy(path),
            path_start_progress=path_progress(path, (self.odom.pose.pose.position.x,
                                                   self.odom.pose.pose.position.y)))

    def observe_execution_motion(self, old, new):
        attempt = self.execution_attempt
        control, control_at = self.control_status_snapshot()
        if (attempt is None or old is None or control_at is None
                or time.monotonic() - control_at > 1. or not control.get('actuating')):
            return
        attempt['actuated'] = True
        a, b = old.pose.pose.position, new.pose.pose.position
        step = distance((a.x, a.y), (b.x, b.y))
        # Respawns/jumps cannot create traversal evidence.
        if step > 2.:
            self.finish_execution_feedback(False, 'odometry_jump')
            return
        attempt['observed_distance'] += step
        from .graph_route import distance_to_route
        points = [(p.pose.position.x, p.pose.position.y) for p in attempt['path'].poses]
        if distance_to_route((b.x, b.y), points) > self.tracking_error_limit:
            attempt['tracking_invalid'] = True

    def finish_execution_feedback(self, succeeded, reason, partial=False):
        attempt, self.execution_attempt = self.execution_attempt, None
        if attempt is None:
            return
        duration = (self.get_clock().now().nanoseconds - attempt['start_ns']) * 1e-9
        eligible = (succeeded and attempt['actuated'] and attempt['generation'] == self.query_generation
                    and duration > 0 and attempt['observed_distance'] >= .1
                    and self.input_freshness_issue() is None and not attempt['tracking_invalid']
                    and self.execution_transport_issue() is None)
        event = dict(
            query=attempt['query'], query_generation=attempt['generation'], reason=reason,
            succeeded=bool(succeeded), actuated=attempt['actuated'], duration=max(0., duration),
            observed_distance=attempt['observed_distance'], motion_eligible=bool(eligible))
        def report(learning_ready=False):
            self.execution_event_pub.publish(String(data=json.dumps({**event,
                'learning_eligible': bool(learning_ready)})))
        graph = attempt['graph']
        if not eligible or graph is None or not graph.nodes or self.odom is None:
            report()
            return
        if graph.header.frame_id != self.odom.header.frame_id or abs(
                attempt['start_ns'] - stamp_ns(graph)) > 10 * 10**9:
            report()
            return
        start, end = attempt['start'].pose.pose, self.odom.pose.pose
        def anchor(pose):
            p = pose.position
            valid = [n for n in graph.nodes if math.isfinite(n.pose.position.x)
                     and math.isfinite(n.pose.position.y) and any(n.uuid.id)]
            if not valid:
                return None
            n = min(valid, key=lambda n: distance((p.x, p.y), (n.pose.position.x, n.pose.position.y)))
            return n if distance((p.x, p.y), (n.pose.position.x, n.pose.position.y)) <= (1. if partial else 6.) else None
        a, b = anchor(start), anchor(end)
        if a is None or b is None or list(a.uuid.id) == list(b.uuid.id):
            report()
            return
        from .graph_path_bridge import xy, yaw
        msg = TraversalFeedback()
        msg.header = deepcopy(self.odom.header)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.from_node, msg.to_node = deepcopy(a.uuid), deepcopy(b.uuid)
        msg.traversability_class = 'default'
        msg.robot_profile_id = 'default'
        msg.start_pose, msg.end_pose = deepcopy(start), deepcopy(end)
        points = [xy(p.pose) for p in attempt['initial_path'].poses]
        direction = next(((b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])
                          if distance(a, b) > 1e-6), None)
        if direction is None:
            report()
            return
        # Travel bearing differs from vehicle orientation during reverse.
        msg.commanded_heading = math.atan2(direction[1], direction[0])
        msg.observed_heading = yaw(end)
        msg.commanded_distance = float(attempt['planned_distance_completed'] + max(0.,
            path_progress(attempt['path'], (end.position.x, end.position.y))
            - attempt['path_start_progress']))
        msg.observed_distance, msg.duration = float(attempt['observed_distance']), float(duration)
        msg.outcome, msg.retryable = TraversalFeedback.OUTCOME_SUCCEEDED, False
        # Record correlation without pretending an arbitrary local segment is
        # exactly a single graph edge. Consumers must keep this evidence scoped.
        msg.observations = [KeyValue(key='execution_source', value=[1.]),
                            KeyValue(key='query_generation', value=[float(attempt['generation'])]),
                            KeyValue(key='completed_prefix', value=[float(partial)])]
        self.execution_feedback_pub.publish(msg)
        report(True)
