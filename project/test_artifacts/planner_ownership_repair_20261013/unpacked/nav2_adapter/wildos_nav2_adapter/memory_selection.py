"""Evaluate observed viewpoints through GraphNav and collision-checked CarLike paths."""
import json
import math
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .graph_route import distance
from .memory_candidates import decode_candidates, heading_options, path_cost
from .target_observation import observation_approaches


def position(pose):
    return (pose.position.x, pose.position.y)


def orientation(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class MemorySelection:
    def init_memory_selection(self):
        # Query and candidate snapshots use separate ROS topics, so a
        # candidate can arrive before its query callback. Keep a small
        # transaction-keyed inbox rather than one global slot: delayed
        # candidates for another label must not overwrite the active switch.
        self.memory_pending_payloads = {}
        self.clear_memory_selection()
        self.memory_feedback_pub = self.create_publisher(String, '/spot1/remembered_object_feedback', 10)
        self.memory_selection_pub = self.create_publisher(String, '/wildos/memory_selection', 10)
        self.create_subscription(String, '/spot1/remembered_object_candidates', self.on_memory_candidates, 10)
        self.create_timer(.5, self.monitor_memory_selection)

    def clear_memory_selection(self):
        self.init_confirmed_approach()
        self.active_object_confirmation = None
        self.memory_views = []
        self.memory_source_views = []
        self.memory_rejected = set()
        self.memory_matches = []
        self.memory_deferred = []
        self.memory_active = None
        self.memory_selecting = False
        self.memory_cursor = 0
        self.memory_deadline = 0.0
        self.memory_options = []
        self.memory_full_check = False
        self.memory_arrival_heading = None

    def candidate_transaction_matches(self, issued, query_started, now_ns):
        """Check candidate timestamps against the active query transaction."""
        if issued is not None:
            if (issued > now_ns or now_ns - issued > 5 * 10**9
                    or (self.query_started_ns > 0
                        and issued < self.query_started_ns - 200_000_000)):
                return False
        if query_started is not None:
            # The producer and bridge receive the String query independently.
            # Their ROS-clock timestamps may differ slightly, but a snapshot
            # from an earlier A -> B -> A cycle must not be treated as current.
            if (query_started <= 0 or self.query_started_ns <= 0
                    or abs(query_started - self.query_started_ns) > 1_000_000_000
                    or (issued is not None and query_started > issued + 100_000_000)):
                return False
        return True

    def on_memory_candidates(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                return
            issued = payload.get('issued_at_ns')
            if issued is not None:
                issued = int(issued)
                now = self.get_clock().now().nanoseconds
                # Validate basic age before buffering an out-of-order topic
                # delivery. Query-relative checks run only after that query
                # becomes active, so valid candidates are not compared with
                # the preceding query's timestamp.
                if issued > now or now - issued > 5 * 10**9:
                    return
            query_started = payload.get('query_started_ns')
            if query_started is not None:
                query_started = int(query_started)
            query, views = decode_candidates(message.data)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.get_logger().warning('Ignoring invalid memory candidates')
            return
        if query != self.query:
            # Cross-topic delivery can put candidates ahead of their query.
            received_at = time.monotonic()
            previous = self.memory_pending_payloads.get(query)
            if (previous is None or issued is None or previous[2] is None
                    or issued >= previous[2]):
                self.memory_pending_payloads[query] = (
                    query, views, issued, query_started, received_at)
            # Queries are short-lived transactions. Bound memory use and
            # discard snapshots that cannot safely be activated anymore.
            for old_query, payload in list(self.memory_pending_payloads.items()):
                if received_at - payload[4] > 2.0:
                    self.memory_pending_payloads.pop(old_query, None)
            while len(self.memory_pending_payloads) > 8:
                oldest = min(self.memory_pending_payloads,
                             key=lambda key: self.memory_pending_payloads[key][4])
                self.memory_pending_payloads.pop(oldest, None)
            return
        if not self.candidate_transaction_matches(
                issued, query_started, self.get_clock().now().nanoseconds):
            return
        if self.object_found:
            return
        if views == self.memory_source_views and (views or self.memory_goal is None):
            return
        self.begin_memory_selection(views)

    def activate_pending_memory(self):
        now = time.monotonic()
        for query, payload in list(self.memory_pending_payloads.items()):
            if now - payload[4] > 2.0:
                self.memory_pending_payloads.pop(query, None)
        pending = self.memory_pending_payloads.pop(self.query, None)
        if pending is None:
            return
        query, views, issued, query_started, received_at = pending
        if (query == self.query and time.monotonic() - received_at < 2.0
                and self.candidate_transaction_matches(
                    issued, query_started, self.get_clock().now().nanoseconds)
                and not self.object_found):
            self.begin_memory_selection(views)

    def begin_memory_selection(self, views, rejected=None):
        if self.upstream_instance_id and not self.memory_selecting:
            self.query_reset_pending = True
            self.query_reset_inflight = False
            self.request_instance_release()
        self.reset()
        self.init_observation(preserve_instance=True)
        self.clear_memory_selection()
        self.memory_source_views = deepcopy(views)
        expanded = []
        for view in views:
            options = [view]
            if (view.get('evidence') == 'passive_semantic_landmarks'
                    and not view.get('approach_parent_id') and self.odom is not None and self.grid is not None):
                current = position(self.odom.pose.pose)
                for index, (x, y, heading) in enumerate(observation_approaches(view, current, limit=2)):
                    if self.pose_blockers(self.grid, (x, y), heading):
                        continue
                    alternative = dict(view, id=view['id']+'~approach'+str(index),
                        approach_parent_id=view['id'], position=[x, y, view['position'][2]], heading=heading)
                    options.append(alternative)
                options.sort(key=lambda v: distance(v['position'], current))
            expanded.extend(options)
        self.memory_views = list({v['id']:v for v in expanded}.values())[:8]
        self.memory_rejected = set(rejected or ())
        self.memory_selecting = bool(views)
        self.memory_goal = self.expected = self.memory_heading = None
        self.pending_destination = None
        if views:
            self.advance_memory_view()
        else:
            self.release_memory_goal('no_eligible_memory_view')
        self.retry_graph_reset()

    def make_memory_pose(self, view, heading=None):
        p = PoseStamped()
        p.header.frame_id = view['frame']
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.position.z = view['position']
        h = view['heading'] if heading is None else heading
        p.pose.orientation.z, p.pose.orientation.w = math.sin(h/2), math.cos(h/2)
        return p

    def advance_memory_view(self):
        if self.input_freshness_issue() is not None:
            self.status('memory_waiting_for_inputs')
            return
        if self.query_reset_pending:
            return
        while self.memory_cursor < len(self.memory_views):
            view = self.memory_views[self.memory_cursor]
            self.memory_cursor += 1
            if view['id'] in self.memory_rejected:
                continue
            self.reset()
            self.memory_active = view
            self.memory_goal = self.expected = tuple(view['position'][:2])
            self.memory_heading = view['heading']
            self.memory_options = []
            self.memory_full_check = False
            self.memory_deadline = time.monotonic() + 10.0
            start = position(self.odom.pose.pose)
            if distance(start, self.memory_goal) < 1.0:
                current_heading = orientation(self.odom.pose.pose)
                if not self.pose_blockers(self.grid, start, current_heading):
                    cost = .25 * abs(math.atan2(math.sin(current_heading - self.memory_heading),
                                                math.cos(current_heading - self.memory_heading)))
                    self.memory_matches.append(dict(view=view, cost=cost, heading=current_heading,
                                                    complete=True, path=None))
                    continue
            self.query_grace_until = 0.0
            self.relay_destination(self.make_memory_pose(view))
            self.status('checking_memory_candidate', view_id=view['id'],
                        goal=list(self.memory_goal), index=self.memory_cursor,
                        total=len(self.memory_views))
            return
        self.finish_memory_selection()

    def monitor_memory_selection(self):
        self.monitor_observation()
        if not self.memory_selecting:
            return
        if self.query_reset_pending or self.input_freshness_issue() is not None:
            self.memory_deadline = time.monotonic() + 10.0
            return
        if self.last_status.get('status') == 'start_footprint_blocked':
            # A blocked current vehicle pose says nothing about this object's
            # remembered viewpoint. Retain the destination instead of timing
            # it out and restarting frontier exploration for the same target.
            self.memory_deadline = time.monotonic() + 10.0
            start = position(self.odom.pose.pose)
            if not self.pose_blockers(self.grid, start, orientation(self.odom.pose.pose)):
                self.blocked_endpoint = None
                self.last_plan_start = self.last_plan_endpoint = None
                self.start_segment()
            return
        if self.memory_active is None:
            self.advance_memory_view()
        elif self.last_status.get('status') == 'planner_unavailable':
            self.memory_deadline = time.monotonic() + 10.0
        elif self.memory_deadline and time.monotonic() > self.memory_deadline:
            self.finish_memory_view('planner_timeout' if self.pending else 'graph_route_timeout')

    def memory_terminal_candidates(self, start, endpoint):
        preferred = self.memory_arrival_heading if self.memory_arrival_heading is not None else self.memory_heading
        headings = self.observation_headings(endpoint) or heading_options(
            start, endpoint, preferred, orientation(self.odom.pose.pose))
        return [(endpoint[0], endpoint[1], h, distance(start, endpoint)) for h in headings]

    def record_memory_path(self, path, points, headings):
        route = [position(self.odom.pose.pose)] + self.current_route_points()
        graph_length = sum(distance(a, b) for a, b in zip(route, route[1:]))
        segment_length = sum(distance(a, b) for a, b in zip(points, points[1:]))
        remaining = 0.0 if self.memory_full_check else max(0.0, graph_length - segment_length)
        self.memory_options.append(dict(view=self.memory_active,
            cost=path_cost(points, headings, self.grid, self.memory_heading, remaining),
            heading=headings[-1] if self.memory_full_check else self.memory_heading,
            complete=self.memory_full_check, path=deepcopy(path),
            route=deepcopy(self.route), route_complete=getattr(self, 'route_complete', True),
            target=tuple(points[-1])))
        self.pending = False
        if self.memory_full_check:
            self.candidate_index += 1
            self.try_candidate()  # Compare all feasible arrival orientations.
        else:
            self.finish_memory_view()  # Long routes are verified a segment at a time.

    def memory_candidates_exhausted(self):
        if self.memory_selecting:
            self.finish_memory_view('no_vehicle_path' if not self.memory_options else None)
            return True
        if self.memory_active is not None:
            if self.memory_active.get('object_position'):
                self.observation_attempts.add((round(self.expected[0], 1), round(self.expected[1], 1)))
                self.observation_wait_until = time.monotonic() + 1.
            self.block_current_route()
            self.status('memory_locally_blocked', reason='no_vehicle_path',
                        goal=list(self.memory_goal), reasons=list(self.failure_reasons))
            return True
        return False

    def publish_memory_feedback(self, view, reason):
        self.memory_feedback_pub.publish(String(data=json.dumps({
            'query': self.query, 'id': view.get('approach_parent_id', view['id']),
            'outcome': 'unconfirmed' if reason == 'semantic_hint_unconfirmed' else 'unavailable',
            'reason': reason, 'planner_rejections': list(self.failure_reasons)})))

    def finish_memory_view(self, reason=None):
        if self.memory_options:
            self.memory_matches.append(min(self.memory_options, key=lambda v: v['cost']))
        elif self.memory_active is not None:
            self.memory_deferred.append(dict(view=deepcopy(self.memory_active),
                route=deepcopy(self.route), route_complete=getattr(self, 'route_complete', True)))
            self.memory_rejected.add(self.memory_active['id'])
            self.publish_memory_feedback(self.memory_active, reason or 'no_vehicle_path')
        self.memory_active = None
        self.advance_memory_view()

    def release_memory_goal(self, reason):
        if self.upstream_instance_id:
            self.request_instance_release()
        self.reset()
        self.init_confirmed_approach()
        self.active_object_confirmation = None
        self.memory_selecting = False
        self.memory_active = None
        self.memory_goal = self.expected = self.memory_heading = None
        self.pending_destination = None
        self.query_reset_pending = True
        self.query_reset_inflight = False
        detail = {'status': 'memory_unreachable', 'reason': reason,
                  'rejected': sorted(self.memory_rejected), 'query': self.query}
        self.memory_selection_pub.publish(String(data=json.dumps(detail)))
        self.status('memory_unreachable', reason=reason)
        self.retry_graph_reset()  # Retain the mapped graph and release its old destination.

    def finish_memory_selection(self):
        if not self.memory_matches:
            if self.memory_deferred:
                blocked = self.memory_deferred[0]
                self.reset()
                self.memory_selecting = False
                self.memory_active = blocked['view']
                self.memory_goal = self.expected = tuple(self.memory_active['position'][:2])
                self.memory_heading = self.memory_active['heading']
                self.route = self.latest_route = blocked['route']
                self.route_complete = self.latest_route_complete = blocked['route_complete']
                self.route_endpoint = position(self.route.poses[-1].pose) if self.route else None
                self.relay_destination(self.make_memory_pose(self.memory_active))
                self.block_current_route()
                self.status('memory_locally_blocked', reason='candidates_temporarily_unavailable',
                            goal=list(self.memory_goal))
                if self.memory_active.get('object_position'):
                    self.observation_attempts.add((round(self.memory_goal[0], 1), round(self.memory_goal[1], 1)))
                    self.observation_wait_until = time.monotonic() + 1.
                return
            self.release_memory_goal('no_eligible_memory_view')
            return
        best = min(self.memory_matches, key=lambda v: v['cost'])
        self.reset()
        self.memory_selecting = False
        self.memory_active = best['view']
        self.memory_goal = self.expected = tuple(best['view']['position'][:2])
        self.memory_heading = best['view']['heading']
        self.memory_arrival_heading = best['heading']
        self.memory_selection_pub.publish(String(data=json.dumps({
            'status': 'memory_candidate_selected', 'query': self.query,
            'view_id': best['view']['id'], 'goal': list(self.memory_goal),
            'cost': best['cost'], 'complete_path_checked': best['complete'],
            'rejected': sorted(self.memory_rejected),
            'costs': [{'id': v['view']['id'], 'cost': v['cost'], 'complete': v['complete']}
                      for v in self.memory_matches]})))
        if distance(position(self.odom.pose.pose), self.memory_goal) < 1.0:
            self.query_reset_pending = True
            self.query_reset_inflight = False
            self.pending_destination = None
            self.retry_graph_reset()
            self.status('already_at_goal', goal=list(self.memory_goal), requires_visual_confirmation=True)
            self.await_object_observation()
        else:
            self.relay_destination(self.make_memory_pose(best['view'], best['heading']))
            self.status('memory_candidate_selected', view_id=best['view']['id'], cost=best['cost'])
            if best['path'] is not None:
                self.route = deepcopy(best['route'])
                self.latest_route = self.route
                self.route_complete = self.latest_route_complete = best.get('route_complete', True)
                self.route_endpoint = position(self.route.poses[-1].pose)
                self.execute_checked_path(deepcopy(best['path']), best['target'])

    def reject_selected_memory(self, reason):
        if self.memory_selecting or self.memory_active is None or not self.memory_views:
            return False
        rejected = self.memory_rejected | {self.memory_active['id']}
        if reason == 'semantic_hint_unconfirmed':
            parent = self.memory_active.get('approach_parent_id', self.memory_active['id'])
            rejected |= {v['id'] for v in self.memory_views
                         if v.get('approach_parent_id', v['id']) == parent}
        self.publish_memory_feedback(self.memory_active, reason)
        self.begin_memory_selection(self.memory_source_views, rejected)
        return True

    def retry_memory_after_execution_failure(self):
        """Try another remembered viewpoint after execution retries are exhausted.

        Keep the semantic memory intact: a failed drive attempt only rejects
        this query's selected viewpoint. Candidate planning is rerun from the
        current pose, and frontier exploration remains the fallback if no
        remembered viewpoint is usable.
        """
        if (self.memory_selecting or self.memory_active is None
                or not self.memory_source_views):
            return False
        rejected = set(self.memory_rejected)
        rejected.add(self.memory_active['id'])
        failed_view = self.memory_active['id']
        self.get_logger().warning(
            'Remembered viewpoint execution retries exhausted; checking alternatives: '
            + failed_view)
        self.begin_memory_selection(self.memory_source_views, rejected)
        return True
