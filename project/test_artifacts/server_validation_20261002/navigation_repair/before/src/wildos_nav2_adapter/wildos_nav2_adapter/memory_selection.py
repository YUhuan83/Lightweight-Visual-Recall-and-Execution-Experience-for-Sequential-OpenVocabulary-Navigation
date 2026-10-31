"""Evaluate observed viewpoints through GraphNav and collision-checked CarLike paths."""
import json
import math
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .graph_route import distance, footprint_blockers
from .memory_candidates import decode_candidates, heading_options, path_cost


def position(pose):
    return (pose.position.x, pose.position.y)


def orientation(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class MemorySelection:
    def init_memory_selection(self):
        self.memory_pending_payload = None
        self.clear_memory_selection()
        self.memory_feedback_pub = self.create_publisher(String, '/spot1/remembered_object_feedback', 10)
        self.memory_selection_pub = self.create_publisher(String, '/wildos/memory_selection', 10)
        self.create_subscription(String, '/spot1/remembered_object_candidates', self.on_memory_candidates, 10)
        self.create_timer(.5, self.monitor_memory_selection)

    def clear_memory_selection(self):
        self.memory_views = []
        self.memory_rejected = set()
        self.memory_matches = []
        self.memory_active = None
        self.memory_selecting = False
        self.memory_cursor = 0
        self.memory_deadline = 0.0
        self.memory_options = []
        self.memory_full_check = False
        self.memory_arrival_heading = None

    def on_memory_candidates(self, message):
        try:
            query, views = decode_candidates(message.data)
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Ignoring invalid memory candidates')
            return
        if query != self.query:
            # Cross-topic delivery can put candidates ahead of their query.
            self.memory_pending_payload = (query, views, time.monotonic())
            return
        if views == self.memory_views and (views or self.memory_goal is None):
            return
        self.begin_memory_selection(views)

    def activate_pending_memory(self):
        pending = self.memory_pending_payload
        self.memory_pending_payload = None
        if pending and pending[0] == self.query and time.monotonic() - pending[2] < 2.0:
            self.begin_memory_selection(pending[1])

    def begin_memory_selection(self, views, rejected=None):
        self.reset()
        self.clear_memory_selection()
        self.memory_views = views
        self.memory_rejected = set(rejected or ())
        self.memory_selecting = bool(views)
        self.memory_goal = self.expected = self.memory_heading = None
        self.pending_destination = None
        if views:
            self.advance_memory_view()
        else:
            self.release_memory_goal('no_eligible_memory_view')

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
                if not footprint_blockers(self.grid, start, current_heading):
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
            if not footprint_blockers(self.grid, start, orientation(self.odom.pose.pose)):
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
        return [(endpoint[0], endpoint[1], h, distance(start, endpoint)) for h in
                heading_options(start, endpoint, preferred, orientation(self.odom.pose.pose))]

    def record_memory_path(self, path, points, headings):
        route = [position(self.odom.pose.pose)] + self.current_route_points()
        graph_length = sum(distance(a, b) for a, b in zip(route, route[1:]))
        segment_length = sum(distance(a, b) for a, b in zip(points, points[1:]))
        remaining = 0.0 if self.memory_full_check else max(0.0, graph_length - segment_length)
        self.memory_options.append(dict(view=self.memory_active,
            cost=path_cost(points, headings, self.grid, self.memory_heading, remaining),
            heading=headings[-1] if self.memory_full_check else self.memory_heading,
            complete=self.memory_full_check, path=deepcopy(path)))
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
        return self.reject_selected_memory('no_vehicle_path')

    def publish_memory_feedback(self, view, reason):
        self.memory_feedback_pub.publish(String(data=json.dumps({
            'query': self.query, 'id': view['id'],
            'outcome': 'unreachable' if reason in ('no_vehicle_path', 'follow_failed') else 'unavailable',
            'reason': reason, 'planner_rejections': list(self.failure_reasons)})))

    def finish_memory_view(self, reason=None):
        if self.memory_options:
            self.memory_matches.append(min(self.memory_options, key=lambda v: v['cost']))
        elif self.memory_active is not None:
            self.memory_rejected.add(self.memory_active['id'])
            self.publish_memory_feedback(self.memory_active, reason or 'no_vehicle_path')
        self.memory_active = None
        self.advance_memory_view()

    def release_memory_goal(self, reason):
        self.reset()
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
            self.release_memory_goal('no_reachable_memory_candidate')
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
        else:
            self.relay_destination(self.make_memory_pose(best['view'], best['heading']))
            self.status('memory_candidate_selected', view_id=best['view']['id'], cost=best['cost'])

    def reject_selected_memory(self, reason):
        if self.memory_selecting or self.memory_active is None or not self.memory_views:
            return False
        rejected = self.memory_rejected | {self.memory_active['id']}
        self.publish_memory_feedback(self.memory_active, reason)
        self.begin_memory_selection(self.memory_views, rejected)
        return True
