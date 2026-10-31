"""A query-scoped exploration session with rolling, checked vehicle paths.

Intent changes replace planning, not acknowledged motion. Actual FollowPath
admission, source ownership, collision and reversing-cusp checks stay shared.
"""
from copy import deepcopy
import math
import time
import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from .graph_path_bridge import GraphPathBridge, xy, yaw
from .continuous_path import trim_moving_path, CONTINUOUS_HANDOFF_DISTANCE_M
from .graph_route import distance, forward_arc_arrival_heading
from .input_quality import stamp_ns,valid_pose
from .motion_path import motion_sections


class ContinuousGraphPathBridge(GraphPathBridge):
    def __init__(self):
        self._desired_intent = self._planned_intent = None
        self._motion_route = None
        self._motion_intent = self._motion_serial = self._motion_epoch = None
        self._closing_motion_serial = None
        self._rolling_plan_start = None
        self._controller_path=None
        self._controller_path_received_at=0.
        self._retirement_path_stamp=None
        super().__init__()
        self.retirement_pub=self.create_publisher(Path,'/wildos/controller_path_retirement',1)
        if self.full_path_execution:
            self.create_subscription(Path,'/wildos/controller_path_progress',self.on_controller_progress,1)

    def declare_strategy_parameters(self):
        self.declare_parameter('planner_minimum_turning_radius', self.minimum_turning_radius)
        self.declare_parameter('continuous_path_execution', False)
        self.full_path_execution=bool(self.get_parameter('continuous_path_execution').value)

    def status(self, state, **details):
        details['execution_mode'] = ('continuous_full_path_v2' if self.full_path_execution
                                     else 'continuous_exploration_v1')
        return super().status(state, **details)

    def execution_paths(self,path,points):
        if self.full_path_execution and self.carrier_model=='ackermann':
            return [deepcopy(path)]
        return super().execution_paths(path,points)

    def endpoint_may_complete_immediately(self,points):
        return not self.full_path_execution or len(motion_sections(points))<=1

    def motion_join_geometry(self,points,headings,position_tolerance):
        if self.full_path_execution and self.carrier_model=='ackermann':
            sections=motion_sections(points)
            if len(sections)>1:
                first,last=sections[0]
                return points[first:last+1],headings[first:last+1],.2
        return super().motion_join_geometry(points,headings,position_tolerance)

    def tracking_issue(self,points,headings,section_count):
        if self.full_path_execution and self.carrier_model=='ackermann':
            sections=motion_sections(points)
            if len(sections)>1:
                first,last=sections[0]
                return super().tracking_issue(points[first:last+1],headings[first:last+1],len(sections))
        return super().tracking_issue(points,headings,section_count)

    def reset(self, preserve_execution_corridor=False):
        self._retirement_path_stamp=None
        self._controller_path=None
        self._controller_path_received_at=0.
        self._desired_intent = self._planned_intent = None
        self._motion_route = self._rolling_plan_start = None
        self._motion_intent = self._motion_serial = self._motion_epoch = None
        return super().reset(preserve_execution_corridor=preserve_execution_corridor)

    def on_controller_progress(self,message):
        active=self.follow_active
        if (not self.full_path_execution or active is None or not self.following or
                self.active_follow_serial!=active['serial'] or
                active.get('path_stamp_ns')!=stamp_ns(message) or len(message.poses)<1 or
                self.odom is None or message.header.frame_id!=self.odom.header.frame_id or
                any(not valid_pose(p.pose) for p in message.poses)):
            return
        age=(self.get_clock().now().nanoseconds-stamp_ns(message.poses[0]))*1e-9
        if not 0<=age<=.5:return
        self._controller_path=deepcopy(message)
        self._controller_path_received_at=time.monotonic()

    def current_motion_path(self):
        if not self.full_path_execution:return super().current_motion_path()
        path=self._controller_path;active=self.follow_active
        if (path is None or active is None or
                active.get('path_stamp_ns')!=stamp_ns(path) or
                time.monotonic()-self._controller_path_received_at>.5):
            return None
        return path

    def intent_key(self, route):
        if route is None or not route.poses:
            return None
        metadata = self.exploration_metadata(route)
        if metadata is not None and metadata.get('exploration_id'):
            destination = ('frontier', metadata['exploration_id'])
        else:
            goal = self.expected or self.memory_goal or xy(route.poses[-1].pose)
            destination = ('recall' if self.memory_goal is not None else
                           'object' if self.expected is not None else 'route', tuple(goal))
        return self.query_generation, route.header.frame_id, destination

    def active_window(self):
        if (self.carrier_model != 'ackermann' or not self.follow_enabled or not self.following
                or self.follow_active is None or self.follow_handle is None
                or self.active_follow_serial != self.follow_active['serial']
                or self.follow_request is not None or self.replacement_pending
                or self.follow_quarantine or self.follow_release is not None or self.follow_hold
                or self.recovering or self.execution_recovery_owns_motion()
                or self.object_found or not self.motion_queue
                or self.input_freshness_issue() is not None
                or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None):
            return None
        current = xy(self.odom.pose.pose)
        heading = yaw(self.odom.pose.pose)
        accepted=self.current_motion_path()
        if accepted is None:return None
        path = trim_moving_path(accepted, current, heading)
        if path is None:
            return None
        points = [xy(p.pose) for p in path.poses]
        headings = [yaw(p.pose) for p in path.poses]
        if (path.header.frame_id != self.odom.header.frame_id
                or self.grid.header.frame_id != path.header.frame_id
                or self.local_grid.header.frame_id != path.header.frame_id):
            return None
        for grid in (self.grid, self.local_grid):
            if self.validate_local_path(points, headings, grid, current, heading):
                return None
        edge = next(((b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:])
                     if distance(a,b)>1e-9), None)
        if edge is None:
            return None
        forward = edge[0]*math.cos(headings[0])+edge[1]*math.sin(headings[0]) > 0.
        length = sum(distance(a,b) for a,b in zip(points,points[1:]))
        return path, forward, length

    def discard_intent_plan(self):
        # Invalidate asynchronous compute responses independently of the
        # acknowledged FollowPath epoch/serial. An unaccepted late response
        # is still canceled by the shared current_plan_request gate.
        self.cancel_goal(self.plan_handle, 'superseded_exploration_intent')
        self.plan_handle = self.active_plan_serial = self.plan_request_started_at = None
        self.pending = self.extending_path = self.candidate_waiting_for_map = False
        self.normal_best = self.prepared_extension = None
        self._planned_intent = self._rolling_plan_start = None
        self.clear_exploration_comparison()
        self.last_plan_start = self.last_plan_endpoint = None

    def restrict_current_direction(self):
        if not self.full_path_execution:return True
        active=self.follow_active
        if active is None:return False
        if self._retirement_path_stamp==active.get('path_stamp_ns'):return True
        path=self.current_motion_path()
        if path is None:return False
        # A whole FollowPath contains its future reversing branches. Queue
        # slicing cannot revoke them: constrain the actual controller as well.
        self.retirement_pub.publish(deepcopy(path))
        self._retirement_path_stamp=stamp_ns(path)
        self.status('continuous_direction_retired',path_stamp_ns=stamp_ns(path),
                    retained_action=self.active_follow_serial,
                    boundary=list(xy(path.poses[-1].pose)))
        return True

    def finish_current_exploration_section(self,identity,*,owner_kind='exploration'):
        if not super().finish_current_exploration_section(identity,owner_kind=owner_kind):
            return False
        # The shared admission just checked this same fresh direction path.
        if not self.restrict_current_direction():
            self.stop_retired_motion('retired_controller_progress_unavailable')
            return False
        return True

    def prepare_intent_change(self):
        if self.active_window() is None:
            return False
        if not self.restrict_current_direction():return False
        self.discard_intent_plan()
        # Only the acknowledged, finite current section may continue. An old
        # unexecuted reverse suffix is not permission for the new intention.
        self.motion_queue = self.motion_queue[:1]
        self.route = self.latest_route = None
        self.route_endpoint = None
        self.retired_motion = None
        self._desired_intent = None
        self.status('continuous_intent_handoff', retained_action=self.active_follow_serial)
        return True

    def can_continue_confirmed_approach(self, data, vehicle_position):
        # A new waypoint for the same target is a route update, not a stopped
        # vehicle task. Reuse the existing moving intent handoff and planner.
        if (not self.full_path_execution or self.pending or self.extending_path
                or self.prepared_extension is not None or self.route is None
                or self.query_reset_pending or self.memory_selecting
                or self.memory_active is not None or self.memory_goal is not None
                or self.confirmed_approach_failure is not None
                or self.confirmed_route_object is None or self.expected is None
                or data['query'] != self.query or not self.confirmed_instance_matches(data)):
            return False
        anchor=self.active_object_confirmation or {}
        if data.get('instance_id','') != anchor.get('instance_id',''):
            return False
        stamp=data.get('approach_update_stamp_ns') if data.get('approach_refresh') else data['observation_stamp_ns']
        if stamp is None or not 0<=self.get_clock().now().nanoseconds-stamp<=5_000_000_000:
            return False
        point=data['standoff']; current=vehicle_position; previous=self.expected
        if (distance(previous,self.confirmed_route_object)-distance(point,self.confirmed_route_object)<1.
                or distance(current,point)-distance(current,previous)<1.):
            return False
        window=self.active_window()
        return window is not None and window[1] and window[2]<=CONTINUOUS_HANDOFF_DISTANCE_M

    def retain_motion_for_retired_intent(self, data, metadata):
        if (data.get('reason') not in ('frontier_resolved', 'frontier_missing',
                'observation_completed', 'vehicle_cost_selection', 'explicit_destination',
                'graph_path_unavailable', 'vehicle_route_rejected', 'destination_backoff')
                or self.query_reset_pending or self.active_window() is None):
            return False
        identity = data['exploration_id']
        routes = (self.route, self.latest_route, self._motion_route)
        if not any((self.exploration_metadata(r) or {}).get('exploration_id') == identity
                   for r in routes if r is not None):
            return False
        if self.full_path_execution:
            # Motion belongs to the query-scoped session, rather than each
            # frontier. Reuse the existing bounded handoff while the next
            # intention plans; query changes and real faults still cancel.
            if not self.finish_current_exploration_section(identity):return False
            self.retired_motion['bounded']=True
            return True
        current = self.exploration_metadata(self.route) if self.route is not None else None
        if current and current.get('exploration_id') == identity:
            self.discard_intent_plan()
            self.route = None
            self.route_endpoint = None
        latest = self.exploration_metadata(self.latest_route) if self.latest_route is not None else None
        if latest and latest.get('exploration_id') == identity:
            self.latest_route = None
        self.motion_queue = self.motion_queue[:1]
        self.retired_motion = None
        self.status('continuous_frontier_retired', retired_exploration_id=identity,
                    retirement_reason=data['reason'], retained_action=self.active_follow_serial)
        return True

    def release_continuous_visible_intent(self, reason):
        if (getattr(self,'lookback_active',False) or reason not in (
                'visible_target_approach_timeout','visible_target_no_vehicle_path',
                'visible_target_route_timeout') or not self.prepare_intent_change()):
            return False
        failed = self.expected
        self.init_visible_approach()
        self.expected = None
        if failed is not None:
            self.visible_blocked = (failed,time.monotonic()+15.)
        self.pending_destination = None
        self.request_graph_reset()
        self.status(reason, goal=list(failed) if failed else None,
                    retained_current_section=True, requires_visual_confirmation=True)
        self.retry_graph_reset()
        return True

    def on_route(self, message):
        super().on_route(message)
        if self.latest_route is not message:
            return  # Shared frame/query/freshness/retired-owner admission rejected it.
        key = self.intent_key(message)
        if (self.following and self._motion_intent is not None and key!=self._motion_intent
                and not self.restrict_current_direction()):
            self._desired_intent=key
            self.follow_failed('retired_controller_progress_unavailable')
            return
        restart_idle_compute = False
        if self._desired_intent is not None and key != self._desired_intent:
            if self.follow_request is not None or self.replacement_pending:
                return  # Observe acceptance before deciding how to replace it.
            if self._planned_intent != key:
                # The shared callback may already have admitted and started
                # this new intent. Never cancel that request as though it
                # still belonged to the previous intention.
                restart_idle_compute = self.pending and not self.following
                self.discard_intent_plan()
                if self.following:
                    self.motion_queue = self.motion_queue[:1]
        self._desired_intent = key
        if self.following:
            self.maybe_extend_path()
        elif restart_idle_compute:
            # Saving latest_route is not permission to execute. Once an old
            # compute is canceled, re-enter the shared backoff, retry-budget
            # and route admission checks before starting its replacement.
            super().on_route(message)

    def extension_candidates(self, candidates, old_target):
        key = self.intent_key(self.route)
        if (self._motion_intent is not None and key is not None
                and self._motion_intent != key and self._motion_intent[0] == key[0]
                and self._motion_epoch == self.epoch
                and self._motion_serial is not None
                and self._motion_serial == self.active_follow_serial):
            # A new intention may end close to (or before) the old endpoint.
            # The old 4m progress filter must not discard that exact goal and
            # substitute a shorter graph prefix. This only admits planning;
            # actual-pose, body/map, direction and action-ACK gates still own
            # whether a result can replace the currently accepted section.
            return candidates
        return super().extension_candidates(candidates, old_target)

    def start_segment(self, continuous=False):
        if not self.prepare_visible_vehicle_goal():return
        self._planned_intent = self.intent_key(self.route)
        self._rolling_plan_start = xy(self.odom.pose.pose) if self.odom is not None else None
        if self._desired_intent is None:
            self._desired_intent = self._planned_intent
        return super().start_segment(continuous=continuous)

    def visible_turning_radius(self):
        return max(self.minimum_turning_radius,float(
            self.get_parameter('planner_minimum_turning_radius').value))

    def begin_exploration_comparison(self):
        # Keep WildOS graph/recall scoring. Remove the extra stopped-vehicle
        # multi-frontier ComputePath tournament from this execution strategy.
        # A normal position-only goal may use its feasible circular arrival
        # yaw; explicit observation and remembered orientations stay intact.
        if (self.carrier_model == 'ackermann' and self.odom is not None
                and self.memory_goal is None and not self.memory_selecting
                and self.confirmed_route_goal is None and not getattr(self,'lookback_active',False)
                and self.candidates):
            target = self.candidates[0][:2]
            required = self.exploration_observation_heading()
            destination = self.exploration_destination() or self.route_endpoint
            explicit = required is not None and destination is not None and distance(target,destination)<1e-6
            heading = forward_arc_arrival_heading(xy(self.odom.pose.pose),yaw(self.odom.pose.pose),
                target,max(self.minimum_turning_radius,
                    float(self.get_parameter('planner_minimum_turning_radius').value)))
            if not explicit and heading is not None and self.vehicle_pose_feasible(target,heading):
                preferred = (*target,heading,self.candidates[0][3])
                rest = [c for c in self.candidates if not (distance(c[:2],target)<1e-6
                        and abs(math.remainder(c[2]-heading,2*math.pi))<1e-6)]
                self.candidates = [preferred]+rest
        return False

    def should_compare_endpoint_headings(self, path):
        if self.memory_goal is not None or self.memory_selecting:
            return True
        points = [xy(p.pose) for p in path.poses]
        if len(points)<2 or len(motion_sections(points))!=1:
            return True
        a,b=points[:2];heading=yaw(path.poses[0].pose)
        return (b[0]-a[0])*math.cos(heading)+(b[1]-a[1])*math.sin(heading)<=0.

    def retired_replacement_context(self):
        if not (self.full_path_execution and self.visible_approach_active):
            return super().retired_replacement_context()
        motion=self.retired_motion;route=self.route;hint=self.visible_hint
        if (motion is None or motion.get('route') is None or route is None or not route.poses
                or self.expected is None or hint is None or self.query_reset_pending or self.object_found
                or self.memory_goal is not None or self.memory_active is not None
                or self.memory_selecting or self.active_object_confirmation is not None
                or self.confirmed_route_goal is not None or getattr(self,'lookback_active',False)):
            return False
        now=self.get_clock().now().nanoseconds;stamp=stamp_ns(route)
        metadata=self.route_metadata.get(stamp)
        return (metadata is not None
            and metadata.get('exploration_id') not in self.retired_explorations
            and distance(metadata['goal'],self.expected)<=.25
            and route.header.frame_id==motion['route'].header.frame_id
            and stamp>=self.minimum_route_stamp and 0<=now-stamp<=5_000_000_000
            and hint['query']==self.query
            and hint['observation_stamp_ns']>=max(self.query_started_ns,self.confirmation_not_before_ns)
            and 0<=now-hint['observation_stamp_ns']<=2_000_000_000)

    def maybe_extend_path(self):
        if self.retired_motion is not None:
            return super().maybe_extend_path()
        if (self.query_reset_pending or self.pending or self.extending_path
                or self.replacement_pending or self.prepared_extension is not None
                or self.memory_selecting or time.monotonic()<self.extension_retry_at):
            return
        window=self.active_window()
        if window is None or not window[1] or self.latest_route is None:
            return
        route=self.latest_route
        age=(self.get_clock().now().nanoseconds-stamp_ns(route))*1e-9
        if (not 0<=age<=5. or stamp_ns(route)<self.minimum_route_stamp
                or route.header.frame_id!=self.odom.header.frame_id):
            return
        key=self.intent_key(route)
        changed=key!=self._planned_intent
        if not changed and window[2]>CONTINUOUS_HANDOFF_DISTANCE_M:
            return
        if not changed and self._rolling_plan_start is not None and distance(
                xy(self.odom.pose.pose),self._rolling_plan_start)<2.:
            return
        if self.advance_visible_goal() or self.visible_goal_refresh_pending:
            return
        self.route=route
        self.route_complete=self.latest_route_complete
        self.route_endpoint=xy(route.poses[-1].pose)
        self._desired_intent=key
        self.extending_path=True
        self.start_segment(continuous=True)

    def on_follow_accepted(self, future, epoch, target, serial=None, queue=None, replacement=False):
        duplicate = epoch == self.epoch and serial == self.active_follow_serial and self.follow_handle is not None
        super().on_follow_accepted(future,epoch,target,serial,queue,replacement)
        if not duplicate and epoch==self.epoch and self.follow_active is not None and self.active_follow_serial==serial:
            self._retirement_path_stamp=None
            self._motion_route=deepcopy(self.route)
            self._motion_intent=self.intent_key(self.route)
            self._motion_serial,self._motion_epoch=serial,epoch

    def on_follow_result(self, future, epoch, target, serial=None):
        current = epoch == self._motion_epoch and serial == self._motion_serial
        self._closing_motion_serial = serial if current else None
        try:
            return super().on_follow_result(future,epoch,target,serial)
        finally:
            self._closing_motion_serial = None
            if current and not self.following and self.follow_active is None:
                self._motion_route = self._motion_intent = None
                self._motion_serial = self._motion_epoch = None

    def follow_failed(self, reason, **details):
        owns_failure = (self._motion_epoch == self.epoch and self._motion_serial is not None
            and (self.active_follow_serial == self._motion_serial
                 or self._closing_motion_serial == self._motion_serial))
        if owns_failure and self._motion_intent != self._desired_intent:
            # A failed old action says nothing about the new, unexecuted
            # frontier. Stop through the existing physical barrier, retaining
            # only the fresh route for admission after standstill/ACK.
            latest = self.latest_route
            old_intent,serial = self._motion_intent,self._motion_serial
            if self.active_follow_serial is None:
                self.follow_quarantine.setdefault(serial,dict(handle=None,terminal=True))
            self.reset(preserve_execution_corridor=True)
            self.status('continuous_motion_failed',reason=reason,
                        failed_intent=old_intent,requires_standstill=True,**details)
            if latest is not None:
                self.on_route(latest)
            return
        return super().follow_failed(reason,**details)


def main(args=None):
    rclpy.init(args=args)
    node=ContinuousGraphPathBridge()
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
