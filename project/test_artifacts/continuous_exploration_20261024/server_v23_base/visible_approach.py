"""Execute bounded observation goals without treating them as target arrival."""
import json
import math
import time
from geometry_msgs.msg import PoseStamped
from .graph_route import distance

# A completed observation step retains its task while fresh evidence is
# reacquired. This deadline never extends the lifetime of an old mask.
REOBSERVATION_WAIT_SEC = 10.


class VisibleTargetApproach:
    def init_visible_approach(self):
        self.visible_hint = None
        self.visible_approach_active = False
        self.visible_approach_started_at = None
        self.visible_approach_deadline = None
        self.visible_blocked = None
        self.visible_reobserve_until = None
        self.visible_reobserve_exploration = False
        self.visible_goal_refresh_pending = False

    def check_visible_reobserve_deadline(self):
        """Stop waiting for a fresh mask while frontier exploration continues."""
        deadline = self.visible_reobserve_until
        if (not self.visible_reobserve_exploration or deadline is None
                or time.monotonic() < deadline):
            return False
        self.visible_reobserve_until = None
        self.visible_reobserve_exploration = False
        if self.object_found:
            return False
        self.status('visible_target_reobservation_timeout',
                    query=self.query, exploration_fallback_requested=True)
        return True

    def advance_visible_goal(self):
        """Request the next fresh observation step before the forward leg ends.

        This changes the GraphNav intention, never the accepted FollowPath.
        Its replacement still goes through the ordinary background planner,
        map/curvature checks and action acceptance.
        """
        data = self.visible_hint
        if (not self.visible_approach_active or self.visible_reobserve_until is not None
                or self.visible_goal_refresh_pending or getattr(self,'lookback_active',False)
                or not self.following or self.follow_handle is None or self.pending
                or self.follow_quarantine or self.query_reset_pending or self.object_found
                or self.expected is None or self.active_object_confirmation is not None
                or self.confirmed_route_goal is not None or self.memory_goal is not None
                or self.memory_active is not None or self.memory_selecting
                or self.execution_recovery_owns_motion() or data is None
                or self.input_freshness_issue() is not None
                or self.execution_transport_issue() is not None):
            return False
        now_ns=self.get_clock().now().nanoseconds
        if not 0 <= now_ns-data['observation_stamp_ns'] <= 2_000_000_000:
            return False
        from .graph_path_bridge import xy
        current=xy(self.odom.pose.pose)
        point=tuple(data['point']);previous=self.expected
        # Reuse the ordinary 2--10m proposal envelope. Require a full route
        # sampling interval of extra progress, not a frame-by-frame retarget.
        if (not 2. <= distance(current,point) <= 10.
                or distance(current,point)-distance(current,previous) < 4.
                or distance(point,previous) < 4.
                or sum((p-c)*(g-c) for p,g,c in zip(point,previous,current)) <= 0
                or self.recently_unconfirmed(point)):
            return False
        blocked=self.visible_blocked
        if blocked and time.monotonic()<blocked[1] and distance(point,blocked[0])<2.:
            return False
        pose=PoseStamped();pose.header.frame_id='spot1/odom'
        pose.pose.position.x,pose.pose.position.y=point
        pose.pose.position.z=self.odom.pose.pose.position.z
        pose.pose.orientation.z=math.sin(data['bearing']/2)
        pose.pose.orientation.w=math.cos(data['bearing']/2)
        self.expected=point
        self.latest_route=None
        self.visible_goal_refresh_pending=True
        self.visible_approach_started_at=time.monotonic()
        self.visible_approach_deadline=self.visible_approach_started_at+30.
        self.relay_destination(pose)
        # Reuse the accepted-goal event consumed by GoalStandoff feedback;
        # an unaccepted newer proposal must not replace this failure owner.
        self.status('visible_target_approach',goal=list(point),previous_goal=list(previous),
                    continuous=True,target_range_known=False,requires_visual_confirmation=True)
        return True

    def check_visible_approach_deadline(self):
        if (self.visible_approach_active and self.visible_approach_deadline is not None
                and time.monotonic() > self.visible_approach_deadline):
            # release_visible_approach cannot revoke physical recovery. Do
            # not report a completed release to monitor_inputs while that
            # release is deferred: recovery still needs its monitor ticks.
            if self.execution_recovery_owns_motion():
                return False
            self.release_visible_approach(
                'side_lookback_timeout' if getattr(self, 'lookback_active', False)
                else 'visible_target_approach_timeout')
            return True
        return False

    def on_visible_approach(self, message):
        try:
            data = json.loads(message.data)
            stamp = int(data['observation_stamp_ns'])
            age = self.get_clock().now().nanoseconds-stamp
            point = data['point']
            if (data['query'] != self.query or data['frame'] != 'spot1/odom'
                    or stamp < max(self.query_started_ns, self.confirmation_not_before_ns)
                    or self.instance_release_pending is not None or not 0 <= age <= 2_000_000_000
                    or data.get('evidence') != 'bearing_only'
                    or data.get('position_confirmed') is not False
                    or data.get('search_confirmed') is not False
                    or len(point) != 2 or not all(map(math.isfinite, [*point, data['bearing']]))):
                return
            if self.visible_hint and stamp < self.visible_hint['observation_stamp_ns']:
                return
        except (ValueError, TypeError, KeyError):
            return
        self.visible_hint = data
        self.activate_visible_approach()

    def activate_visible_approach(self):
        self.consume_pending_odometry()
        if self.execution_recovery_owns_motion():
            return False
        if self.check_visible_approach_deadline():
            return False
        self.check_visible_reobserve_deadline()
        reobserving = self.visible_reobserve_until is not None
        if reobserving and time.monotonic() >= self.visible_reobserve_until:
            self.release_visible_approach('visible_target_reobservation_timeout')
            return False
        elif self.visible_approach_active:
            if (self.route is None and not self.following and not self.pending
                    and time.monotonic()-self.visible_approach_started_at >= 10.):
                # This is the wait for GraphNav to supply a route, not a
                # timeout on a momentary idle gap after driving a segment.
                # Accepted routes remain bounded by the overall deadline.
                self.release_visible_approach('visible_target_route_timeout')
            return False
        data = self.visible_hint
        draining_target_motion = (self.retired_motion is not None
                                  and self.retired_motion.get('owner_kind')
                                  == 'visible_target_timeout')
        if (data is None or self.object_found or self.memory_goal is not None
                or self.memory_active is not None or self.memory_selecting
                or self.active_object_confirmation is not None
                or (self.expected is not None and not self.visible_approach_active)
                or (self.query_reset_pending and not draining_target_motion)
                or self.odom is None
                or self.input_freshness_issue() is not None):
            return False
        age = self.get_clock().now().nanoseconds-data['observation_stamp_ns']
        if not 0 <= age <= 2_000_000_000:
            return False
        # A visible target can replace ordinary exploration once. Subsequent
        # observations do not cancel the accepted segment on every video frame.
        from .graph_path_bridge import xy
        current = xy(self.odom.pose.pose)
        point = tuple(data['point'])
        if self.recently_unconfirmed(point):
            return False
        blocked = self.visible_blocked
        if blocked and time.monotonic() < blocked[1] and distance(point, blocked[0]) < 2.:
            return False
        if not 2. <= distance(current, point) <= 10.:
            return False
        pose = PoseStamped()
        pose.header.frame_id = 'spot1/odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = point
        pose.pose.position.z = self.odom.pose.pose.position.z
        pose.pose.orientation.z = math.sin(data['bearing']/2)
        pose.pose.orientation.w = math.cos(data['bearing']/2)
        # A fresh hint cannot cancel an already checked draining section.
        draining_motion = self.retired_motion is not None
        if draining_motion:
            self.clear_exploration_comparison()
            self.latest_route = None
        else:
            self.reset(preserve_execution_corridor=True)
        self.visible_approach_active = True
        self.visible_approach_started_at = time.monotonic()
        self.visible_approach_deadline = time.monotonic() + (
            15. if getattr(self, 'lookback_active', False) else 30.)
        self.visible_reobserve_until = None
        self.visible_reobserve_exploration = False
        self.expected = point
        self.observation_wait_until = None
        self.query_grace_until = 0.
        self.relay_destination(pose)
        self.status('visible_target_approach', goal=list(point),
                    target_range_known=False, requires_visual_confirmation=True,
                    handoff_deferred=draining_motion)
        return True

    def finish_visible_approach(self):
        if self.check_visible_approach_deadline():
            return
        if getattr(self, 'lookback_active', False):
            completed = self.expected
            self.lookback_active = False
            self.reset(preserve_execution_corridor=True)
            self.expected = None
            self.visible_approach_active = False
            self.pending_destination = None
            self.status('side_lookback_done',
                        goal=list(completed) if completed is not None else None)
            self.request_graph_reset()
            self.retry_graph_reset()
            return
        completed = self.expected
        self.reset(preserve_execution_corridor=True)
        self.expected = None
        self.visible_approach_active = False
        self.pending_destination = None
        self.observation_wait_until = None
        self.status('visible_target_observation_step_done',
                    target_range_known=False, requires_visual_confirmation=True)
        if self.activate_visible_approach():
            return
        # Do not leave the vehicle idle while waiting for the next camera mask.
        # Release the completed observation goal to GraphNav immediately; a
        # later fresh hint can still preempt exploration through the normal
        # reset barrier and checked target route.
        self.expected = None
        self.visible_approach_active = False
        self.visible_reobserve_until = time.monotonic()+REOBSERVATION_WAIT_SEC
        self.visible_reobserve_exploration = True
        self.status('visible_target_reobserving', goal=list(completed),
                    target_range_known=False, requires_visual_confirmation=True,
                    wait_sec=REOBSERVATION_WAIT_SEC,
                    exploration_fallback_requested=True)
        self.request_graph_reset()
        self.retry_graph_reset()

    def release_visible_approach(self, reason):
        if self.execution_recovery_owns_motion():
            return
        if getattr(self, 'lookback_active', False):
            failed = self.expected
            self.lookback_active = False
            self.reset(preserve_execution_corridor=True)
            self.expected = None
            self.visible_approach_active = False
            self.pending_destination = None
            self.status('side_lookback_failed',
                        goal=list(failed) if failed is not None else None,
                        reason=reason)
            self.request_graph_reset()
            self.retry_graph_reset()
            return
        failed = self.expected
        if (reason == 'visible_target_approach_timeout' and failed is not None
                and self.finish_current_exploration_section(
                    'visible-target-timeout:{}:{:.3f}:{:.3f}'.format(
                        self.query_generation, failed[0], failed[1]),
                    owner_kind='visible_target_timeout')
                and self.retired_motion is not None
                and (self.retired_motion.get('forward')
                     or not self.retired_motion.get('bounded'))):
            # Keep the current revalidated section alive while GraphNav
            # obtains a new route. A short reverse cusp may finish; a longer
            # reverse path is rejected before retirement. Each borrowed
            # cycle rechecks odom, transport and both costmaps.
            self.clear_exploration_comparison()
            self.init_visible_approach()
            self.expected = None
            self.visible_blocked = (failed, time.monotonic()+15.)
            self.pending_destination = None
            self.route = self.latest_route = None
            self.route_endpoint = None
            self.route_complete = self.latest_route_complete = True
            self.request_graph_reset()
            self.status(reason, goal=list(failed), retained_current_section=True,
                        handoff_forward=self.retired_motion['forward'],
                        handoff_bounded=self.retired_motion['bounded'],
                        handoff_distance_limit_m=(5. if self.retired_motion['bounded'] else None),
                        handoff_timeout_sec=(5. if self.retired_motion['bounded'] else None),
                        requires_visual_confirmation=True)
            self.retry_graph_reset()
            return
        self.reset(preserve_execution_corridor=True)
        self.init_visible_approach()
        self.expected = None
        if failed is not None:
            self.visible_blocked = (failed, time.monotonic()+15.)
        self.pending_destination = None
        self.request_graph_reset()
        self.status(reason, goal=list(failed) if failed is not None else None,
                    requires_visual_confirmation=True)
        self.retry_graph_reset()
