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
        self.visible_blocked = None
        self.visible_reobserve_until = None

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
        if self.execution_recovery_owns_motion():
            return False
        reobserving = self.visible_approach_active and self.visible_reobserve_until is not None
        if reobserving:
            if time.monotonic() >= self.visible_reobserve_until:
                self.release_visible_approach('visible_target_reobservation_timeout')
                return False
        elif self.visible_approach_active:
            if (not self.following and not self.pending
                    and time.monotonic()-self.visible_approach_started_at >= 10.):
                self.release_visible_approach('visible_target_route_timeout')
            return False
        data = self.visible_hint
        if (data is None or self.object_found or self.memory_goal is not None
                or self.memory_active is not None or self.memory_selecting
                or self.active_object_confirmation is not None
                or (self.expected is not None and not self.visible_approach_active)
                or self.query_reset_pending or self.odom is None
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
        self.reset(preserve_execution_corridor=True)
        self.visible_approach_active = True
        self.visible_approach_started_at = time.monotonic()
        self.visible_reobserve_until = None
        self.expected = point
        self.observation_wait_until = None
        self.query_grace_until = 0.
        self.relay_destination(pose)
        self.status('visible_target_approach', goal=list(point),
                    target_range_known=False, requires_visual_confirmation=True)
        return True

    def finish_visible_approach(self):
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
        # A short detection gap is not a completed or abandoned target search.
        # Keep ownership at the reached pose, with no path/action to repeat.
        # Only a fresh mask-derived hint may start the next checked route.
        self.expected = completed
        self.visible_approach_active = True
        self.visible_reobserve_until = time.monotonic()+REOBSERVATION_WAIT_SEC
        self.status('visible_target_reobserving', goal=list(completed),
                    target_range_known=False, requires_visual_confirmation=True,
                    wait_sec=REOBSERVATION_WAIT_SEC)

    def release_visible_approach(self, reason):
        if self.execution_recovery_owns_motion():
            return
        failed = self.expected
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
