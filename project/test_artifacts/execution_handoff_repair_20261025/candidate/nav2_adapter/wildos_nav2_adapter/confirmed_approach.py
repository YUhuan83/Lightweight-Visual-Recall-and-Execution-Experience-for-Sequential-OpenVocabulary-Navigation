"""Keep a confirmed approach stable until its accepted route finishes."""
import math
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from .graph_route import distance
from .arrival_criteria import STANDOFF_ARRIVAL_TOLERANCE_M


class ConfirmedTargetApproach:
    def init_confirmed_approach(self):
        self.confirmed_route_goal = None
        self.confirmed_route_object = None
        self.confirmed_route_radius = 0.
        self.confirmed_route_started_at = None
        self.deferred_confirmed_approach = None
        self.confirmed_approach_failure = None
        self.failed_confirmed_goals = []

    def failed_confirmed_view(self, position):
        # A different heading at the same blocked camera baseline is not a
        # new observation. This is query-local navigation evidence only.
        return any(distance(position, p) <= 2*STANDOFF_ARRIVAL_TOLERANCE_M
                   for p in self.failed_confirmed_goals)

    def fail_confirmed_approach(self, reason, **details):
        if (self.object_found or self.query_reset_pending or self.memory_active is not None
                or self.memory_goal is not None or self.confirmed_route_goal is None
                or self.expected is None or self.execution_recovery_owns_motion()
                or distance(self.expected, self.confirmed_route_goal) > 1e-6):
            return False
        if self.confirmed_approach_failure is not None:
            return True
        failed = tuple(self.expected)
        self.failed_confirmed_goals.append(failed)
        self.failed_confirmed_goals = self.failed_confirmed_goals[-8:]
        # Keep the object identity, measured escape corridor and its budget.
        # reset retires old actions; the observation monitor waits for their
        # terminal/standstill barrier before handing off to another waypoint.
        self.reset(preserve_execution_corridor=True)
        self.confirmed_approach_failure = dict(goal=failed, reason=reason)
        self.confirmed_route_started_at = None
        self.live_observation_deadline = None
        self.observation_wait_until = time.monotonic()
        self.status('confirmed_approach_failed', goal=list(failed), reason=reason,
                    requires_visual_confirmation=True, **details)
        return True

    def consume_confirmed_approach(self):
        data = self.deferred_confirmed_approach
        if (data is None or self.object_found or self.memory_selecting
                or self.confirmed_route_busy() or self.input_freshness_issue() is not None):
            return False
        self.deferred_confirmed_approach = None
        if data['query'] != self.query or not self.confirmed_instance_matches(data):
            return False
        from .graph_path_bridge import xy
        previous = self.expected
        self.route_confirmed_target(data, xy(self.odom.pose.pose))
        # A retained world coordinate is only a navigation seed. It never
        # re-enters on_object_confirmation or creates fresh arrival evidence.
        return self.expected != previous

    def confirmed_route_busy(self):
        waiting = (self.route is None and self.confirmed_route_started_at is not None
                   and time.monotonic()-self.confirmed_route_started_at < 10.)
        return (self.pending or self.following or waiting
                or self.confirmed_approach_failure is not None)

    def confirmed_instance_matches(self, data):
        # Keep the association origin fixed for this route, rather than letting
        # a succession of nearby patches walk the anchor onto another instance.
        return (self.confirmed_route_object is None
                or distance(self.confirmed_route_object, data['object_position'])
                   <= 3. + self.confirmed_route_radius + float(data['radius']))

    def route_confirmed_target(self, data, vehicle_position):
        standoff = tuple(map(float, data['standoff']))
        previous = self.expected
        if self.failed_confirmed_view(standoff):
            return True  # Fresh evidence cannot revive a failed waypoint.
        if self.confirmed_approach_failure is not None:
            if self.confirmed_instance_matches(data):
                self.deferred_confirmed_approach = deepcopy(data)
            return True  # The failure/stop transaction owns this handoff.
        latched = (self.confirmed_route_goal is not None and previous is not None
                   and distance(previous, self.confirmed_route_goal) < 1e-6)
        if previous is not None and distance(previous, standoff) <= STANDOFF_ARRIVAL_TOLERANCE_M:
            if self.confirmed_route_goal is None:
                self.confirmed_route_goal = tuple(previous)
                self.confirmed_route_object = tuple(data['object_position'][:2])
                self.confirmed_route_radius = float(data['radius'])
                self.confirmed_route_started_at = time.monotonic()
            return False
        if latched and self.confirmed_route_busy():
            self.deferred_confirmed_approach = deepcopy(data)
            self.status('confirmed_standoff_update_deferred', query=self.query,
                        goal=list(previous), proposed_goal=list(standoff),
                        reason='accepted_route_active')
            return True
        if latched:
            progress = (distance(previous, self.confirmed_route_object)
                        - distance(standoff, self.confirmed_route_object))
            required_progress = (.6 if distance(vehicle_position, previous)
                                 <= STANDOFF_ARRIVAL_TOLERANCE_M else 2.)
            if progress < required_progress:
                self.deferred_confirmed_approach = deepcopy(data)
                self.status('confirmed_standoff_update_deferred', query=self.query,
                            goal=list(previous), proposed_goal=list(standoff),
                            reason='no_material_approach_progress')
                return True

        goal = PoseStamped()
        goal.header.frame_id = 'spot1/odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = standoff
        goal.pose.position.z = self.odom.pose.pose.position.z
        heading = math.atan2(data['object_position'][1]-standoff[1],
                             data['object_position'][0]-standoff[0])
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        if not self.prepare_intent_change():
            self.reset()
        # Ownership passes from the observation action to this confirmed
        # route. Its old deadline must never cancel the replacement route.
        self.observation_active = False
        self.live_observation_target = None
        self.live_observation_deadline = None
        self.deferred_confirmed_approach = None
        self.expected = self.confirmed_route_goal = standoff
        self.confirmed_route_object = tuple(data['object_position'][:2])
        self.confirmed_route_radius = float(data['radius'])
        self.confirmed_route_started_at = time.monotonic()
        self.observation_wait_until = None
        self.query_grace_until = 0.
        if self.memory_active is not None:
            self.memory_arrival_heading = heading
        self.relay_destination(goal)
        self.status('memory_standoff_approach' if self.memory_active else 'object_standoff_approach',
                    query=self.query, previous_goal=list(previous) if previous else None,
                    goal=list(standoff), vehicle_position=list(vehicle_position),
                    object_position=data['object_position'],
                    distance=distance(vehicle_position, standoff),
                    tolerance=STANDOFF_ARRIVAL_TOLERANCE_M)
        return True
