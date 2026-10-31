"""Bounded, car-like observation approaches around a remembered object region."""
import math
import time
from .graph_route import distance


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
    def init_observation(self):
        self.observation_attempts = set()
        self.observation_wait_until = None
        self.observation_active = False
        self.unconfirmed_approaches = []

    def recently_unconfirmed(self, point):
        now = time.monotonic()
        self.unconfirmed_approaches = [(p, deadline) for p, deadline in
                                      self.unconfirmed_approaches if deadline > now]
        return any(distance(point, p) < 5. for p, _ in self.unconfirmed_approaches)

    def await_object_observation(self):
        if self.object_found or self.expected is None:
            return
        if self.observation_wait_until is None:
            self.observation_wait_until = time.monotonic() + 5.
            self.status('awaiting_visual_confirmation', goal=list(self.expected),
                        requires_visual_confirmation=True,
                        approaches_tried=len(self.observation_attempts))

    def monitor_observation(self):
        if (self.object_found or self.memory_selecting
                or self.pending or self.following or self.observation_wait_until is None
                or time.monotonic() < self.observation_wait_until
                or self.input_freshness_issue() is not None or self.query_reset_pending):
            return
        if self.memory_active is None:
            # Arrival at a semantic waypoint proves neither the class nor a
            # successful search. Release its explicit graph goal if fresh
            # observations did not confirm it, and resume frontier selection.
            failed = self.expected
            if failed is not None:
                self.unconfirmed_approaches.append((failed, time.monotonic()+60.))
                self.unconfirmed_approaches = self.unconfirmed_approaches[-16:]
            self.reset()
            self.expected = None
            self.pending_destination = None
            self.observation_wait_until = None
            self.query_reset_pending = True
            self.query_reset_inflight = False
            self.status('unconfirmed_target_resume_exploration',
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
        object_position = self.memory_active.get('object_position') if self.memory_active else None
        if not self.observation_active or not object_position:
            return None
        heading = math.atan2(object_position[1]-endpoint[1], object_position[0]-endpoint[0])
        # The front camera must face the target. The ideal point carrier can
        # rotate in place; the Ackermann profile checks heading through its path.
        fov = min(math.pi, max(.2, self.memory_active.get('horizontal_fov', 1.4)))
        return [heading, heading-fov/4, heading+fov/4]
