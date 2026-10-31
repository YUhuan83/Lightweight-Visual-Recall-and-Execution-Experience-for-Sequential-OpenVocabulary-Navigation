"""ROS-independent validation and serialized latest-goal arbitration."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Goal:
    frame: str
    stamp_ns: int
    position: tuple
    quaternion: tuple

    @property
    def yaw(self):
        x, y, z, w = self.quaternion
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def validate(goal, now_ns, expected_frame='odom', max_age=5.0):
    if goal.frame != expected_frame:
        return 'unexpected frame (no automatic TF conversion)'
    if len(goal.position) != 3 or len(goal.quaternion) != 4:
        return 'invalid pose dimensions'
    if not all(math.isfinite(v) for v in (*goal.position, *goal.quaternion)):
        return 'non-finite pose'
    if abs(math.hypot(*goal.quaternion) - 1.0) > 0.001:
        return 'quaternion must be unit length'
    if now_ns <= 0 or goal.stamp_ns <= 0:
        return 'clock not initialized or zero/negative stamp'
    age_ns = now_ns - goal.stamp_ns
    if age_ns < 0:
        return 'future stamp (check clock domain)'
    if age_ns > max_age * 1_000_000_000:
        return 'expired goal'
    return None


def materially_changed(a, b, distance=0.1, heading=0.1):
    delta = a.yaw - b.yaw
    return (math.dist(a.position, b.position) >= distance or
            abs(math.atan2(math.sin(delta), math.cos(delta))) >= heading)


class GoalPolicy:
    """One in-flight task, one replaceable pending goal, monotonic callback IDs.

    Cancel acknowledgement is NOT completion. Only a terminal result releases
    the slot. Transport errors freeze the slot rather than risk overlapping goals.
    """

    def __init__(self, distance=0.1, heading=0.1):
        if not all(math.isfinite(v) and v > 0 for v in (distance, heading)):
            raise ValueError('distance and heading thresholds must be finite and positive')
        self.distance, self.heading = distance, heading
        self.pending = None
        self.current = None
        self.last_sent = None
        self.state = 'idle'
        self.sequence = 0
        self.token = None
        self.latest_stamp = 0
        self.stopping = False

    def offer(self, goal):
        if self.stopping or goal.stamp_ns < self.latest_stamp:
            return False
        self.latest_stamp = goal.stamp_ns
        baseline = self.current if self.current is not None else self.last_sent
        if baseline is not None and not materially_changed(
                baseline, goal, self.distance, self.heading):
            # A newer return to the current goal supersedes any queued detour.
            # Once cancellation starts, however, the current goal must be resent.
            self.pending = goal if self.state == 'canceling' else None
            return self.pending is not None
        self.pending = goal
        return True

    def begin_send(self):
        if self.stopping or self.state != 'idle' or self.pending is None:
            return None
        self.sequence += 1
        self.token = self.sequence
        self.current, self.pending = self.pending, None
        self.last_sent = self.current
        self.state = 'sending'
        return self.token, self.current

    def accepted(self, token, accepted):
        if token != self.token or self.state != 'sending':
            return False
        if accepted:
            self.state = 'active'
        else:
            self.finish(token)
        return True

    def begin_cancel(self):
        if self.state != 'active' or not (self.stopping or self.pending is not None):
            return None
        self.state = 'canceling'
        return self.token

    def finish(self, token):
        if token != self.token or self.state == 'idle':
            return False
        self.state = 'idle'
        self.current = self.token = None
        return True

    def fault(self, token):
        if token == self.token and self.state != 'idle':
            self.state = 'blocked'

    def stop(self):
        self.stopping = True
        self.pending = None
