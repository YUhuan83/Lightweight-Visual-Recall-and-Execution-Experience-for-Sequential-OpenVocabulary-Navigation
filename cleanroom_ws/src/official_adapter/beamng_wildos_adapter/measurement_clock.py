"""Align an advancing sensor clock with ROS time after simulation pauses."""

from collections import deque


def closest_state_sample(state_samples, source_ns, max_skew_ns):
    """Return the closest timestamped pose sample within the allowed skew."""
    if not state_samples:
        return None
    sample = min(state_samples, key=lambda item: abs(item[0] - source_ns))
    skew_ns = sample[0] - source_ns
    return (sample, skew_ns) if abs(skew_ns) <= max_skew_ns else None


class MeasurementClock:
    def __init__(self, tolerance_ns=500_000_000):
        self.tolerance_ns = tolerance_ns
        self.offset = None
        self.cutoff = None
        self.last_source = None
        self.last_mapped = None
        self.last_update_reset = False
        self.anchors = deque(maxlen=200)

    def update(self, source_ns, ros_ns):
        self.last_update_reset = False
        if (
            self.last_source is not None
            and source_ns < self.last_source - self.tolerance_ns
            and (self.last_mapped is None or ros_ns > self.last_mapped)
        ):
            # A restarted simulation can reset its sensor clock while ROS time
            # keeps advancing. Start a new epoch and reject queued frames from
            # the old epoch via the new cutoff.
            self.offset = ros_ns - source_ns
            self.cutoff = source_ns
            self.last_source = source_ns
            self.last_mapped = ros_ns
            self.anchors.clear()
            self.anchors.append((source_ns, ros_ns))
            self.last_update_reset = True
            return ros_ns
        # Repeated paused measurements keep their original time. A delayed
        # state cannot move the clock backwards or refresh old sensor data.
        if self.last_source is not None and source_ns <= self.last_source:
            return self.last_mapped if source_ns == self.last_source else None
        # Never forecast ROS time from the simulation clock. A small change in
        # real-time factor previously produced future odometry for up to 0.5 s,
        # which downstream freshness guards correctly refused to use.
        if self.last_mapped is not None and ros_ns <= self.last_mapped:
            return None
        mapped = source_ns + self.offset if self.offset is not None else None
        discontinuity = mapped is None or abs(mapped - ros_ns) > self.tolerance_ns
        if (
            mapped is None
            or mapped > ros_ns
            or mapped <= (self.last_mapped or 0)
            or ros_ns - mapped > self.tolerance_ns
        ):
            self.offset = ros_ns - source_ns
            if discontinuity:
                self.cutoff = source_ns
                self.anchors.clear()
        self.last_source = source_ns
        self.last_mapped = source_ns + self.offset
        self.anchors.append((source_ns, self.last_mapped))
        return self.last_mapped

    def map(self, source_ns):
        # Frames newer than the last state have no validated clock anchor yet.
        if (
            self.offset is None
            or source_ns < self.cutoff
            or source_ns > self.last_source
        ):
            return None
        # Interpolate between validated state stamps, rather than re-stamping
        # queued camera/LiDAR frames using a newly adjusted offset.
        previous = None
        for source, mapped in self.anchors:
            if source == source_ns:
                return mapped
            if source > source_ns:
                if previous is None:
                    return None
                a, b = previous
                return b + (mapped - b) * (source_ns - a) // (source - a)
            previous = source, mapped
        return None
