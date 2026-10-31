"""Align an advancing sensor clock with ROS time after simulation pauses."""


class MeasurementClock:
    def __init__(self, tolerance_ns=500_000_000):
        self.tolerance_ns = tolerance_ns
        self.offset = None
        self.cutoff = None
        self.last_source = None
        self.last_mapped = None

    def update(self, source_ns, ros_ns):
        # Repeated paused measurements keep their original time. A delayed
        # state cannot move the clock backwards or refresh old sensor data.
        if self.last_source is not None and source_ns <= self.last_source:
            return self.last_mapped if source_ns == self.last_source else None
        if self.offset is None or abs(source_ns + self.offset - ros_ns) > self.tolerance_ns:
            self.offset = ros_ns - source_ns
            self.cutoff = source_ns
        self.last_source = source_ns
        self.last_mapped = source_ns + self.offset
        return self.last_mapped

    def map(self, source_ns):
        if self.offset is None or source_ns < self.cutoff:
            return None
        return source_ns + self.offset
