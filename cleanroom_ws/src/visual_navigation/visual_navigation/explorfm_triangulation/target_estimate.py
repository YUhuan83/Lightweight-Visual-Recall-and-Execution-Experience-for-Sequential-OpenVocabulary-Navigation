"""A bounded spatial consensus of measured XYZ observations for one query."""

from collections import deque
import numpy as np


class TargetEstimate:
    def __init__(self):
        self.samples = deque(maxlen=5)
        self.position = None

    def clear(self):
        self.samples.clear()
        self.position = None

    def update(self, point, stamp_ns):
        if len(point) != 3 or not np.isfinite(point).all():
            return None
        if self.samples and stamp_ns <= self.samples[-1][0]:
            return None
        self.samples.append((stamp_ns, np.asarray(point, dtype=float)))
        while self.samples and stamp_ns - self.samples[0][0] > 10_000_000_000:
            self.samples.popleft()
        points = np.array([sample[1] for sample in self.samples])
        neighbours = (
            np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2) <= 1.5
        )
        if self.position is not None:
            # A new recent cluster is not automatically the same object.
            # Keep association across gaps until the query is cleared. Use
            # XY here so roof/facade height changes can still be refined.
            associated = (
                np.linalg.norm(points[:, :2] - self.position[:2], axis=1) <= 1.5
            )
            neighbours &= associated[:, None] & associated[None, :]
        selected = neighbours[np.argmax(neighbours.sum(axis=1))]
        if selected.sum() < 3 or not selected[-1]:
            return None
        measured = points[selected]
        center = np.median(measured, axis=0)
        self.position = center.copy()
        return dict(
            position=center,
            radius=float(np.linalg.norm(measured - center, axis=1).max()),
            inliers=int(selected.sum()),
            observations=len(points),
        )
