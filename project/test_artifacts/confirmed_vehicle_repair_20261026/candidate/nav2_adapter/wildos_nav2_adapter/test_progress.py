"""Bounded maneuver progress for field trials; never a task success signal."""
import math


class ManeuverProgress:
    def __init__(self):
        self.best = {}

    def observe(self, point, status):
        """Count new approach progress without rewarding repeated shuttling.

        Goals share a half-metre bin across replans. A new goal merely records
        a baseline; actual movement must improve its best distance by 0.3 m.
        The harness separately enforces a total deadline and task acceptance.
        """
        if not point or not status or status.get('status') != 'following':
            return False
        goal = status.get('goal')
        if not goal or len(goal) < 2 or not all(map(math.isfinite, (*point, *goal[:2]))):
            return False
        key = tuple(round(v*2) for v in goal[:2])
        gap = math.dist(point[:2], goal[:2])
        previous = self.best.setdefault(key, gap)
        if gap < previous-.3:
            self.best[key] = gap
            return True
        return False
