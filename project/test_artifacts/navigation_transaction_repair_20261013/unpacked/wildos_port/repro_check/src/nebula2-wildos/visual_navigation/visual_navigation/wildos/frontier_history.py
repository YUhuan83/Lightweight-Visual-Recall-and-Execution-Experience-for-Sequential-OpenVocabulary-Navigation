"""Bounded, short-lived suppression of recently retired frontiers."""

from collections import deque


class RecentFrontierHistory:
    def __init__(self, ttl_seconds=300.0, max_entries=512):
        self.ttl_seconds = max(0.0, float(ttl_seconds))
        self.entries = deque(maxlen=max(1, int(max_entries)))

    def add(self, position, now):
        self.prune(now)
        self.entries.append((float(now), float(position[0]), float(position[1])))

    def prune(self, now):
        cutoff = float(now) - self.ttl_seconds
        while self.entries and self.entries[0][0] < cutoff:
            self.entries.popleft()

    def nearby(self, position, radius, now):
        self.prune(now)
        x, y = float(position[0]), float(position[1])
        radius_sq = max(0.0, float(radius)) ** 2
        return any((x - px) ** 2 + (y - py) ** 2 < radius_sq
                   for _, px, py in self.entries)
