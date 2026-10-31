"""Bounded observation viewpoints for one running object-search session."""
from copy import deepcopy
import math


class ViewpointMemory:
    def __init__(self, max_views=8, bin_count=16):
        self.max_views = max(1, int(max_views))
        self.bin_count = max(1, int(bin_count))
        self.views = {}
        self.serial = 0

    @staticmethod
    def key(query):
        return ' '.join(str(query).casefold().split())

    def __contains__(self, query):
        return bool(self.views.get(self.key(query)))

    def observe(self, query, frame, position, heading, now):
        values = [*position, heading, now]
        if len(position) != 3 or not frame or not all(math.isfinite(v) for v in values):
            raise ValueError('invalid observed viewpoint')
        key = self.key(query)
        if not key:
            raise ValueError('empty target')
        index = round((heading % (2 * math.pi)) * self.bin_count / (2 * math.pi)) % self.bin_count
        views = self.views.setdefault(key, [])
        match = next((v for v in views if v['frame'] == frame and v['bin'] == index
                      and math.dist(v['position'], position) < 1.0), None)
        if match is None:
            self.serial += 1
            match = {'id': str(self.serial), 'count': 0}
            views.append(match)
        match.update(frame=frame, position=list(position), heading=float(heading),
                     bin=index, observed_at=float(now), count=match['count'] + 1,
                     blocked_until=0.0)
        # Repeated observations at one location do not erase distinct views.
        if len(views) > self.max_views:
            views.remove(min(views, key=lambda v: v['observed_at']))
        return deepcopy(match)

    def candidates(self, query, now):
        return deepcopy([v for v in self.views.get(self.key(query), [])
                         if v['blocked_until'] <= now])

    def reject(self, query, view_id, now, cooldown=30.0):
        for view in self.views.get(self.key(query), []):
            if view['id'] == view_id:
                view['blocked_until'] = float(now) + max(0.0, float(cooldown))
                return True
        return False
