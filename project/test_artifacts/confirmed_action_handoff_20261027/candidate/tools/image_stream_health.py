"""Bounded timestamp/identity check for a raw-to-preview diagnostic stream."""
from collections import OrderedDict


class ImageStreamHealth:
    def __init__(self):
        self.sources = OrderedDict()
        self.previews = OrderedDict()
        self.last_preview_ns = 0
        self.advancing = 0
        self.last_source_ns = 0
        self.advancing_sources = 0

    def observe(self, label, stamp_ns, frame_id, now_ns):
        if (stamp_ns <= 0 or not frame_id or not -50_000_000 <= now_ns-stamp_ns <= 2_000_000_000):
            return
        target = self.sources if label == 'camera' else self.previews
        target[(stamp_ns, frame_id)] = None
        while len(target) > 128:
            target.popitem(last=False)
        if label == 'preview' and stamp_ns > self.last_preview_ns:
            self.last_preview_ns = stamp_ns
            self.advancing += 1
        if label == 'camera' and stamp_ns > self.last_source_ns:
            self.last_source_ns = stamp_ns
            self.advancing_sources += 1

    def source_report(self, now_ns):
        age = (now_ns-self.last_source_ns)/1e9 if self.last_source_ns else None
        return dict(advancing_camera_samples=self.advancing_sources,
                    last_camera_age_sec=age,
                    passed=bool(self.advancing_sources >= 2 and age is not None
                                and -.05 <= age <= 2.))

    def report(self, now_ns):
        matched = len(self.sources.keys() & self.previews.keys())
        age = (now_ns-self.last_preview_ns)/1e9 if self.last_preview_ns else None
        return dict(advancing_preview_samples=self.advancing, matched_source_samples=matched,
                    last_preview_age_sec=age,
                    passed=bool(self.advancing >= 2 and matched >= 2 and age is not None
                                and -.05 <= age <= 2.))
