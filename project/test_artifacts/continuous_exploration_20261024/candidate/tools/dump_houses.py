#!/usr/bin/env python3
import json, math
m = json.load(open('/home/ly/nizhen_countryside_deploy/standalone_rural/generated/build_manifest.json'))
sp = m['spawn']['pos']
print("spawn", [round(x, 2) for x in sp])
print("road_width_m", m.get('road_width_m'), "loop_length_m", round(m.get('loop_length_m', 0), 1))
houses = m['houses']
print("houses type:", type(houses).__name__, "count", len(houses))
def pos_of(h):
    if isinstance(h, dict):
        for k in ('pos', 'position', 'location', 'center'):
            if k in h:
                return h[k], h
        return None, h
    if isinstance(h, (list, tuple)) and len(h) >= 3 and all(isinstance(x, (int, float)) for x in h[:3]):
        return list(h[:3]), h
    return None, h
rows = []
for h in houses:
    p, raw = pos_of(h)
    if p is None:
        print("RAW", json.dumps(h)[:300]); continue
    dx, dy = p[0] - sp[0], p[1] - sp[1]
    dist = math.hypot(dx, dy)
    ang = math.degrees(math.atan2(dy, dx))
    rows.append((dist, ang, p, raw))
for dist, ang, p, raw in sorted(rows):
    extra = {k: v for k, v in raw.items() if k not in ('pos',)} if isinstance(raw, dict) else ''
    print("dist=%7.2f  ang_deg=%7.1f  pos=%s  %s" % (dist, ang, [round(x, 2) for x in p], json.dumps(extra)[:160] if extra else ''))
