#!/usr/bin/env python3
import json, math, zipfile
ZIP = '/home/ly/beamng-maps/nizhen_countryside_standalone.zip'
spawn = (-40.0, -80.0)
items = []
with zipfile.ZipFile(ZIP) as z:
    for f in z.infolist():
        if not f.filename.endswith('items.level.json'):
            continue
        for line in z.read(f).decode('utf-8-sig').splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except Exception:
                continue
            items.append((f.filename, row))
print("total items", len(items))
import collections
cls = collections.Counter(r.get('class') for _, r in items)
print("classes", dict(cls))
rows = []
for fn, r in items:
    asset = ' '.join(str(r.get(k, '')) for k in ('name', 'internalName', 'shapeName', 'shapeFile', 'fileName', '__parent')).lower()
    if 'house' in asset or 'building' in asset or 'cabin' in asset:
        p = r.get('position')
        if isinstance(p, list) and len(p) >= 3:
            dx, dy = p[0] - spawn[0], p[1] - spawn[1]
            rows.append((math.hypot(dx, dy), math.degrees(math.atan2(dy, dx)), p, r.get('name'), asset[:70]))
for dist, ang, p, name, asset in sorted(rows):
    print("dist=%7.2f  ang=%7.1f  pos=%s  name=%s  asset=%s" % (dist, ang, [round(x, 2) for x in p], name, asset))
print("---- spawn spheres ----")
for fn, r in items:
    if r.get('class') == 'SpawnSphere':
        print("spawn", r.get('name'), r.get('position'))
