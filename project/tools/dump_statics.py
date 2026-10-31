#!/usr/bin/env python3
import json, math, zipfile
ZIP = '/home/ly/beamng-maps/nizhen_countryside_standalone.zip'
items = []
with zipfile.ZipFile(ZIP) as z:
    for f in z.infolist():
        if f.filename.endswith('items.level.json'):
            for line in z.read(f).decode('utf-8-sig').splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    items.append(json.loads(line))
                except Exception:
                    pass
for r in items:
    if r.get('class') == 'TSStatic':
        p = r.get('position')
        if isinstance(p, list) and len(p) >= 3:
            print("%-40s pos=%s" % (str(r.get('name'))[:40], [round(x, 2) for x in p]))
