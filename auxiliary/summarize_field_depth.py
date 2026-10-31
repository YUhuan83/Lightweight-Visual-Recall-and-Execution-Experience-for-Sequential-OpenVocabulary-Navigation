import json,hashlib
from pathlib import Path
p=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\depth_geometry_20261027\field')
a=p/'field_depth_replay_1.json';b=p/'field_depth_replay_2.json';assert a.read_bytes()==b.read_bytes()
r=json.loads(a.read_text(encoding='utf8'));print('SHA',hashlib.sha256(a.read_bytes()).hexdigest())
for t in r['rounds']:
 print({k:v for k,v in t.items() if k not in ('rows','recorded_points')})
 for item in t['recorded_points']:
  d=item['data'];print(json.dumps({k:d.get(k) for k in ('observation_stamp_ns','position','source','radius','quality')}))
 print('replayed_valid',[(x['stamp_ns'],x.get('position_error_m')) for x in t['rows'] if x['result']=='lidar'])
