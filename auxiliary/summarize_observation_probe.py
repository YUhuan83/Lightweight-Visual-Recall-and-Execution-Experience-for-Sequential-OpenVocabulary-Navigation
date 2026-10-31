"""Validate two completed native replay runs; diagnostic only."""
import hashlib
import json
import math
from pathlib import Path

root=Path(__file__).parent.parent/'拟真/test_artifacts/integrated_progress_20261025'
folder=root/'observation_pose_probe'
manifest=json.loads((folder/'manifest.json').read_text(encoding='utf8'))
cases=[]
for item in manifest:
    name=Path(item['file']).name
    runs=[]
    for repeat in (1,2):
        rows=[]
        for line in (folder/f'{name}.run_{repeat}.log').read_text(encoding='utf8').splitlines():
            if not line.startswith('OBS_RESULT '): continue
            # Error reason contains spaces; compare the complete result except
            # timing, which varies with machine load and is not a path identity.
            rows.append(line.rsplit(' elapsed=',1)[0])
        assert len(rows)==len(item['labels']), (name,repeat)
        runs.append(rows)
    assert runs[0]==runs[1],name
    x,y,yaw=item['start'];gx,gy,_=item['goals'][0]
    dx=gx-x;dy=gy-y
    lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
    radius=(dx*dx+dy*dy)/(2*abs(lateral)) if abs(lateral)>1e-12 else None
    cases.append(dict(file=name,request_serial=item['request_serial'],start=item['start'],goals=item['goals'],labels=item['labels'],forward_single_circle_radius_m=radius,both_runs_same_path_outcome=True,results=runs[0],input_sha256=hashlib.sha256((folder/name).read_bytes()).hexdigest()))
report=dict(scope='Real Smac plugin, recorded requested poses and latest preceding raw public costmap. No vehicle/action execution, private locked field map not reconstructed.',planner_radius_m=12.,cases=cases)
(root/'observation_probe_summary.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
