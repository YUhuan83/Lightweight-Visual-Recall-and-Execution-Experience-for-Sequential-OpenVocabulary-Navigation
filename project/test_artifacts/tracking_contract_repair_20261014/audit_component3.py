import json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'official_adapter'),str(ROOT/'nav2_adapter')]
import numpy as np
from types import SimpleNamespace
from beamng_wildos_adapter.geometry import body_rotation,shifted_body_position
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.tracking_preview import preview_tracking
out=Path(__file__).parent/'physical_cusp_component3'
report=json.loads((out/'result.json').read_text())
events=report['events'];failed=next(e for e in events if e.get('status')=='follow_failed')
at=failed['stamp_ns']/1e9
planned=[e for e in events if e.get('kind')=='planned_path'][-1]
admission=[e for e in events if e.get('status')=='planned'][-1]
goal=planned['points'][-1];rows=[]
for line in (out/'native_samples.jsonl').read_text().splitlines():
    row=json.loads(line);t=row['wall_time']
    if at-7<=t<=at+.15:
        vector=lambda v:SimpleNamespace(x=v[0],y=v[1],z=v[2])
        r=body_rotation(vector(row['direction']),vector(row['up']))
        p=shifted_body_position(np.array(row['position']),r,np.array(SBR_BASE_OFFSET))
        yaw=math.atan2(r[1,0],r[0,0]);dx,dy=goal[0]-p[0],goal[1]-p[1]
        rows.append(dict(time_to_abort=t-at,pose=[p[0],p[1],yaw],
            gap=math.hypot(dx,dy),body_endpoint=[dx*math.cos(yaw)+dy*math.sin(yaw),
            -dx*math.sin(yaw)+dy*math.cos(yaw)],speed=row['speed'],
            steering=row['sent']['steering'],brake=row['sent']['brake'],
            throttle=row['sent']['throttle'],age=row['telemetry_age_sec']))
preview=preview_tracking(planned['points'],planned['headings'],
    admission['actual_start_pose'][:2],admission['actual_start_pose'][2],8.)
summary={'passed':False,'scope':'Native rear-axle reconstruction is not exact odom/TF replay',
    'abort_time':at,'goal':goal,'admission':admission,'preview_reached':preview.reached,
    'preview_closest_m':preview.closest_m,'native_closest_m':min(r['gap'] for r in rows),
    'near_abort':rows[-8:],'rows':rows,'publishers':report['publishers_before_arm']}
(out/'diagnostic_review.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
