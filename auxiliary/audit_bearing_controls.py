"""Controlled geometry probes only, never alter live sensor or navigation state."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import triangulate_bearings
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
data=json.loads((root/'bearing_geometry_audit.json').read_text())
evidence=json.loads((root/'target_approach_audit.json').read_text())
reports=[]
for report,actual in zip(data,evidence):
    target=np.asarray(actual['topics']['/spot1/object_localization']['rows'][0]['value']['position'])
    variants=[]
    for window in (5.,10.,30.):
        for fixed_point in (False,True):
            views=[];previous=None;reasons=collections.Counter();positions=[]
            for row in report['rows']:
                if not row['paired']:continue
                t=row['stamp_ns'];views=[v for v in views if 0<=t-v['stamp_ns']<=window*1e9]
                if not views:previous=None
                if row['reason']=='lidar_localized':reasons['lidar']+=1;continue
                if row['reason']=='depth_ambiguity_reset':views=[];previous=None;continue
                position=np.asarray(row['robot_position'])
                if previous is not None and np.linalg.norm(position-previous)<1.:
                    reasons['too_close']+=1;continue
                origin=np.asarray(row['camera_origin'])
                ray=(target-origin) if fixed_point else np.asarray(row['ray'])
                views.append(dict(origin=origin,ray=ray,stamp_ns=t,angular_tolerance=.04));views=views[-12:]
                previous=position
                result=triangulate_bearings(views,1.)
                reasons['pass' if result is not None else 'reject']+=1
                if result is not None:positions.append(dict(stamp_ns=t,position=result['position'].tolist(),radius=result['radius']))
            variants.append(dict(history_window_s=window,fixed_point_control=fixed_point,counts=dict(reasons),positions=positions))
    reports.append(dict(trial=report['trial'],prune_events=sum(r.get('pruned_views',0)>0 for r in report['rows']),
        pruned_views=sum(r.get('pruned_views',0) for r in report['rows']),variants=variants,
        caveat='Fixed-point rays are synthetic controls toward the first LiDAR proposal, not true target identity or permission to use old geometry. Longer history is a diagnostic, not a proposed safety-timeout change.'))
encoded=json.dumps(reports,sort_keys=True).encode();(root/'bearing_controls.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=reports)))
