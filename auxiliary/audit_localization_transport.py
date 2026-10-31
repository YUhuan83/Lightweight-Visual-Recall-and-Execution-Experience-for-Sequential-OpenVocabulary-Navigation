"""Closed evidence: near/localization cloud bounds, identity and acquisition age."""
import json,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.pointcloud import decode_xyz
root=Path('/home/ly/execution_continuity_review_20261022/v49_localization_range');r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
g=next(Path(r['trial']).glob('sessions/*/goal_001'));db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True);topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(n):
 i,c=topics[n]
 for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):yield t,deserialize_message(b,c)
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def percentiles(values):return dict(zip(('min','p50','p95','max'),map(float,np.percentile(values,[0,50,95,100])))) if len(values) else None
streams={};maps={}
for name,limit in (('/spot1/ouster/front/points_filtered',40.),('/spot1/ouster/front/points_for_localization',80.)):
 widths=[];radii=[];beyond=0;ages=[];stamps=[];frames=set();violations=0
 for t,m in rows(name):
  p=decode_xyz(m);d=np.linalg.norm(p.astype(float),axis=1);stamps.append(ns(m));ages.append((t-ns(m))*1e-9);widths.append(len(p));frames.add(m.header.frame_id)
  if len(d):radii.append(float(d.max()));beyond+=int(np.sum(d>40.0001));violations+=int(np.sum(d>limit+.0001))
 maps[name]={s:None for s in stamps}
 streams[name]=dict(messages=len(stamps),max_points=max(widths,default=0),range_limit_m=limit,max_radius_m=max(radii,default=0.),points_beyond_40m=beyond,range_violations=violations,frames=sorted(frames),receipt_age_sec=percentiles(ages),period_sec=percentiles(np.diff(stamps)*1e-9 if len(stamps)>1 else []))
near,wide=list(maps.values());common=len(set(near)&set(wide))
loc=[json.loads(m.data) for _,m in rows('/spot1/object_localization')];valid=[d for d in loc if d.get('localization_valid')]
map_stamps=[ns(m) for _,m in rows('/spot1/local_map')];db.close()
out=dict(trial=r['trial'],streams=streams,matched_measurement_stamps=common,near_only_stamps=len(set(near)-set(wide)),localization_only_stamps=len(set(wide)-set(near)),map_period_sec=percentiles(np.diff(map_stamps)*1e-9 if len(map_stamps)>1 else []),valid_localizations=len(valid),localized_observer_ranges_m=[float(np.linalg.norm(np.asarray(d['position'])-np.asarray(d['observer_position']))) for d in valid],scope='Recorded output ranges and temporal identity. Bounded output does not alone certify terrain classification or final arrival.')
(root/'range_transport_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))

