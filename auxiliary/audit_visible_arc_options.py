"""Diagnostic only: constant-curvature alternatives on recorded vehicle maps."""
import hashlib,json,math,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
out=root.parent/'target_contract_20261025'
source=json.loads((out/'visible_pose_geometry.json').read_text());reports=[]
for record in source:
    trial=root/'full'/record['trial'];goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    maps={}
    for name in ('/global_costmap/costmap','/local_costmap/costmap'):
        tid,typ=topics[name];cls=get_message(typ)
        maps[name]={m.header.stamp.sec*10**9+m.header.stamp.nanosec:m for _,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)) for m in [deserialize_message(data,cls)]}
    rows=[]
    for r in record['rows']:
        if r.get('bearing') is None or 'pose' not in r:continue
        inp=r['event']['replay_inputs'];gs=[maps[name].get(inp[key]) for name,key in [('/global_costmap/costmap','global_costmap_stamp_ns'),('/local_costmap/costmap','local_costmap_stamp_ns')]]
        row=dict(original=r,options=[])
        if any(g is None for g in gs):row['missing_map']=True;rows.append(row);continue
        x,y,yaw=r['pose'];bearing=r['bearing'];turn=math.remainder(bearing-yaw,2*math.pi)
        for length in (8.,6.,4.,2.):
            preferred=max(-1/12,min(1/12,turn/length))
            for k in dict.fromkeys((preferred,0.,1/24,-1/24,1/12,-1/12)):
                def sample(s):
                    h=yaw+k*s
                    if abs(k)<1e-9:return x+s*math.cos(yaw),y+s*math.sin(yaw),h
                    return x+(math.sin(h)-math.sin(yaw))/k,y+(-math.cos(h)+math.cos(yaw))/k,h
                end=sample(length);view=abs(math.remainder(bearing-end[2],2*math.pi));progress=(end[0]-x)*math.cos(bearing)+(end[1]-y)*math.sin(bearing)
                if view>.6 or progress<2.:continue
                samples=[sample(length*i/math.ceil(length/.25)) for i in range(math.ceil(length/.25)+1)]
                blocked=next(((j,i) for j,g in enumerate(gs) for i,p in enumerate(samples) if footprint_blockers(g,p[:2],p[2],limit=1)),None)
                if blocked is None:row['options'].append(dict(length=length,curvature=k,pose=end,progress=progress,view_angle_deg=math.degrees(view)))
        rows.append(row)
    db.close();reports.append(dict(trial=record['trial'],rows=rows))
encoded=json.dumps(reports,sort_keys=True).encode();(out/'visible_arc_options.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=[dict(trial=r['trial'],options=[dict(stamp=x['original']['event']['stamp_ns'],original_direct_radius=x['original']['direct_arc_radius_m'],available=len(x['options']),best=max(x['options'],key=lambda p:p['progress']) if x['options'] else None,missing_map=x.get('missing_map',False)) for x in r['rows']]) for r in reports])))
