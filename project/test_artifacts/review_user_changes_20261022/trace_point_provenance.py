"""Trace recorded marking/clearing evidence at the earlier blocked world cells."""
from pathlib import Path
import bisect,collections,json,sqlite3,sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.pointcloud import decode_xyz

trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
nearby='--nearby' in sys.argv
bounds=(736.5,738.25,-27.25,-26.) if nearby else (737.,737.75,-26.75,-26.5)
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
event=next(json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines() if json.loads(s).get('kind')=='plan_status')
origin=event['stamp_ns']/1e9-event['elapsed']
def rows(name):
    i,t=topics[name]
    for stamp,raw in db.execute('select timestamp,data from messages where topic_id=? and timestamp<? order by timestamp',(i,int((origin+40)*1e9))):
        yield stamp,deserialize_message(raw,t)
def source_stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
history=collections.defaultdict(list)
for name in ['/tf','/tf_static']:
    if name not in topics:continue
    for _,m in rows(name):
        for tr in m.transforms:
            q=tr.transform.rotation;t=tr.transform.translation
            T=np.eye(4);T[:3,:3]=matrix_from_quaternion([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z]
            history[tr.child_frame_id].append((0 if name=='/tf_static' else source_stamp(tr),tr.header.frame_id,T))
for entries in history.values():entries.sort(key=lambda a:a[0])
def world(frame,ns):
    T=np.eye(4);ages=[];seen=set()
    while frame!='spot1/odom':
        if frame in seen or frame not in history:return None,None
        seen.add(frame);entries=history[frame];j=bisect.bisect_right([e[0] for e in entries],ns)-1
        if j<0:return None,None
        ts,parent,A=entries[j];T=A@T
        if ts:ages.append((ns-ts)/1e9)
        frame=parent
    return T,max(ages,default=0.)

report={'trial':str(trial),'origin':origin,'topic_counts':{},'evidence':[],'map_transitions':[],'bounds':bounds,'scope':'first 40s; nearest preceding source TF; includes voxel-neighbor region only when --nearby'}
for name in ['/spot1/ouster/front/points_filtered','/wildos/points','/wildos/clearing_points']:
    if name not in topics:continue
    count=missing=0
    for stamp,m in rows(name):
        count+=1;ns=source_stamp(m);T,age=world(m.header.frame_id,ns)
        if T is None:missing+=1;continue
        p=decode_xyz(m);w=p@T[:3,:3].T+T[:3,3]
        hits=(w[:,0]>=bounds[0])&(w[:,0]<bounds[1])&(w[:,1]>=bounds[2])&(w[:,1]<bounds[3])
        if not hits.any():continue
        base,base_age=world('spot1/base_link',ns)
        body=(w[hits]-base[:3,3])@base[:3,:3] if base is not None else np.empty((0,3))
        report['evidence'].append(dict(topic=name,elapsed=(stamp/1e9-origin),source_elapsed=ns/1e9-origin,tf_age=age,points=int(hits.sum()),world_z=[float(w[hits,2].min()),float(w[hits,2].max())],body_z=[float(body[:,2].min()),float(body[:,2].max())] if len(body) else None,body_range=[float(np.linalg.norm(body[:,:2],axis=1).min()),float(np.linalg.norm(body[:,:2],axis=1).max())] if len(body) else None,world_points=w[hits].tolist()))
    report['topic_counts'][name]=dict(messages=count,missing_tf=missing)
prev=None
for stamp,m in rows('/global_costmap/costmap'):
    vals=[]
    for x in [737.125,737.375,737.625]:
        mx=int(np.floor((x-m.info.origin.position.x)/m.info.resolution));my=int(np.floor((-26.625-m.info.origin.position.y)/m.info.resolution))
        vals.append(int(m.data[my*m.info.width+mx]) if 0<=mx<m.info.width and 0<=my<m.info.height else None)
    if vals!=prev:report['map_transitions'].append(dict(elapsed=stamp/1e9-origin,values=vals));prev=vals
out=trial/('point_provenance_nearby.json' if nearby else 'point_provenance.json');out.write_text(json.dumps(report,indent=2))
print(json.dumps(dict(trial=str(trial),topic_counts=report['topic_counts'],hits=collections.Counter(e['topic'] for e in report['evidence']),map_transitions=report['map_transitions'],first_hits=[{k:v for k,v in e.items() if k!='world_points'} for e in report['evidence'][:10]]),indent=2))
