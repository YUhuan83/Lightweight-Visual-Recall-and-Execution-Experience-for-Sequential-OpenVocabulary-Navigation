import collections,hashlib,json,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
out=[]
for trial in sorted((root/'full').glob('trial_*')):
    goal=next(trial.glob('sessions/*/goal_001'));db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    result=dict(trial=str(trial),topics={})
    for name in ('/spot1/object_mask','/spot1/ouster/front/points_filtered','/tf'):
        tid,typ=topics[name];cls=get_message(typ);items=collections.defaultdict(list)
        for bt,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
            m=deserialize_message(blob,cls)
            if name=='/tf':
                for tr in m.transforms:items[(tr.child_frame_id,stamp(tr))].append((bt,str(tr.transform)))
            else:items[stamp(m)].append((bt,hashlib.sha256(blob).hexdigest()))
        duplicates=[dict(key=k,count=len(v),distinct=len(set(s for _,s in v)),recorded_span_s=(v[-1][0]-v[0][0])/1e9) for k,v in items.items() if len(v)>1]
        result['topics'][name]=dict(unique=len(items),duplicate_groups=len(duplicates),differing_groups=sum(r['distinct']>1 for r in duplicates),examples=duplicates[:8])
    db.close();out.append(result)
(root/'sensor_identity_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
