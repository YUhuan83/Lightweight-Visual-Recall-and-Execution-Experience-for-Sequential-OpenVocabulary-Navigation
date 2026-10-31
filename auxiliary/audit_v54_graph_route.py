"""Read closed v54 graph/path ownership and endpoint selection."""
import json
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/execution_continuity_review_20261022/v54_confirmed_graph_handoff')
receipt=json.loads((root/'field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
for n,(i,t) in topics.items():
    if any(k in n for k in ('graph','route','goal','path')):
        count=db.execute('select count(*) from messages where topic_id=?',(i,)).fetchone()[0]
        print(n,t,count)
        if 'Graph' in t and count:
            m=deserialize_message(db.execute('select data from messages where topic_id=? order by timestamp limit 1',(i,)).fetchone()[0],get_message(t))
            print('FIELDS',m.get_fields_and_field_types())
events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
q=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
offset=next(e['stamp_ns']*1e-9-(e['elapsed']-q) for e in events if e.get('stamp_ns'))
def messages(name):
    i,t=topics[name]
    return [(stamp,deserialize_message(data,get_message(t))) for stamp,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
statuses={d['stamp_ns']:d for _,m in messages('/spot1/graphnav_planner/route_status') for d in [json.loads(m.data)]}
out=[];previous=None
for _,m in messages('/spot1/graphnav_planner/path'):
    stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    t=stamp*1e-9-offset
    if t<58:continue
    points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]
    s=statuses.get(stamp)
    row=dict(query_s=t,stamp_ns=stamp,status=s,points=points)
    out.append(row)
    key=([round(v,1) for v in points[-1]] if points else [], s.get('goal') if s else None)
    if key!=previous:
        print(json.dumps(dict(time=round(t,3),status=s,start=points[0] if points else [],end=points[-1] if points else [],n=len(points))))
        previous=key
(root/'graph_route_audit.json').write_text(json.dumps(out,indent=2))
db.close()
