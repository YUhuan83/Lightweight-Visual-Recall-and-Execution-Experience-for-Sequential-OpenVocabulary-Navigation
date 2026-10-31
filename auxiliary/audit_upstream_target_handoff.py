"""CLOSED v57: original raw goal -> retained GraphNav -> actual vehicle path."""
import json,sqlite3,math,collections
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach')
r=json.loads((root/'field_result.json').read_text())
assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
goal=next(Path(r['trial']).glob('sessions/*/goal_001'))
events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
q=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
offset=next(e['stamp_ns']*1e-9-(e['elapsed']-q) for e in events if e.get('stamp_ns'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(n):
    i,c=topics[n]
    return [(t,deserialize_message(data,c)) for t,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def xyz(p):return [p.x,p.y,p.z]
raw=[dict(stamp_ns=ns(m),query_s=ns(m)*1e-9-offset,position=xyz(m.pose.position)) for _,m in rows('/spot1/raw_object_waypoint') if ns(m)*1e-9-offset>=0]
statuses={d['stamp_ns']:d for _,m in rows('/spot1/graphnav_planner/route_status') for d in [json.loads(m.data)]}
paths=[]
for _,m in rows('/spot1/graphnav_planner/path'):
    stamp=ns(m);s=statuses.get(stamp)
    if not m.poses or not s:continue
    matching=min(raw,key=lambda v:math.dist(v['position'][:2],s['goal']),default=None)
    if matching is None or math.dist(matching['position'][:2],s['goal'])>1e-6:continue
    a,b=xyz(m.poses[0].pose.position),xyz(m.poses[-1].pose.position)
    paths.append(dict(query_s=stamp*1e-9-offset,status=s,start=a,endpoint=b,
        start_goal_gap_m=math.dist(a[:2],s['goal']),endpoint_goal_gap_m=math.dist(b[:2],s['goal'])))
accepted=[]
for e in events:
    if e.get('status')!='awaiting_graph_route' or not e.get('goal'):continue
    matching=min(raw,key=lambda v:math.dist(v['position'][:2],e['goal']),default=None)
    if matching and math.dist(matching['position'][:2],e['goal'])<1e-6:
        accepted.append(dict(query_s=e['elapsed']-q,goal=e['goal'],raw_stamp_ns=matching['stamp_ns']))
db.close()
counts=collections.Counter(e.get('status') for e in events)
report=dict(scope=__doc__,raw_goals=raw,accepted_raw_destinations=accepted,
    route_modes=dict(collections.Counter(p['status']['mode'] for p in paths)),
    matching_graph_paths=len(paths),farther_endpoints=sum(p['endpoint_goal_gap_m']>p['start_goal_gap_m'] for p in paths),
    added_approach_status_counts={k:counts[k] for k in ('object_standoff_approach','confirmed_standoff_update_deferred','visible_target_approach','awaiting_visual_confirmation','target_instance_released')},
    graph_paths=paths,interface=json.loads((root/'actual_approach_interface.json').read_text()),
    passed=False,note='Euclidean farther endpoint is an observation, not proof every detour is wrong. No full arrival.')
(root/'upstream_handoff_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(raw_goals=len(raw),accepted=accepted,route_modes=report['route_modes'],matching_graph_paths=len(paths),farther_endpoints=report['farther_endpoints'],added_approach_status_counts=report['added_approach_status_counts'],example_farther=next((p for p in paths if p['endpoint_goal_gap_m']>p['start_goal_gap_m']),None)),indent=2))
