"""Read closed arc-trial bag; quantify raw goals, arcs, odom and status."""
import collections,json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
trial=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/full/trial_20261028_093504')
result=json.loads((trial/'result.json').read_text())
assert result['closed']
goal=next(trial.glob('sessions/*/goal_001'))
ev=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
q=next(x['elapsed'] for x in ev if x.get('kind')=='query_sent')
first=next(x for x in ev if x.get('stamp_ns'))
origin=first['stamp_ns']-round((first['elapsed']-q)*1e9)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(n):
 i,c=topics[n]
 return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,origin,origin+180_000_000_000))]
odom=rows('/spot1/odom');raw=rows('/spot1/raw_object_waypoint');arc=rows('/lookahead_collision_arc')
db.close()
pos=[m.pose.pose.position for _,m in odom]
total=sum(math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(pos,pos[1:])) if len(pos)>1 else None
net=math.hypot(pos[-1].x-pos[0].x,pos[-1].y-pos[0].y) if len(pos)>1 else None
goals=[{'query_s':(t-origin)*1e-9,'xyz':[m.pose.position.x,m.pose.position.y,m.pose.position.z]} for t,m in raw]
arcs=[{'query_s':(t-origin)*1e-9,'frame':m.header.frame_id,'poses':len(m.poses)} for t,m in arc]
report={'scope':'closed bag; odom interval sampled at bag receipt; no semantic ground truth','trial':str(trial),'odom_samples':len(odom),'odom_net_m':net,'odom_total_sampled_m':total,'raw_goals':goals,'arc_messages':len(arcs),'nonempty_arc_messages':sum(x['poses']>0 for x in arcs),'arc_frames':dict(collections.Counter(x['frame'] for x in arcs)),'status_counts':dict(collections.Counter(x['status'] for x in ev if x.get('kind')=='plan_status')),'failure_events':[x for x in ev if x.get('status') in ('follow_failed','continuous_motion_failed')], 'target_rejections':dict(collections.Counter(x.get('reason') for x in ev if x.get('status')=='target_evidence_rejected')),'accepted_goals':[x for x in ev if x.get('status')=='awaiting_graph_route' and x.get('goal')]}
(trial/'arc_trial_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('raw_goals','failure_events','accepted_goals')},indent=2))
print('raw_goals',len(goals),'first',goals[:2],'accepted',report['accepted_goals'])
