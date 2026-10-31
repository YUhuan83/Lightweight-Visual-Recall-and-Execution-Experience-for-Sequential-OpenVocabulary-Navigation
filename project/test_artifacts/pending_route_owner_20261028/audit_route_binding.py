"""Compare planned event's source route stamp and reported graph endpoint."""
import json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as PathMsg
trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'));ev=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
i=db.execute('select id from topics where name=?',('/spot1/graphnav_planner/path',)).fetchone()[0]
routes={}
for b, in db.execute('select data from messages where topic_id=?',(i,)):
 p=deserialize_message(b,PathMsg)
 if p.poses:routes[p.header.stamp.sec*10**9+p.header.stamp.nanosec]=[p.poses[-1].pose.position.x,p.poses[-1].pose.position.y]
db.close();cases=[]
for e in ev:
 if e.get('status')!='planned' or not e.get('graph_goal'):continue
 stamp=e.get('replay_inputs',{}).get('route_stamp_ns');endpoint=routes.get(stamp)
 if endpoint is not None:cases.append(dict(elapsed=e['elapsed'],source_stamp_ns=stamp,source_endpoint=endpoint,reported_graph_goal=e['graph_goal'],gap_m=math.dist(endpoint,e['graph_goal'])))
out=dict(trial=str(trial),matched=len(cases),mismatches=sum(c['gap_m']>1e-6 for c in cases),cases=cases)
(trial/'route_binding_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='cases'}))
