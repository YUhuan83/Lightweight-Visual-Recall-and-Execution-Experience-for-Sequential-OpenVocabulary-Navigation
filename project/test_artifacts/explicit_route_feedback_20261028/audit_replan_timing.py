"""Closed recording: when does the currently executing path first become blocked?"""
import json,sqlite3,bisect
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.graph_route import validate_local_prefix
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
trial=Path('/home/ly/execution_continuity_review_20261022/v59_map_update_replan/full/trial_20261028_214226')
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
 i,c=topics[name];return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
od=rows('/spot1/odom');paths=rows('/wildos/controller_path_progress');maps=rows('/global_costmap/costmap')
db.close();pt=[t for t,_ in paths];mt=[t for t,_ in maps]
out=[]
for end in (1793195095504013576,1793195121584301567):
 samples=[]
 for t,o in od:
  if not end-10_000_000_000<=t<=end:continue
  pi=bisect.bisect_right(pt,t)-1;mi=bisect.bisect_right(mt,t)-1
  if pi<0 or mi<0 or t-pt[pi]>.5e9:continue
  path=trim_moving_path(paths[pi][1],xy(o.pose.pose),yaw(o.pose.pose))
  if path is None:continue
  ps=[xy(p.pose) for p in path.poses];hs=[yaw(p.pose) for p in path.poses]
  reasons={str(d):validate_local_prefix(ps,hs,maps[mi][1],xy(o.pose.pose),yaw(o.pose.pose),maximum=d) for d in (2.,8.)}
  samples.append(dict(before_collision_s=(end-t)*1e-9,reasons=reasons,map_age_s=(t-mt[mi])*1e-9,path_stamp_ns=path.header.stamp.sec*10**9+path.header.stamp.nanosec))
 out.append(dict(collision_ns=end,first_blocked={str(d):next((s for s in samples if s['reasons'][str(d)]),None) for d in (2.,8.)},samples=samples))
(trial/'replan_timing_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps([{k:v for k,v in x.items() if k!='samples'} for x in out],indent=2))
