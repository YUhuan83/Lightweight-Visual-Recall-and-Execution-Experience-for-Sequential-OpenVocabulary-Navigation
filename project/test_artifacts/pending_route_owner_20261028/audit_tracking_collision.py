"""Closed v58: distinguish old path, actual tracking and command arc."""
import json,sqlite3,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers,distance
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
trial=Path('/home/ly/execution_continuity_review_20261022/v58_confirmed_raw_handoff/full/trial_20261028_094705')
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
def before(n,t):
 i,c=topics[n];r=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,t)).fetchone()
 return r[0],deserialize_message(r[1],c)
out=[]
for stamp in (1793152166544698619,1793152203433633330):
 ot,o=before('/spot1/odom',stamp);mt,m=before('/local_costmap/costmap',stamp);gt,g=before('/global_costmap/costmap',stamp)
 pt,p=before('/wildos/controller_path_progress',stamp);at,a=before('/lookahead_collision_arc',stamp)
 source_ns=p.header.stamp.sec*10**9+p.header.stamp.nanosec
 planned=next((e for e in events if e.get('status')=='planned' and e.get('path_source_stamp_ns')==source_ns),None)
 dispatch_maps={}
 if not planned:
  i,c=topics['/wildos/planned_path']
  for rt,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc',(i,stamp)):
   pp=deserialize_message(b,c)
   if pp.header.stamp.sec*10**9+pp.header.stamp.nanosec==source_ns:
    for name in ('local','global'):dispatch_maps[name]=before('/'+name+'_costmap/costmap',rt)[1]
    break
 if planned:
  for name in ('local','global'):
   expected=planned.get('replay_inputs',{}).get(name+'_costmap_stamp_ns')
   topic='/'+name+'_costmap/costmap';i,c=topics[topic]
   for rt,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 30',(i,planned['stamp_ns'])):
    mm=deserialize_message(b,c)
    if mm.header.stamp.sec*10**9+mm.header.stamp.nanosec==expected:
     dispatch_maps[name]=mm;break
 start=xy(o.pose.pose);near=min(range(len(p.poses)),key=lambda i:distance(start,xy(p.poses[i].pose)))
 checks=[]
 for i,pose in enumerate(p.poses[near:]):
  if distance(start,xy(pose.pose))>4:break
  b=footprint_blockers(m,xy(pose.pose),yaw(pose.pose),limit=3)
  gb=footprint_blockers(g,xy(pose.pose),yaw(pose.pose),limit=3)
  if b or gb: checks.append(dict(index=near+i,pose=[*xy(pose.pose),yaw(pose.pose)],local=b,global_map=gb,at_dispatch={k:footprint_blockers(v,xy(pose.pose),yaw(pose.pose),limit=3) for k,v in dispatch_maps.items()}))
 out.append(dict(stamp_ns=stamp,progress_age_s=(stamp-pt)*1e-9,progress_header_ns=p.header.stamp.sec*10**9+p.header.stamp.nanosec,odom_header_ns=o.header.stamp.sec*10**9+o.header.stamp.nanosec,vehicle=[*start,yaw(o.pose.pose)],speed=[o.twist.twist.linear.x,o.twist.twist.angular.z],nearest_path_pose=[*xy(p.poses[near].pose),yaw(p.poses[near].pose)],path_lateral_gap_m=distance(start,xy(p.poses[near].pose)),path_heading_gap_rad=math.remainder(yaw(o.pose.pose)-yaw(p.poses[near].pose),2*math.pi),path_blocked_within4m=checks[:6],arc_to_path_max_m=max(min(distance(xy(v.pose),xy(w.pose)) for w in p.poses) for v in a.poses)))
db.close();(trial/'tracking_collision_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
