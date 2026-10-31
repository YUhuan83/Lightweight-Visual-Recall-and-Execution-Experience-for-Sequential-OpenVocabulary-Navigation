"""Replay received trace evidence, not an exact private callback-state capture."""
from pathlib import Path
from collections import deque
from types import SimpleNamespace as S
import json,math,sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import wildos_nav2_adapter.execution_recovery as recovery
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.graph_route import validate_vehicle_path,validate_local_prefix

r=Path('/home/ly/frontier_cost_repair_20261016');s=Path('/home/ly/wildos-console/audit_results/session_20261016_145143');g=s/'goal_001'
assert (s/'session_result.json').exists() and (g/'result.json').exists()
assert Path('/home/ly/wildos-console/field_frontier_cost_search_20261016_145142/result.json').exists()
events=[json.loads(x) for x in (g/'events.jsonl').read_text().splitlines()]
failures=[e for e in events if e.get('status')=='no_vehicle_path'][:4]
db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
ids=[i for i,(n,k) in topics.items() if n in ('/spot1/odom','/wildos/control_status','/wildos/plan_status')]
clock=[0.];recovery.time=S(monotonic=lambda:clock[0])
def grid(name,cutoff):
 tid,kind=next((i,k) for i,(n,k) in topics.items() if n==name)
 ts,b=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,cutoff)).fetchone()
 return deserialize_message(b,kind)
report=[]
for failure in failures:
 cutoff=failure['stamp_ns'];owner=S(recovering=False,following=False,control_status_at=None,control_status={},escape_trace=deque(maxlen=96))
 trace=deque(maxlen=96);direction=None;odom=None
 for ts,tid,b in db.execute('select timestamp,topic_id,data from messages where timestamp<=? and topic_id in ('+','.join('?' for i in ids)+') order by timestamp',[cutoff,*ids]):
  clock[0]=ts/1e9;name,kind=topics[tid];m=deserialize_message(b,kind)
  if name=='/wildos/control_status':owner.control_status=json.loads(m.data);owner.control_status_at=clock[0]
  elif name=='/wildos/plan_status':
   state=json.loads(m.data).get('status')
   if state=='following':owner.following=True
   elif state in ('follow_failed','segment_done','route_done','exploration_retired','no_vehicle_path','planning_only'):owner.following=False
  else:
   odom=m;recovery.ExecutionRecovery.remember_driven_pose(owner,m)
   if (owner.following and owner.control_status_at is not None and clock[0]-owner.control_status_at<=1.
       and owner.control_status.get('actuating') and abs(owner.control_status.get('target_speed',0.))>0.):
    point=xy(m.pose.pose);newdir=1 if owner.control_status['target_speed']>0 else -1
    if trace and math.dist(point,trace[-1][:2])<.3:continue
    if (trace and math.dist(point,trace[-1][:2])>2.) or (direction is not None and direction!=newdir):trace.clear()
    direction=newdir;trace.append((*point,yaw(m.pose.pose),clock[0]))
 now=cutoff/1e9;point=xy(odom.pose.pose);heading=yaw(odom.pose.pose)
 old=recovery.reverse_escape(owner.escape_trace,point,heading,now,8.)
 virtual=[(x,y,h+(math.pi if direction==-1 else 0.),t) for x,y,h,t in trace]
 fitted=recovery.reverse_escape(virtual,point,heading+(math.pi if direction==-1 else 0.),now,8.)
 if fitted and direction==-1:fitted=[(x,y,h-math.pi) for x,y,h in fitted]
 item=dict(stamp_ns=cutoff,identity=failure.get('exploration_id'),old_trace_points=len(owner.escape_trace),
  old_last_age_s=now-owner.escape_trace[-1][3] if owner.escape_trace else None,
  old_last_gap_m=math.dist(point,owner.escape_trace[-1][:2]) if owner.escape_trace else None,
  old_fitted=old is not None,directional_points=len(trace),last_direction=direction,directional_fitted=fitted is not None)
 global_grid,local_grid=grid('/global_costmap/costmap',cutoff),grid('/local_costmap/costmap',cutoff)
 def age(message):return now-message.header.stamp.sec-message.header.stamp.nanosec*1e-9
 item.update(global_source_age_s=age(global_grid),local_source_age_s=age(local_grid),
             odom_source_age_s=age(odom),odom_speed=math.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y),
             odom_xy=point)
 if old:
  pts=[p[:2] for p in old];hs=[p[2] for p in old]
  item.update(old_global_validation=validate_vehicle_path(pts,hs,pts,global_grid,pts[-1],start=point,minimum_turning_radius=8.,heading_quantization_bins=72),
              old_local_validation=validate_local_prefix(pts,hs,local_grid,point,heading))
 if fitted:
  points=[p[:2] for p in fitted];headings=[p[2] for p in fitted]
  item.update(global_validation=validate_vehicle_path(points,headings,points,grid('/global_costmap/costmap',cutoff),points[-1],start=point,minimum_turning_radius=8.,heading_quantization_bins=72),
   local_validation=validate_local_prefix(points,headings,grid('/local_costmap/costmap',cutoff),point,heading),escape_poses=fitted)
 report.append(item)
result=dict(scope='Bag-receipt reconstruction with synthetic monotonic times; preserving same-frame trace; approximate callback order, not private runtime state. Directional fitting is diagnostic only, production unchanged.',samples=report)
(r/'field/recovery_closed_review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
