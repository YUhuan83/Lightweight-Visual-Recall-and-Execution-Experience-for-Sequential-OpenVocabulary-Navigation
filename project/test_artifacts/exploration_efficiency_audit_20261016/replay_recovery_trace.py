from pathlib import Path
from collections import deque
from types import SimpleNamespace as S
import sqlite3,json,math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import wildos_nav2_adapter.execution_recovery as recovery
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.graph_route import validate_vehicle_path,validate_local_prefix
s=Path('/home/ly/wildos-console/audit_results/session_20261016_095740');r=Path('/home/ly/exploration_efficiency_audit_20261016')
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
events=[json.loads(l) for l in (s/'goal_001/events.jsonl').read_text().splitlines()]
failures=[x for x in events if x.get('status')=='no_vehicle_path'];cutoff=failures[1]['stamp_ns']
c=sqlite3.connect(next((s/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={i:(name,get_message(typ)) for i,name,typ in c.execute('select id,name,type from topics')}
owner=S(recovering=False,following=False,control_status_at=None,control_status={},escape_trace=deque(maxlen=96))
clock=[0.];recovery.time=S(monotonic=lambda:clock[0]);odom=None
names=('/spot1/odom','/wildos/control_status','/wildos/plan_status');ids=[i for i,(n,_) in topics.items() if n in names]
for ts,tid,blob in c.execute('select timestamp,topic_id,data from messages where timestamp<=? and topic_id in ('+','.join('?' for _ in ids)+') order by timestamp',[cutoff,*ids]):
 clock[0]=ts/1e9;name,kind=topics[tid];m=deserialize_message(blob,kind)
 if name=='/wildos/control_status':owner.control_status=json.loads(m.data);owner.control_status_at=clock[0]
 elif name=='/spot1/odom':odom=m;recovery.ExecutionRecovery.remember_driven_pose(owner,m)
 else:
  state=json.loads(m.data).get('status')
  if state=='following':owner.following=True
  elif state in ('follow_failed','segment_done','route_done','exploration_retired','no_vehicle_path','planning_only'):owner.following=False
assert odom is not None
poses=recovery.reverse_escape(owner.escape_trace,xy(odom.pose.pose),yaw(odom.pose.pose),cutoff/1e9,8.)
report=dict(scope='Reconstruct forward driven corridor from recorded odometry/control/plan messages, preserving physical trace across frontier retirement. Uses bag receipt order/times, not exact private callback trace.',cutoff_ns=cutoff,trace_points=len(owner.escape_trace),trace=[list(x) for x in owner.escape_trace],start=[*xy(odom.pose.pose),yaw(odom.pose.pose)],escape_poses=poses)
if poses:
 def grid(topic):
  tid,kind=next((i,t) for i,(n,t) in topics.items() if n==topic);ts,b=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,cutoff)).fetchone();return ts,deserialize_message(b,kind)
 tg,g=grid('/global_costmap/costmap');tl,l=grid('/local_costmap/costmap');points=[p[:2] for p in poses];headings=[p[2] for p in poses]
 report.update(global_validation=validate_vehicle_path(points,headings,points,g,points[-1],start=points[0],minimum_turning_radius=8.,heading_quantization_bins=72),local_validation=validate_local_prefix(points,headings,l,points[0],headings[0]),global_map_received_ns=tg,local_map_received_ns=tl,escape_length_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])))
(r/'recovery_trace_review.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('escape_poses','trace')},indent=2))
