"""Closed v49 geometry/transport window at deferred confirmed updates.

Action acknowledgement fields are controlled to match recorded following state;
no action server or counterfactual vehicle motion is simulated.
"""
import bisect,json,sqlite3,sys,time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import rclpy
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.input_quality import stamp_ns
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v49_localization_range'
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
g=next(Path(r['trial']).glob('sessions/*/goal_001'));db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def load(n):
 i,c=topics[n];return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
needed=('/spot1/odom','/global_costmap/costmap','/local_costmap/costmap','/wildos/controller_path_progress','/wildos/planned_path','/wildos/native_control_status','/wildos/control_status','/spot1/object_confirmation')
data={n:load(n) for n in needed};db.close();indices={n:[t for t,_ in rows] for n,rows in data.items()}
confirmations=[(t,json.loads(m.data)) for t,m in data['/spot1/object_confirmation']]
def last(n,t):
 i=bisect.bisect_right(indices[n],t)-1
 return data[n][i] if i>=0 else (None,None)
full={stamp_ns(m):m for _,m in data['/wildos/planned_path']}
events=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines()];q=next(e['elapsed'] for e in events if e['kind']=='query_sent')
selected=[]
for target in (125.43,129.2,172.55,174.49,176.43):
 candidates=[e for e in events if e.get('status')=='confirmed_standoff_update_deferred' and e.get('reason')=='accepted_route_active']
 selected.append(min(candidates,key=lambda e:abs(e['elapsed']-q-target)))
rclpy.init();n=ContinuousGraphPathBridge();reports=[]
try:
 n.full_path_execution=True;n.carrier_model='ackermann';n.minimum_turning_radius=8.;n.follow_enabled=True;n.require_source_control=True
 for e in selected:
  now=e['stamp_ns'];seconds=now*1e-9
  with patch('time.monotonic',lambda:seconds),patch('time.time',lambda:seconds),patch('time.time_ns',lambda:now):
   n.get_clock=lambda:S(now=lambda:S(nanoseconds=now,to_msg=lambda:Time(sec=now//10**9,nanosec=now%10**9)))
   n.odom=last('/spot1/odom',now)[1];n.grid=last('/global_costmap/costmap',now)[1];n.local_grid=last('/local_costmap/costmap',now)[1]
   n.odom_received_at=stamp_ns(n.odom)*1e-9;n.grid_received_at=stamp_ns(n.grid)*1e-9;n.local_grid_received_at=stamp_ns(n.local_grid)*1e-9
   n.native_status=n.control_status=None;n.native_status_at=n.control_status_at=None
   nt,nm=last('/wildos/native_control_status',now);ct,cm=last('/wildos/control_status',now)
   if nm:n.on_native_status(nm)
   if cm:n.on_control_status(cm)
   pt,path=last('/wildos/controller_path_progress',now);identity=stamp_ns(path);accepted=full.get(identity)
   n._controller_path=path;n._controller_path_received_at=pt*1e-9
   n.following=True;n.active_follow_serial=17;n.follow_active=dict(serial=17,path_stamp_ns=identity)
   n.follow_handle=Mock();n.motion_queue=[accepted] if accepted else [];n.follow_request=None;n.replacement_pending=False
   window=n.active_window()
   row=dict(query_s=e['elapsed']-q,now_ns=now,old_goal=e['goal'],proposed_goal=e['proposed_goal'],odom=[*xy(n.odom.pose.pose),yaw(n.odom.pose.pose)],path_identity=identity,accepted_recorded=accepted is not None,
       progress_age_s=(now-stamp_ns(path.poses[0]))*1e-9,receipt_age_s=seconds-pt*1e-9,input_issue=n.input_freshness_issue(),local_issue=n.local_freshness_issue(),transport_issue=n.execution_transport_issue(),window_available=window is not None)
   if window:row.update(forward=window[1],remaining_m=window[2])
   from wildos_nav2_adapter.graph_route import distance
   candidates=[(t,d) for t,d in confirmations if d.get('standoff') and distance(d['standoff'],e['proposed_goal'])<1e-6 and d['observation_stamp_ns']<=now and t<=now+50_000_000]
   if candidates:
    _,proposal=max(candidates,key=lambda p:p[0])
    initial=next(x for x in events if x.get('status')=='object_standoff_approach')
    n.query=proposal['query'];n.route=n.latest_route=accepted
    n.expected=n.confirmed_route_goal=tuple(e['goal'])
    n.confirmed_route_object=tuple(initial['object_position'][:2]);n.confirmed_route_radius=float(proposal['radius'])
    n.confirmed_route_started_at=seconds-1
    n.active_object_confirmation=dict(query=n.query,instance_id=proposal.get('instance_id',''))
    n.relay_destination=Mock();before=n.epoch;handle=n.follow_handle
    n.route_confirmed_target(proposal,xy(n.odom.pose.pose))
    row.update(proposal_found=True,route_updated=n.expected==tuple(e['proposed_goal']),epoch_unchanged=n.epoch==before,accepted_action_retained=n.follow_handle is handle,object_found=n.object_found)
   else:row['proposal_found']=False
   reports.append(row)
finally:n.destroy_node();rclpy.shutdown()
out=dict(scope=__doc__,reports=reports);Path(sys.argv[1] if len(sys.argv)>1 else root/'confirmed_window_audit.json').write_text(json.dumps(out,indent=2,sort_keys=True));print(json.dumps(out,indent=2))
