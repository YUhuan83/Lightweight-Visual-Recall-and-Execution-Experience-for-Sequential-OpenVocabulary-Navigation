"""Closed recording, actual message stamps; original versus pruned local sweep."""
import bisect,json,sqlite3,sys,inspect
from types import SimpleNamespace as S
import rclpy
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.graph_route import validate_local_prefix,footprint_blockers,distance,cell_cost
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.input_quality import stamp_ns
from wildos_nav2_adapter.observation_pose import connected_free_cells,observation_pose_clear
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v51_confirmed_action_handoff'
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
g=next(Path(r['trial']).glob('sessions/*/goal_001'));db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(n):
 i,c=topics[n];return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
odom=rows('/spot1/odom');local=rows('/local_costmap/costmap');glob=rows('/global_costmap/costmap');paths=rows('/wildos/full_planned_path');confirmations=[(t,json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')];db.close()
def exact(seq,stamp):return next(m for _,m in seq if stamp_ns(m)==stamp)
ev=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines()];q=next(x['elapsed'] for x in ev if x.get('kind')=='query_sent');out=[]
rclpy.init();producer=GoalStandoff()
for e in ev:
 if e.get('status')!='follow_failed' or e.get('detail')!='local_footprint_collision':continue
 stamps=e['replay_inputs'];o=exact(odom,stamps['odom_stamp_ns']);l=exact(local,stamps['local_costmap_stamp_ns']);gg=exact(glob,stamps['global_costmap_stamp_ns'])
 candidates=[(t,p) for t,p in paths if t<=e['stamp_ns']+50_000_000 and p.poses and distance(xy(p.poses[-1].pose),e['graph_goal'])<1.]
 t,p=max(candidates,key=lambda x:x[0]);points=[xy(v.pose) for v in p.poses];headings=[yaw(v.pose) for v in p.poses]
 start=xy(o.pose.pose);heading=yaw(o.pose.pose);trim=trim_moving_path(p,start,heading)
 rr=dict(query_s=e['elapsed']-q,path_received_delta_s=(e['stamp_ns']-t)*1e-9,path_source_ns=stamp_ns(p),start=[*start,heading],path_first=[*points[0],headings[0]],target=points[-1],start_to_first_m=distance(start,points[0]),local_start_blockers=footprint_blockers(l,start,heading),original_reason=validate_local_prefix(points,headings,l,start,heading),global_reason=validate_local_prefix(points,headings,gg,start,heading),trim_available=trim is not None)
 if trim:rr['trimmed_reason']=validate_local_prefix([xy(v.pose) for v in trim.poses],[yaw(v.pose) for v in trim.poses],l,start,heading)
 rr['original_four_m_reason']=validate_local_prefix(points,headings,l,start,heading,maximum=4.)
 proposal_t,proposal=max(((t,d) for t,d in confirmations if t<=e['stamp_ns'] and d.get('standoff') and distance(d['standoff'],e['graph_goal'])<1e-6),key=lambda x:x[0])
 po=next(m for t,m in reversed(odom) if t<=proposal_t);pg=next(m for t,m in reversed(glob) if t<=proposal_t);pl=next(m for t,m in reversed(local) if t<=proposal_t)
 point=proposal['standoff'];ph=proposal.get('standoff_navigation_heading')
 if ph is not None:
  connected=connected_free_cells(pg,xy(po.pose.pose))
  rr['proposal']=dict(received_ns=proposal_t,heading=ph,global_clear=observation_pose_clear(pg,connected,point,ph),local_blockers=footprint_blockers(pl,point,ph),local_age_s=(proposal_t-stamp_ns(pl))*1e-9)
  producer.get_clock=lambda:S(now=lambda:S(nanoseconds=proposal_t))
  producer.odom=po;producer.vehicle_grid=producer.local_vehicle_grid=producer.vehicle_connectivity=None
  producer.on_vehicle_grid(pg)
  if 'local' in inspect.signature(producer.on_vehicle_grid).parameters:producer.on_vehicle_grid(pl,local=True)
  else:producer.local_vehicle_grid=pl
  rr['producer_accepts_proposal']=producer.observation_pose_filter(target=proposal['object_position'][:2])(point,ph)
 for point,h in zip(points,headings):
  blockers=footprint_blockers(l,point,h,limit=1)
  if blockers:
   rr['first_blocked_pose']=[*point,h];rr['local_body_offsets']=blockers;rr['global_blockers_at_same_pose']=footprint_blockers(gg,point,h);break
 out.append(rr)
producer.destroy_node();rclpy.shutdown()
result=dict(scope=__doc__,cases=out);Path(sys.argv[1] if len(sys.argv)>1 else root/'local_prefix_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
