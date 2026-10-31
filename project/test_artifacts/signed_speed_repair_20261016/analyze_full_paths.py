from pathlib import Path
import json,sqlite3,math,re
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.motion_path import motion_sections
s=Path('/home/ly/wildos-console/audit_results/session_20261016_085025')
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
events=[json.loads(l) for l in (s/'goal_001/events.jsonl').read_text().splitlines()]
statuses=[e for e in events if e.get('kind')=='plan_status' and e.get('stamp_ns')]
transitions=[e for e in statuses if e['status'] in ('following','segment_done','follow_failed','replanning_after_maneuver','route_done')]
requests=[float(t) for t in re.findall(r'\[([0-9]+\.[0-9]+)\] \[wildos_graph_path_bridge\]: ComputePath request sent:',(s/'stack.log').read_text())]
c=sqlite3.connect(next((s/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid,typ=c.execute("select id,type from topics where name='/wildos/full_planned_path'").fetchone();kind=get_message(typ)
records=[]
for ns,blob in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
 p=deserialize_message(blob,kind);points=[(v.pose.position.x,v.pose.position.y) for v in p.poses];sections=motion_sections(points);t=ns/1e9
 prior=[e for e in transitions if e['stamp_ns']<=ns];last=prior[-1] if prior else None
 later=[e for e in statuses if e['stamp_ns']>ns and e['status']=='following'];dispatch=later[0] if later else None
 reqs=[x for x in requests if x<=t];request=reqs[-1] if reqs else None
 completed_first=bool(last and last['status']=='segment_done' and request and request<last['stamp_ns']/1e9)
 records.append(dict(received_ns=ns,poses=len(points),sections=len(sections),length_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])),target=list(points[-1]),preceding_motion_status=last['status'] if last else None,request_started_before_segment_done=completed_first,next_following_delay_sec=(dispatch['stamp_ns']-ns)/1e9 if dispatch else None,next_following_goal=dispatch.get('goal') if dispatch else None))
c.close()
r=dict(scope='Published full candidate paths and recorded state/log ordering; not private bridge prepared_extension field or every planner request byte snapshot',count=len(records),completion_first_cases=[v for v in records if v['request_started_before_segment_done']],records=records)
(s/'full_path_handoff_review.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
