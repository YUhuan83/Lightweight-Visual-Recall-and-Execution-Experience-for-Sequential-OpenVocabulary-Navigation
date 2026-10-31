from pathlib import Path
import json,sqlite3,math,collections,bisect
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.motion_path import motion_sections
s=Path('/home/ly/wildos-console/audit_results/session_20261016_085025');assert (s/'session_result.json').exists()
a=[json.loads(l) for l in (s/'vision_participation.jsonl').read_text().splitlines()];scores=[v for v in a if v['kind']=='scores']
e=[json.loads(l) for l in (s/'goal_001/events.jsonl').read_text().splitlines()];following=[v for v in e if v.get('status')=='following']
def connect(p):
 c=sqlite3.connect(next(p.glob('*.db3')).as_uri()+'?mode=ro',uri=True)
 topics={n:(i,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')};return c,topics
def load(c,topics,name):
 i,typ=topics[name];return [(t,deserialize_message(b,typ)) for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
c,tp=connect(s/'goal_001/interfaces');odom=load(c,tp,'/spot1/odom');ots=[v[0] for v in odom]
records=[]
for ns,msg in load(c,tp,'/spot1/graphnav_planner/route_status'):
 d=json.loads(msg.data);goal=d['goal'];near=[(abs(v['wall']-ns/1e9),v,n) for v in scores if abs(v['wall']-ns/1e9)<2.5 for n in v['candidates'] if math.dist(n['xy'],goal)<1e-4];near.sort(key=lambda v:v[0]);candidate=near[0][2] if near else None
 idx=bisect.bisect_right(ots,ns)-1;o=odom[max(0,idx)][1];pos=o.pose.pose.position;q=o.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));h=d.get('observation_heading');err=abs(math.remainder(yaw-h,2*math.pi)) if h is not None else None
 records.append(dict(received_ns=ns,goal=goal,mode=d['mode'],complete=d['complete'],heading=h,body_position=[pos.x,pos.y],body_distance_m=math.hypot(pos.x-goal[0],pos.y-goal[1]),heading_error_rad=err,speed_mps=math.hypot(o.twist.twist.linear.x,o.twist.twist.linear.y),candidate=candidate,candidate_snapshot_offset_sec=near[0][0] if near else None))
c.close();sc,st=connect(s/'sensor_interfaces');full=[]
for ns,path in load(sc,st,'/wildos/full_planned_path'):
 pts=[(v.pose.position.x,v.pose.position.y) for v in path.poses];sections=motion_sections(pts);later=[v for v in following if v['stamp_ns']>=ns];n=later[0] if later else None
 first=pts[sections[0][1]] if sections else None;executed=bool(n and first and (n['stamp_ns']-ns)<3e9 and math.dist(first,n['goal'])<.25)
 full.append(dict(received_ns=ns,poses=[[v.pose.position.x,v.pose.position.y,2*math.atan2(v.pose.orientation.z,v.pose.orientation.w)] for v in path.poses],length_m=sum(math.dist(a,b) for a,b in zip(pts,pts[1:])),displacement_m=math.dist(pts[0],pts[-1]),sections=len(sections),first_section_endpoint=first,next_following=n,execution_first_section_matched=executed))
sc.close();groups=[]
for z in records:
 key=tuple(z['goal'])
 if not groups or tuple(groups[-1]['goal'])!=key:groups.append(dict(goal=z['goal'],begin_ns=z['received_ns'],end_ns=z['received_ns'],messages=0,visual_matches=0,default_matches=0,candidate_ids=set(),minimum_body_distance_m=z['body_distance_m']))
 g=groups[-1];g['end_ns']=z['received_ns'];g['messages']+=1;g['minimum_body_distance_m']=min(g['minimum_body_distance_m'],z['body_distance_m'])
 if z['candidate']:
  g['candidate_ids'].add(z['candidate']['id']);g['default_matches']+=z['candidate']['default']==[1.];g['visual_matches']+=z['candidate']['default']==[0.]
for g in groups:g['candidate_ids']=sorted(g['candidate_ids']);g['duration_sec']=(g['end_ns']-g['begin_ns'])/1e9
r=dict(scope='Closed recorded route decisions, nearest published odometry, 1Hz visual score snapshots and full candidate paths; no private planner visit/coverage state. Candidate matches use exact endpoint within2.5s, not guaranteed identical topology publication.',route_decisions=records,consecutive_goal_groups=groups,full_candidates=full,topic_counts_summary={'route_status':len(records),'full_candidates':len(full),'executed_first_sections_matched':sum(v['execution_first_section_matched'] for v in full)})
out=Path('/home/ly/search_decision_audit_20261016/closed_decision_trace.json');out.write_text(json.dumps(r,indent=2));print(json.dumps(dict(output=str(out),groups=groups,summary=r['topic_counts_summary']),indent=2))
