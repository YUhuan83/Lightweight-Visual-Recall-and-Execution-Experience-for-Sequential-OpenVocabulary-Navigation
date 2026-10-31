"""Inspect closed path queues against published maps at plan and handoff times."""
import json,math,sqlite3,hashlib,sys,bisect
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

root=Path('/home/ly/visible_pursuit_repair_20261017')
sys.path.insert(0,str(root/'candidate/nav2_adapter'))
from wildos_nav2_adapter.graph_route import validate_vehicle_path,cell_cost,footprint_blockers
from wildos_nav2_adapter.motion_path import motion_sections
s=root/'field_results/session_20261017_155029';g=s/'goal_001'
assert (s/'session_result.json').exists() and (g/'result.json').exists()
origin=1792223480.1329412
def connect(path):return sqlite3.connect(path.as_uri()+'?mode=ro',uri=True)
goal=connect(g/'interfaces/interfaces_0.db3');raw=connect(s/'sensor_interfaces/sensor_interfaces_0.db3')
def topic(c,name):return c.execute('select id,type from topics where name=?',(name,)).fetchone()
def records(c,name):
    i,t=topic(c,name);kind=get_message(t)
    for ts,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):
        yield ts,deserialize_message(b,kind)
def latest(c,name,ts):
    i,t=topic(c,name);row=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,int(ts))).fetchone()
    return (row[0],deserialize_message(row[1],get_message(t))) if row else (None,None)
def xy(p):return (p.position.x,p.position.y)
def yaw(p):
    q=p.orientation;return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def path(m):return [(p.pose.position.x,p.pose.position.y,yaw(p.pose)) for p in m.poses]
full=[dict(time=t,path=path(m),header=message_to_ordereddict(m.header)) for t,m in records(raw,'/wildos/full_planned_path')]
events=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines() if l]
events=[e for e in events if e.get('kind')=='plan_status']
published=[(t,path(m)) for t,m in records(goal,'/wildos/planned_path')]
dispatch=[]
for i,(t,points) in enumerate(published):
    planned=min((e for e in events if e['status']=='planned'),key=lambda e:abs(e['stamp_ns']-t))
    terminal=next((e for e in events if e['stamp_ns']>=t and e['status'] in ('maneuver_done','segment_done','follow_failed','exploration_segment_finished')),None)
    match=[]
    for f in full:
        if f['time']>t:continue
        for k,(a,b) in enumerate(motion_sections(f['path'])):
            branch=f['path'][a:b+1]
            if len(branch)==len(points) and np.max(np.abs(np.asarray(branch)-np.asarray(points)))<1e-5:
                match.append((f,k))
    chosen=max(match,key=lambda v:v[0]['time']) if match else None
    dispatch.append(dict(time=t,relative_sec=t/1e9-origin,points=points,planned=planned,terminal=terminal,
                         full_time=chosen[0]['time'] if chosen else None,section=chosen[1] if chosen else None))
cases=[]
for e in events:
    if e['status']!='replanning_after_maneuver':continue
    d=next((v for v in reversed(dispatch) if v['time']<=e['stamp_ns']),None)
    if not d or d['full_time'] is None:
        cases.append(dict(event=e,match=False));continue
    f=next(v for v in full if v['time']==d['full_time']);sections=motion_sections(f['path']);k=d['section']+1
    if k>=len(sections):
        cases.append(dict(event=e,match=False,reason='no_next_section'));continue
    a,b=sections[k];branch=f['path'][a:b+1];pts=[p[:2] for p in branch];heads=[p[2] for p in branch]
    ot,odom=latest(goal,'/spot1/odom',e['stamp_ns']);current=xy(odom.pose.pose)
    gt,grid=latest(goal,'/global_costmap/costmap',e['stamp_ns'])
    pt,plan_grid=latest(goal,'/global_costmap/costmap',f['time'])
    lt,local=latest(goal,'/local_costmap/costmap',e['stamp_ns'])
    _,route=latest(goal,'/spot1/graphnav_planner/path',e['stamp_ns']);rpts=[xy(p.pose) for p in route.poses]
    def check(m):return validate_vehicle_path(pts,heads,[current]+rpts,m,pts[-1],start=current,minimum_turning_radius=8.,heading_quantization_bins=72)
    hit=None
    for p,q in zip(pts,pts[1:]):
        n=max(1,math.ceil(math.dist(p,q)/max(grid.info.resolution,.1)))
        for i in range(n+1):
            x=p[0]+(q[0]-p[0])*i/n;y=p[1]+(q[1]-p[1])*i/n
            if cell_cost(grid,x,y)>=99:
                hit=dict(xy=[x,y],now_cost=int(cell_cost(grid,x,y)),plan_time_cost=int(cell_cost(plan_grid,x,y)),distance_from_current=math.dist(current,(x,y)))
                break
        if hit:break
    cases.append(dict(event=e,match=True,full_time=f['time'],full_age_sec=(e['stamp_ns']-f['time'])/1e9,
        rejected_section=k,sections=len(sections),previous_dispatch_relative_sec=d['relative_sec'],
        branch=branch,current=[*current,yaw(odom.pose.pose)],
        global_check_at_handoff=check(grid),global_check_at_plan=check(plan_grid),first_center_hit=hit,
        public_map_received_age_sec=(e['stamp_ns']-gt)/1e9,local_map_received_age_sec=(e['stamp_ns']-lt)/1e9,
        public_map_stamp_ns=grid.header.stamp.sec*10**9+grid.header.stamp.nanosec,
        plan_public_map_stamp_ns=plan_grid.header.stamp.sec*10**9+plan_grid.header.stamp.nanosec))

out=dict(scope='Closed published-path/public-map reconstruction. Latest bag message is not proof of exact bridge cache or private Nav2 costmap bytes.',
    origin_wall=origin,full_candidates=full,dispatch=dispatch,handoff_cases=cases)
(root/'execution_loop_review.json').write_text(json.dumps(out,indent=2))
print(json.dumps(dict(full_candidates=len(full),dispatches=len(dispatch),matched_dispatches=sum(d['full_time'] is not None for d in dispatch),
    cases=[{k:v for k,v in c.items() if k not in ('branch','event')}|dict(relative_sec=c['event']['stamp_ns']/1e9-origin,reported=c['event'].get('reason'),retry_count=c['event'].get('retry_count')) for c in cases]),indent=2))
