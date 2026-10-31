"""Export actually admitted temporary-observation shunts from a closed bag."""
import json,math,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.motion_path import motion_sections
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v35_confirmed_vehicle'
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
goal=next(Path(r['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(typ)) for i,n,typ in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
plans=[(t,json.loads(m.data)) for t,m in rows('/wildos/plan_status')]
out=[]
for t,path in rows('/wildos/full_planned_path'):
    points=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
    headings=[math.atan2(2*(p.pose.orientation.w*p.pose.orientation.z+p.pose.orientation.x*p.pose.orientation.y),1-2*(p.pose.orientation.y**2+p.pose.orientation.z**2)) for p in path.poses]
    sections=motion_sections(points)
    if len(sections)<2:continue
    previous=next((d for s,d in reversed(plans) if s<=t),None)
    if not previous or previous.get('source')!='visible_target_via_graphnav':continue
    admitted=next((d for s,d in plans if t<=s<=t+3_000_000_000 and d.get('status')=='planned'
        and d.get('source')=='visible_target_via_graphnav' and d.get('motion_sections_remaining',0)>1
        and d.get('graph_goal') and math.dist(d['graph_goal'],points[-1])<.6),None)
    if admitted is None:continue
    if any(math.dist(points[0],c['points'][0])<1. for c in out):continue
    out.append(dict(stamp_ns=t,frame=path.header.frame_id,points=points,headings=headings,
        sections=len(sections),planning=previous,admitted=admitted))
    if len(out)==2:break
db.close()
assert out,'No matching admitted temporary-observation path'
report=dict(scope='Real full path produced by validated bridge and then admitted with multiple motion sections; no new planning or vehicle execution.',trial=r['trial'],cases=out)
(root/'weak_shunt_cases.json').write_text(json.dumps(report,indent=2))
print(json.dumps([dict(stamp_ns=c['stamp_ns'],sections=c['sections'],poses=len(c['points']),length=sum(math.dist(a,b) for a,b in zip(c['points'],c['points'][1:])),direct_distance=math.dist(c['points'][0],c['points'][-1])) for c in out],indent=2))
