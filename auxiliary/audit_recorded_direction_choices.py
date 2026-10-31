"""Trace accepted section geometry to full-path candidates in closed A/B bags."""
import hashlib
import json
import math
import sqlite3
from pathlib import Path
from nav_msgs.msg import Path as RosPath
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from wildos_nav2_adapter.motion_path import motion_sections

root=Path('/home/ly/execution_continuity_review_20261022')
records=json.loads((root/'stop_plan_ab_20261024/results.json').read_text())
output=[]
def xy(p):return (p.position.x,p.position.y)
def yaw(p):
    q=p.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose_key(p):return tuple(round(v,3) for v in (*xy(p),yaw(p)))
def shape(path):
    points=[xy(p.pose) for p in path.poses]
    heading=yaw(path.poses[0].pose)
    edge=next(((b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]) if math.dist(a,b)>1e-6),(0.,0.))
    sections=motion_sections(points)
    lengths=[sum(math.dist(a,b) for a,b in zip(points[first:last+1],points[first+1:last+1]))
             for first,last in sections]
    return dict(start=[*points[0],heading],goal=[*points[-1],yaw(path.poses[-1].pose)],
                length=sum(lengths),sections=len(sections),section_lengths=lengths,
                direction=1 if edge[0]*math.cos(heading)+edge[1]*math.sin(heading)>0 else -1)
for record in records:
    trial=Path(record['trial'])
    assert record['result']['closed'] and not record['postcheck']['processes']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    def read(topic,kind):
        tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
        return [(stamp,deserialize_message(blob,kind)) for stamp,blob in db.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    plans=sorted({d['stamp_ns']:d for _,m in read('/wildos/plan_status',String)
                  for d in [json.loads(m.data)]}.items())
    full=read('/wildos/full_planned_path',RosPath)
    segments=read('/wildos/planned_path',RosPath)
    db.close()
    full_keys=[{pose_key(p.pose) for p in path.poses} for _,path in full]
    accepted=[]
    for stamp,event in plans:
        if event['status']!='following' or 'goal' not in event:continue
        target=event['goal']
        prior=[(t,d) for t,d in plans if t<=stamp and d['status']=='planned'
               and d.get('goal') is not None and math.dist(d['goal'],target)<1e-4 and stamp-t<5_000_000_000]
        if not prior:continue
        pt,planned=prior[-1]
        candidates=[(t,p) for t,p in segments if p.poses and abs(t-pt)<300_000_000
                    and math.dist(xy(p.poses[-1].pose),target)<1e-4]
        if not candidates:continue
        section_t,section=min(candidates,key=lambda item:abs(item[0]-pt))
        keys=[pose_key(section.poses[min(len(section.poses)-1,int(len(section.poses)*r))].pose)
              for r in (.25,.5,.75)]
        matches=[i for i,(t,p) in enumerate(full) if t<=section_t+30_000_000
                 and section_t-t<60_000_000_000 and all(key in full_keys[i] for key in keys)]
        selected=matches[-1] if matches else None
        accepted.append(dict(stamp_ns=stamp,source=event.get('source'),goal=target,
            actual_start=planned.get('actual_start_pose'),remaining_sections=planned.get('motion_sections_remaining'),
            section=shape(section),full_candidate_index=selected,
            full=shape(full[selected][1]) if selected is not None else None,
            matching_full_candidates=len(matches),geometry_match='three section poses rounded to 1mm/.001rad'))
    flips=[dict(previous=a,current=b) for a,b in zip(accepted,accepted[1:])
           if a['section']['direction']!=b['section']['direction']]
    continuous=[d for _,d in plans if d['status']=='visible_target_approach' and d.get('continuous')]
    resets=[d for _,d in plans if d['status']=='visible_target_approach' and not d.get('continuous')]
    result=dict(trial=str(trial),version=record['version'],near_stop_pct=record['fluency']['near_stop_pct'],
                acknowledged_sections=accepted,direction_changes_between_acknowledged_sections=flips,
                continuous_visual_updates=len(continuous),new_visual_intentions=len(resets),
                full_candidate_count=len(full),
                caveat='Geometry matching is not an action UUID; publication/ACK ordering and duplicate geometries may remain ambiguous.')
    (trial/'direction_choice_audit.json').write_text(json.dumps(result,indent=2))
    output.append(result)
db.close()
encoded=json.dumps(output,sort_keys=True).encode()
(root/'stop_plan_ab_20261024/direction_choice_audit.json').write_bytes(encoded)
print(hashlib.sha256(encoded).hexdigest())
