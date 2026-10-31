"""Attribute published full-path cusps to connector and searched suffix."""
from pathlib import Path
import json,math,re,sqlite3
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as ROSPath

trial=Path('/home/ly/execution_continuity_review_20261022/v2/full/trial_20261022_163552')
session=next((trial/'sessions').glob('session_*'));goal=session/'goal_001'
summary=json.loads((trial/'fluency_audit/summary_420.json/closed_trial_summary.json').read_text())
t0=summary['query_start_bag_wall']
selected=[];current=[]
for line in (session/'stack.log').read_text().splitlines():
    time=re.search(r'\[(\d{10}\.\d+)\]',line)
    if not time:continue
    stamp=float(time[1])
    match=re.search(r'connector_candidate: cost=([\d.]+).*?poses=(\d+) connector_poses=(\d+) cusps=(\d+)',line)
    if match: current.append(dict(cost=float(match[1]),poses=int(match[2]),prefix=int(match[3]),cusps=int(match[4])))
    match=re.search(r'continuous_complete_selected: cost=([\d.]+)',line)
    if match:
        cost=float(match[1]); choices=[r for r in current if abs(r['cost']-cost)<=.00051]
        selected.append(dict(stamp=stamp,cost=cost,choices=choices,kind='connector'))
        current=[]
    elif 'continuous_goal_selected:' in line:
        selected.append(dict(stamp=stamp,kind='direct'));current=[]
    elif 'continuous_start:' in line:current=[]

def cusps(poses):
    result=[];previous=None
    for i,(a,b) in enumerate(zip(poses,poses[1:]),1):
        dx=b.pose.position.x-a.pose.position.x;dy=b.pose.position.y-a.pose.position.y
        if math.hypot(dx,dy)<1e-9:continue
        if previous and dx*previous[0]+dy*previous[1]<0: result.append(i-1)
        previous=(dx,dy)
    return result

db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
topic=db.execute('select id,type from topics where name=?',('/wildos/full_planned_path',)).fetchone()
assert topic and topic[1]=='nav_msgs/msg/Path',topic
out=[]
for ns,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topic[0],)):
    path=deserialize_message(data,ROSPath);now=ns*1e-9
    matches=[r for r in selected if 0<=now-r['stamp']<=.5]
    picked=matches[-1] if matches else None
    c=cusps(path.poses)
    row=dict(source_s=now-t0,poses=len(path.poses),cusps=len(c),matched=False)
    if picked:
        row['kind']=picked['kind'];row['selection_to_receipt_s']=now-picked['stamp']
        choices=[r for r in picked.get('choices',[]) if r['poses']==len(path.poses) and r['cusps']==len(c)]
        prefixes={r['prefix'] for r in choices}
        if picked['kind']=='direct':row['matched']=True;row['direct_cusps']=len(c)
        elif len(prefixes)==1:
            prefix=prefixes.pop();row.update(matched=True,connector_poses=prefix,
                connector_cusps=sum(i<prefix-1 for i in c),boundary_cusps=sum(i==prefix-1 for i in c),
                suffix_cusps=sum(i>prefix-1 for i in c),selection_cost=picked['cost'])
    out.append(row)
result={'scope':__doc__,'paths':out,'matched':sum(r['matched'] for r in out),
        'connector_cusps':sum(r.get('connector_cusps',0) for r in out),
        'boundary_cusps':sum(r.get('boundary_cusps',0) for r in out),
        'suffix_cusps':sum(r.get('suffix_cusps',0) for r in out),
        'direct_cusps':sum(r.get('direct_cusps',0) for r in out)}
(trial/'fluency_audit/cusp_origin.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
