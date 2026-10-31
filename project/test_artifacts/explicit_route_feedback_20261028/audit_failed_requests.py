"""Read closed v63 interfaces; preserve exact raw-map/start/goal fixtures."""
import json, math, sqlite3
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback')
trial=root/'full/trial_20261028_230923'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
out=root/'request_audit';out.mkdir(exist_ok=True)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def last(name,stamp):
    i,kind=topics[name]
    t,b=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()
    return t,deserialize_message(b,kind)
events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
requests={e['request_serial']:e for e in events if e.get('status')=='planning'}
summary=[]
for serial in (24,34):
    e=requests[serial];stamp=e['stamp_ns'];mt,m=last('/global_costmap/costmap_raw',stamp);ot,o=last('/spot1/odom',stamp)
    p=o.pose.pose.position;q=o.pose.pose.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    start=(p.x,p.y,yaw);target=e['requested_goal_pose'];meta=m.metadata
    filename=out/f'request_{serial}.txt'
    with filename.open('w') as f:
        f.write(' '.join(map(str,[meta.size_x,meta.size_y,meta.resolution,meta.origin.position.x,meta.origin.position.y,*start,1]))+'\n')
        f.write(' '.join(map(str,target))+'\n')
        for i,c in enumerate(m.data):
            if c:f.write(f'{i} {c}\n')
    grid=np.array(m.data,dtype=float).reshape(meta.size_y,meta.size_x)
    # Palette separates actual lethal/inscribed/unknown from traversable inflation.
    view=np.zeros((*grid.shape,3));view[:]=.93
    view[(grid>0)&(grid<253)]=[.84,.78,.55]
    view[grid==253]=[.9,.35,.35];view[grid==254]=[.12,.12,.12];view[grid==255]=[.62,.69,.78]
    fig,ax=plt.subplots(figsize=(8,8))
    ax.imshow(view,origin='lower',extent=[meta.origin.position.x,meta.origin.position.x+meta.size_x*meta.resolution,meta.origin.position.y,meta.origin.position.y+meta.size_y*meta.resolution])
    ax.plot(p.x,p.y,'go',label='actual axle pose');ax.arrow(p.x,p.y,3*math.cos(yaw),3*math.sin(yaw),width=.08,color='g')
    ax.plot(target[0],target[1],'rx',markersize=10,label='requested vehicle goal')
    ax.arrow(target[0],target[1],2*math.cos(target[2]),2*math.sin(target[2]),width=.06,color='r')
    path_topics=[n for n in topics if n.endswith('/planned_path') or n.endswith('/path')]
    for name in path_topics:
        try:
            pt,path=last(name,stamp)
            if not hasattr(path,'poses'):continue
            ax.plot([v.pose.position.x for v in path.poses],[v.pose.position.y for v in path.poses],label=name,alpha=.7)
        except TypeError:pass
    ax.set_xlim(p.x-18,p.x+18);ax.set_ylim(p.y-25,p.y+10);ax.set_aspect('equal');ax.legend(fontsize=7)
    ax.set_title(f'Closed v63 request {serial}, query +{e["elapsed"]-events[0]["elapsed"]:.2f}s')
    fig.savefig(out/f'request_{serial}.png',dpi=150);plt.close(fig)
    summary.append(dict(request=serial,start=start,target=target,map_receipt_age_s=(stamp-mt)*1e-9,odom_receipt_age_s=(stamp-ot)*1e-9,fixture=str(filename)))
db.close()
(out/'scope.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
