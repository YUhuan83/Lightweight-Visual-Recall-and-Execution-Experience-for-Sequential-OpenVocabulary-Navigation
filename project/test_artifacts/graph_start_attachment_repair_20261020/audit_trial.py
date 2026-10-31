"""Review a terminal field trial: actual displacement, stops, route/ownership events."""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import math
import sqlite3
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String

p=argparse.ArgumentParser();p.add_argument('trial',type=Path);args=p.parse_args()
driver=json.loads((args.trial/'result.json').read_text());assert driver['closed']
session=Path(driver['session']);goal=session/'goal_001';db=goal/'interfaces/interfaces_0.db3'
con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in con.execute('SELECT id,name,type FROM topics')}
series={}
for topic in ('/wildos/plan_status','/wildos/execution_feedback','/spot1/graphnav_planner/route_status','/spot1/graphnav_planner/retired_exploration'):
    if topic not in topics:continue
    ident,typ=topics[topic];assert typ=='std_msgs/msg/String'
    rows=[]
    for stamp,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(ident,)):
        try: rows.append((stamp,json.loads(deserialize_message(data,String).data)))
        except (ValueError,TypeError):continue
    series[topic]=rows
odom=[];ident,_=topics['/spot1/odom']
for stamp,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(ident,)):
    m=deserialize_message(data,Odometry);pos=m.pose.pose.position;v=m.twist.twist.linear
    odom.append((stamp,[pos.x,pos.y],math.hypot(v.x,v.y)))
con.close();begin=odom[0][0]
stops=[];start=None
for t,xy,v in odom:
    if v<.1:
        if start is None:start=t
    elif start is not None:
        if t-start>=5e9:stops.append(dict(start_s=(start-begin)*1e-9,end_s=(t-begin)*1e-9))
        start=None
if start is not None and odom[-1][0]-start>=5e9:stops.append(dict(start_s=(start-begin)*1e-9,end_s=(odom[-1][0]-begin)*1e-9))
plans=series.get('/wildos/plan_status',[])
for stop in stops:
    latest=next((v for t,v in reversed(plans) if t<=begin+round(stop['start_s']*1e9)),None)
    stop['status_at_start']=latest
retirements=[dict(time_s=(t-begin)*1e-9,**v) for t,v in series.get('/spot1/graphnav_planner/retired_exploration',[])]
summary=dict(scope=__doc__,driver=driver,odom_samples=len(odom),first_xy=odom[0][1],last_xy=odom[-1][1],
             net_displacement_m=math.dist(odom[0][1],odom[-1][1]),
             traveled_distance_m=sum(math.dist(a[1],b[1]) for a,b in zip(odom,odom[1:])),
             duration_s=(odom[-1][0]-begin)*1e-9,max_displacement_m=max(math.dist(odom[0][1],x[1]) for x in odom),
             plan_status_counts=dict(collections.Counter(v.get('status') for _,v in plans)),
             execution_reason_counts=dict(collections.Counter(v.get('reason') for _,v in series.get('/wildos/execution_feedback',[]))),
             route_complete_counts=dict(collections.Counter(str(v.get('complete')) for _,v in series.get('/spot1/graphnav_planner/route_status',[]))),
             retirements=retirements,stops_over_5sec=stops,
             topics={n:t for n,(_,t) in topics.items()},complete_navigation_passed=False,
             limits='Public recorded interfaces; stop intervals include admission, maneuvering and terminal cleanup; full visual arrival needs image/identity review.')
(args.trial/'closed_audit.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('driver','topics','stops_over_5sec','retirements')},indent=2))
print('retirements',len(retirements),'stops',[(s['start_s'],s['end_s'],(s['status_at_start'] or {}).get('status')) for s in stops])
