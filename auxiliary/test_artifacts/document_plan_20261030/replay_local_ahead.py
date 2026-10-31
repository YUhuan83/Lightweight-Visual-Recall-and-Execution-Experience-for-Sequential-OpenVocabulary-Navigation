"""CLOSED v67 replay: old imminent versus covered 8m local-map admission."""
import bisect,json,math,sqlite3,time
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import validate_local_prefix
from wildos_nav2_adapter.continuous_path import trim_moving_path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
trial=root.with_name('v67_target_road_120s')/'full/trial_20261029_212751'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
q=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
first=next(e for e in events if e.get('stamp_ns'));origin=first['stamp_ns']-round((first['elapsed']-q)*1e9)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
print('Recorded relevant topics:',[n for n in topics if any(k in n for k in ('path','costmap','odom'))],flush=True)
def rows(name):
    i,c=topics[name]
    return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
        (i,origin,origin+120000000000))]
odom=rows('/spot1/odom');ot=[t for t,_ in odom]
local=rows('/local_costmap/costmap')
path_topic='/wildos/controller_path_progress'
if path_topic not in topics:
    raise RuntimeError('Actual controller progress path is not recorded; do not substitute another path silently')
paths=rows(path_topic);pt=[t for t,_ in paths];db.close()
reports=[]
for repeat in (1,2):
    records=[];began=time.perf_counter()
    for stamp,m in local:
        second=(stamp-origin)*1e-9
        if not 80<=second<=95.751:continue
        oi=bisect.bisect_right(ot,stamp)-1;pi=bisect.bisect_right(pt,stamp)-1
        if min(oi,pi)<0:continue
        _,o=odom[oi];_,path=paths[pi];pose=o.pose.pose;p=pose.position;a=pose.orientation
        yaw=math.atan2(2*(a.w*a.z+a.x*a.y),1-2*(a.y*a.y+a.z*a.z))
        path=trim_moving_path(path,(p.x,p.y),yaw)
        if path is None:continue
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        headings=[math.atan2(2*(p.pose.orientation.w*p.pose.orientation.z+p.pose.orientation.x*p.pose.orientation.y),
                            1-2*(p.pose.orientation.y**2+p.pose.orientation.z**2)) for p in path.poses]
        old=validate_local_prefix(points,headings,m,(p.x,p.y),yaw)
        new=validate_local_prefix(points,headings,m,(p.x,p.y),yaw,8.,covered_only=True)
        records.append(dict(q=second,old=old,new=new,start=[p.x,p.y,yaw],map_stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
            odom_age_s=(stamp-ot[oi])*1e-9,path_age_s=(stamp-pt[pi])*1e-9,points=points,headings=headings))
    early=next((r for r in records if r['new'] and not r['old']),None)
    reports.append(dict(repeat=repeat,frames=len(records),first_early=early,
        lead_to_recorded_abort_s=95.751-early['q'] if early else None,elapsed_s=time.perf_counter()-began))
assert reports[0]['first_early']==reports[1]['first_early']
report=dict(scope='Recorded controller progress, odom and local map; trigger replay only, not counterfactual driving',runs=reports)
(root/'v67_local_ahead_replay.json').write_text(json.dumps(report,indent=2))
print(json.dumps([{k:v for k,v in r.items() if k!='first_early'}|{'first_early_q':r['first_early']['q'] if r['first_early'] else None} for r in reports],indent=2))
