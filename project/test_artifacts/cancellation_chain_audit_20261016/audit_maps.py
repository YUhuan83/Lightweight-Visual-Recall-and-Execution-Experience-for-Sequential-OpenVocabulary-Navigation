"""Read-only bounded public-map check at actual cancellation times."""
from pathlib import Path
import collections,json,math,sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.graph_route import validate_local_prefix,validate_vehicle_path,distance

root=Path('/home/ly/cancellation_chain_audit_20261016')
session=Path('/home/ly/frontier_resolution_repair_20261016/field_results/session_20261016_200923')
assert (session/'session_result.json').exists() and (session/'goal_001/result.json').exists()
assert Path('/home/ly/frontier_resolution_repair_20261016/field_scope_baseline_search_20261016_200921/result.json').exists()
audit=json.loads((root/'timeline.json').read_text())
db=sqlite3.connect(next((session/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def latest(topic,cutoff):
    tid,cls=topics[topic]
    row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,cutoff)).fetchone()
    return (row[0],deserialize_message(row[1],cls)) if row else (None,None)
def xy(p):return (p.position.x,p.position.y)
def yaw(p):
    q=p.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def age(m,now):return (now-m.header.stamp.sec*10**9-m.header.stamp.nanosec)/1e9
records=[]
for row in audit['cancellations']:
    ts=row['stamp_ns'];target=row['previous_follow']['goal']
    ot,o=latest('/spot1/odom',ts)
    gt,g=latest('/global_costmap/costmap',ts)
    lt,l=latest('/local_costmap/costmap',ts)
    paths=[]
    for topic in ('/received_global_plan','/wildos/planned_path'):
        tid,cls=topics[topic]
        for receipt,raw in db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 4',(tid,ts)):
            p=deserialize_message(raw,cls)
            if p.poses and distance(xy(p.poses[-1].pose),target)<.05:
                paths.append((receipt,topic,p));break
    # Prefer the controller's received plan when its endpoint matches actual accepted goal.
    chosen=next((p for p in paths if p[1]=='/received_global_plan'),paths[0] if paths else None)
    result=dict(stamp_ns=ts,exploration_id=row['source_retirement']['exploration_id'],
        target=target,path_found=chosen is not None,odom_source_age_s=age(o,ts),global_source_age_s=age(g,ts),local_source_age_s=age(l,ts))
    if chosen:
        pt,topic,path=chosen
        assert path.header.frame_id==o.header.frame_id==g.header.frame_id==l.header.frame_id=='spot1/odom'
        trimmed=trim_moving_path(path,xy(o.pose.pose),yaw(o.pose.pose))
        result.update(path_topic=topic,path_receipt_age_s=(ts-pt)/1e9,trimmed=trimmed is not None,
            speed=math.hypot(o.twist.twist.linear.x,o.twist.twist.linear.y))
        if trimmed:
            points=[xy(p.pose) for p in trimmed.poses];headings=[yaw(p.pose) for p in trimmed.poses]
            result.update(global_prefix_issue=validate_local_prefix(points,headings,g,xy(o.pose.pose),yaw(o.pose.pose)),
                local_prefix_issue=validate_local_prefix(points,headings,l,xy(o.pose.pose),yaw(o.pose.pose)),
                remaining_global_path_issue=validate_vehicle_path(points,headings,points,g,xy(trimmed.poses[-1].pose),
                    minimum_turning_radius=8.,start=xy(o.pose.pose),heading_quantization_bins=72))
    records.append(result)
scope='Latest publicly recorded maps/odom received before cancel, matching accepted endpoint; production trim and safety functions. Not private plugin costmap or actual RPP command-arc replay.'
summary=dict(scope=scope,cancellations=len(records),matching_plans=sum(r['path_found'] for r in records),
    trimmed=sum(r.get('trimmed',False) for r in records),
    prefix_results=dict(collections.Counter(str((r.get('global_prefix_issue'),r.get('local_prefix_issue'))) for r in records if r.get('trimmed'))),
    remaining_path_results=dict(collections.Counter(str(r.get('remaining_global_path_issue')) for r in records if r.get('trimmed'))))
(root/'map_review.json').write_text(json.dumps(dict(summary=summary,records=records),indent=2)+'\n')
print(json.dumps(summary,indent=2))
