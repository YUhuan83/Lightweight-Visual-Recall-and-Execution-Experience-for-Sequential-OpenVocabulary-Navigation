"""Closed full paths: compare cusp segment sizes with real action tolerances."""
import hashlib,json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.motion_path import motion_sections
base=Path('/home/ly/execution_continuity_review_20261022');root=base/sys.argv[1]
assert root.name in ('v34_vehicle_observation','v35_confirmed_vehicle','v36_weak_observation')
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
goal=next(Path(r['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(typ)) for i,n,typ in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
plans=[(t,json.loads(m.data)) for t,m in rows('/wildos/plan_status')]
odom={m.header.stamp.sec*10**9+m.header.stamp.nanosec:m for _,m in rows('/spot1/odom')}
native=[(t,json.loads(m.data)) for t,m in rows('/wildos/native_control_status')]
paths=[]
for t,m in rows('/wildos/full_planned_path'):
    stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    poses=[[p.pose.position.x,p.pose.position.y,math.atan2(2*(p.pose.orientation.w*p.pose.orientation.z+p.pose.orientation.x*p.pose.orientation.y),1-2*(p.pose.orientation.y**2+p.pose.orientation.z**2))] for p in m.poses]
    sections=motion_sections([p[:2] for p in poses]);parts=[]
    for a,b in sections:
        length=sum(math.dist(poses[i][:2],poses[i+1][:2]) for i in range(a,b))
        parts.append(dict(first=a,last=b,length_m=length,start=poses[a],end=poses[b]))
    p=next((d for s,d in reversed(plans) if s<=t),{})
    errors=[]
    for s,d in plans:
        if s<t or d.get('path_source_stamp_ns')!=stamp or d.get('status')!='replanning_after_maneuver':continue
        error=dict(d)
        o=odom.get(d.get('replay_inputs',{}).get('odom_stamp_ns'))
        if o is not None:
            pose=o.pose.pose;q=pose.orientation;v=o.twist.twist
            error['consumed_odom_pose']=[pose.position.x,pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            error['consumed_planar_speed']=math.hypot(v.linear.x,v.linear.y)
            error['consumed_yaw_rate']=v.angular.z
        status=next((value for received,value in reversed(native) if received<=s),{})
        error['native_speed_at_failure']=status.get('speed')
        error['native_telemetry_age_at_failure']=status.get('telemetry_age_sec')
        errors.append(error)
    if any(x['length_m']<.2 for x in parts) or errors:
        paths.append(dict(received_ns=t,stamp_ns=stamp,frame=m.header.frame_id,source=p.get('source'),plan_serial=p.get('request_serial'),
            sections=parts,matched_replans=errors,poses=poses))
db.close()
report=dict(scope='Actual closed full path samples and exact path_source_stamp matching; short length alone is not proof every path was executed.',trial=r['trial'],paths=paths)
name=sys.argv[2] if len(sys.argv)>2 else 'motion_section_audit_1.json'
encoded=json.dumps(report,indent=2).encode();(root/name).write_bytes(encoded)
print(json.dumps(dict(report_sha256=hashlib.sha256(encoded).hexdigest(),paths=len(paths),
    tiny_sections=sum(x['length_m']<.2 for p in paths for x in p['sections']),
    exact_matched_failures=[dict(stamp_ns=p['stamp_ns'],section_lengths_m=[x['length_m'] for x in p['sections']],reasons=[x.get('join_failure_reason') or x.get('reason') for x in p['matched_replans']],measured=[{k:d.get(k) for k in ('consumed_odom_pose','consumed_planar_speed','consumed_yaw_rate','native_speed_at_failure')} for d in p['matched_replans']]) for p in paths if p['matched_replans']]),indent=2))
