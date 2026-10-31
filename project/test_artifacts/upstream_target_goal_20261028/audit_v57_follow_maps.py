"""Inspect recorded map/odom near RPP aborts; not a reproduction of its command arc."""
import json,sqlite3,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers
from wildos_nav2_adapter.graph_path_bridge import xy,yaw

root=Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach')
receipt=json.loads((root/'field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def before(name,stamp):
    i,c=topics[name];row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()
    return (row[0],deserialize_message(row[1],c)) if row else (None,None)
out=[]
for stamp in [1793150208648132432,1793150232688017379,1793150254603779477]:
    ot,o=before('/spot1/odom',stamp);mt,m=before('/local_costmap/costmap',stamp);ct,c=before('/wildos/cmd_vel',stamp)
    p=xy(o.pose.pose);h=yaw(o.pose.pose)
    blockers=footprint_blockers(m,p,h,limit=20)
    out.append({'abort_ns':stamp,'odom_receipt_age_s':(stamp-ot)*1e-9,'map_receipt_age_s':(stamp-mt)*1e-9,'command_receipt_age_s':(stamp-ct)*1e-9,'odom_header_ns':o.header.stamp.sec*1000000000+o.header.stamp.nanosec,'map_header_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'map_frame':m.header.frame_id,'odom_frame':o.header.frame_id,'position':[p[0],p[1],h],'last_published_command':[c.linear.x,c.angular.z], 'current_footprint_blockers':blockers})
db.close()
(root/'v57_follow_maps.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
