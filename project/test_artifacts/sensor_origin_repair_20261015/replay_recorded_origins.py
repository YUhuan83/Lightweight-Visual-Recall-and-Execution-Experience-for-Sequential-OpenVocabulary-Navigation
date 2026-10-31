from pathlib import Path
import sqlite3,json
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from wildos_nav2_adapter.pointcloud import decode_xyz
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET
from beamng_wildos_adapter.mapping import occupancy_from_points
s=Path('/home/ly/wildos-console/audit_results/session_20261015_201924')
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
c=sqlite3.connect(next((s/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid=c.execute("select id from topics where name='/spot1/ouster/front/points_filtered'").fetchone()[0]
start,end=c.execute('select min(timestamp),max(timestamp) from messages where topic_id=?',(tid,)).fetchone()
rows=[]
for at in (start+(end-start)*i//7 for i in range(8)):
 ts,blob=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,int(at))).fetchone()
 cloud=deserialize_message(blob,PointCloud2);points=decode_xyz(cloud)-np.array(SBR_BASE_OFFSET)
 old=occupancy_from_points(points,sensor_origin_xy=-np.array(SBR_BASE_OFFSET)[:2],clear_robot_footprint=True)
 new=occupancy_from_points(points,sensor_origin_xy=np.array(SBR_LIDAR_OFFSET)[:2],clear_robot_footprint=True)
 assert np.array_equal(old==100,new==100),'Observed obstacles must not change'
 rows.append(dict(received_ns=ts,source_frame=cloud.header.frame_id,changed_cells=int(np.sum(old!=new)),old_free_new_unknown=int(np.sum((old==0)&(new<0))),old_unknown_new_free=int(np.sum((old<0)&(new==0))),obstacle_cells=int(np.sum(new==100))))
c.close();r={'scope':'Same sealed raw scans, same obstacle filter; only ray origin changed. Free-space differences are observability evidence, not simulator ground truth.','samples':rows}
p=Path('/home/ly/sensor_origin_repair_20261015/recorded_ray_origin_replay.json');p.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
