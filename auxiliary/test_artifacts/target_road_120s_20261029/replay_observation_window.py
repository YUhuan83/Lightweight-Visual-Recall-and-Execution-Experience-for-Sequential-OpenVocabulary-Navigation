"""Use actual closed-bag clouds to inspect map-update observation loss."""
import json
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import OccupancyGrid
from wildos_nav2_adapter.pointcloud import decode_xyz
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.graph_route import footprint_blockers

root=Path(__file__).resolve().parent;trial=root/'full/trial_20261029_212751'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
audit=json.loads((trial/'tracking_collision_audit.json').read_text())[0]
pose=audit['path_blocked_within4m'][0]['pose'];abort=audit['stamp_ns']
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(k)) for i,n,k in db.execute('select id,name,type from topics')}
ns=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec
index,kind=topics['/global_costmap/costmap']
rt,data=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(index,abort)).fetchone()
map_msg=deserialize_message(data,kind);stamp=ns(map_msg)
i,k=topics['/spot1/odom']
odoms={ns(m):m for (data,) in db.execute('select data from messages where topic_id=?',(i,)) for m in [deserialize_message(data,k)]}
i,k=topics['/wildos/points'];frames=[];output=root/'observation_fixture';output.mkdir(exist_ok=True)
for receipt,data in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,stamp-1_500_000_000,stamp)):
    cloud=deserialize_message(data,k);odom=odoms.get(ns(cloud))
    if odom is None:continue
    q=odom.pose.pose.orientation;p=odom.pose.pose.position
    r=np.asarray(matrix_from_quaternion([q.x,q.y,q.z,q.w]));base=np.array([p.x,p.y,p.z])
    assert cloud.header.frame_id=='spot1/base_link'
    points=decode_xyz(cloud);world=points@r.T+base
    world=world[np.linalg.norm(points,axis=1)<=25.]
    m=OccupancyGrid();m.info=map_msg.info
    cells=np.floor((world[:,:2]-[m.info.origin.position.x,m.info.origin.position.y])/m.info.resolution).astype(int)
    grid=np.zeros((m.info.height,m.info.width),dtype=np.int8)
    valid=(cells>=0).all(axis=1)&(cells[:,0]<m.info.width)&(cells[:,1]<m.info.height)
    cells=cells[valid];grid[cells[:,1],cells[:,0]]=100;m.data=grid.ravel().tolist()
    path=output/f'frame_{len(frames)}.bin'
    # Keep raw world cloud for a real native ObservationBuffer regression.
    world.astype('<f4').tofile(path)
    frames.append(dict(receipt_ns=receipt,source_stamp_ns=ns(cloud),file=str(path),
        receipt_age_s=(stamp-receipt)*1e-9,source_age_s=(stamp-ns(cloud))*1e-9,
        blockers=footprint_blockers(m,pose[:2],pose[2])))
db.close()
report=dict(scope='Marking-only native-grid raster replay at the last global map publication. Does not identify the exact internal map-update callback instant.',
    map_stamp_ns=stamp,map_receipt_ns=rt,pose=pose,origin=[map_msg.info.origin.position.x,map_msg.info.origin.position.y],
    resolution=map_msg.info.resolution,width=map_msg.info.width,height=map_msg.info.height,frames=frames)
(root/'observation_window_replay.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
