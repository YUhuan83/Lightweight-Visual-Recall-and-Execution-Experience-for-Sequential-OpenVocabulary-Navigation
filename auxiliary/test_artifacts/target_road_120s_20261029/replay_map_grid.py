"""Same measured obstacle returns, native-grid geometry and existing raster check."""
import json
import math
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import OccupancyGrid
from wildos_nav2_adapter.pointcloud import decode_xyz
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.graph_route import footprint_blockers

root=Path(__file__).resolve().parent
trial=root/'full/trial_20261029_212751'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
audit=json.loads((trial/'tracking_collision_audit.json').read_text())[0]
pose=audit['path_blocked_within4m'][0]['pose']; stamp=audit['stamp_ns']
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(k)) for i,n,k in db.execute('select id,name,type from topics')}
def before(topic):
    i,k=topics[topic];data=db.execute('select data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()[0]
    return deserialize_message(data,k)
cloud=before('/wildos/points')
ns=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec
i,k=topics['/spot1/odom']
odom=next(m for (data,) in db.execute('select data from messages where topic_id=?',(i,))
          for m in [deserialize_message(data,k)] if ns(m)==ns(cloud))
q=odom.pose.pose.orientation;p=odom.pose.pose.position
r=np.asarray(matrix_from_quaternion([q.x,q.y,q.z,q.w]));base=np.array([p.x,p.y,p.z])
points=decode_xyz(cloud);assert cloud.header.frame_id=='spot1/base_link'
world=points@r.T+base
world=world[np.linalg.norm(points,axis=1)<=25.]
db.close()
reports=[]
for resolution, origin in ((.25,(695.,-137.75)),(.2,(695.2,-138.))):
    m=OccupancyGrid();m.info.resolution=resolution;m.info.width=m.info.height=round(80/resolution)
    m.info.origin.position.x,m.info.origin.position.y=origin;m.info.origin.orientation.w=1.
    cells=np.floor((world[:,:2]-origin)/resolution).astype(int)
    data=np.zeros((m.info.height,m.info.width),dtype=np.int8)
    valid=(cells>=0).all(axis=1)&(cells[:,0]<m.info.width)&(cells[:,1]<m.info.height)
    cells=cells[valid];data[cells[:,1],cells[:,0]]=100;m.data=data.ravel().tolist()
    reports.append(dict(resolution=resolution,origin=origin,blockers=footprint_blockers(m,pose[:2],pose[2])))
report=dict(scope='Same measured current marking cloud; no clearing, history, inflation, or update-time differences. Tests raster representation only.',pose=pose,grids=reports)
(trial/'same_cloud_grid_replay.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
