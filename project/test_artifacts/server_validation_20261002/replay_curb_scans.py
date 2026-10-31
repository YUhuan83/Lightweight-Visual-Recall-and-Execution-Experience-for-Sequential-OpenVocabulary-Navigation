"""Read earlier bag scans to separate blind regions from height filtering."""
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.pointcloud import decode_xyz
from beamng_wildos_adapter.geometry import quaternion_matrix
from wildos_nav2_adapter.pointcloud import filter_robot_footprint,filter_height,local_ground_plane,split_costmap_points
folder=ROOT/'stability_frontier_trial5';audit=json.loads((ROOT/'failure_audit/stability_frontier_trial5/audit.json').read_text());base=audit['trial_base_wall_time']
db=sqlite3.connect('file:'+str(next((folder/'rosbag').glob('*.db3')))+'?mode=ro',uri=True)
topics={name:(ident,get_message(kind)) for ident,name,kind in db.execute('select id,name,type from topics')}
def before(topic,stamp):
    ident,kind=topics[topic];row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(ident,stamp)).fetchone()
    return (row[0],deserialize_message(row[1],kind)) if row else (None,None)
records=[]
for elapsed in [5.,30.,99.,105.,119.,134.,140.,145.,149.]:
    t,cloud=before('/spot1/ouster/front/points_filtered',int((base+elapsed)*1e9))
    if cloud is None:continue
    ot,odom=before('/spot1/odom',t);p=odom.pose.pose.position;q=odom.pose.pose.orientation
    rotation=quaternion_matrix([q.x,q.y,q.z,q.w]);rear=np.array([p.x,p.y,p.z])
    points=decode_xyz(cloud)-np.array([-2.172,.330,0.])
    outside=filter_robot_footprint(points,3.4,1.05,1.05);height=filter_height(outside,.1,3.)
    plane=local_ground_plane(outside,8.)
    marking,_=split_costmap_points(points,min_height=.1,max_height=3.,filter_footprint=True,
        front_extent=3.4,rear_extent=1.05,half_width=1.05,voxel=.2,max_points=20000,
        ground_plane_radius=8.,ground_clearance=.25)
    def stats(cloud_points):
        world=cloud_points@rotation.T+rear
        region=((world[:,0]>=725.3)&(world[:,0]<=727.)&(world[:,1]>=-15.)&(world[:,1]<=-12.)&(world[:,2]>=51.7)&(world[:,2]<=52.4))
        selected=cloud_points[region];out={'region_points':len(selected)}
        if len(selected):
            out['base_z_p05_p50_p95']=np.percentile(selected[:,2],[5,50,95]).tolist()
            if plane is not None:
                h=selected[:,2]-selected[:,:2]@plane[:2]-plane[2]
                out['road_clearance_p05_p50_p95']=np.percentile(h,[5,50,95]).tolist()
                out['above_road_5_8_12_25cm']=[int(np.sum(h>=threshold)) for threshold in [.05,.08,.12,.25]]
        return out
    grids={}
    for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
        gt,grid=before(topic,t)
        if grid is None:continue
        iy,ix=np.indices((grid.info.height,grid.info.width));resolution=grid.info.resolution
        x=grid.info.origin.position.x+(ix+.5)*resolution;y=grid.info.origin.position.y+(iy+.5)*resolution
        region=(x>=725.3)&(x<=727.)&(y>=-15.)&(y<=-12.)
        data=np.array(grid.data).reshape(grid.info.height,grid.info.width)[region]
        grids[topic]={'region_cells':len(data),'occupied_100':int(np.sum(data==100)),
            'inflated_99':int(np.sum(data==99)),'max_cost':int(data.max()) if len(data) else None}
    records.append({'elapsed':elapsed,'rear':rear.tolist(),'odom_receive_age':(t-ot)/1e9,'costmaps':grids,
        'plane':plane.tolist() if plane is not None else None,
        'stages':{key:stats(value) for key,value in [('input',points),('outside_body',outside),('absolute_height_pass',height),('marking',marking)]}})
result={'records':records,'region_world':{'x':[725.3,727.],'y':[-15.,-12.],'z':[51.7,52.4]},
    'limitation':'Region also contains road; heights inferred relative to fitted road. Uses odometry preceding scan reception, so driving scans have small pose timing uncertainty.'}
(folder/'curb_scan_replay.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);db.close()
