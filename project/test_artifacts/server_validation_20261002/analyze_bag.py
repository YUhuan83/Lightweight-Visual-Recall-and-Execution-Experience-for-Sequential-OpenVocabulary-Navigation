"""Inspect actual RPP collision poses against time-matched bagged grids."""
from pathlib import Path
import json
import math
import sqlite3
import subprocess
import sys
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:], '--runtime-child'], env=runtime_env()).returncode)

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_path_bridge import xy, yaw
from wildos_nav2_adapter.graph_route import footprint_blockers, cell_cost, validate_vehicle_path
from beamng_wildos_adapter.pointcloud import decode_xyz

folder = ROOT/sys.argv[1]
connection = sqlite3.connect(next((folder/'rosbag').glob('*.db3')))
topics = {name:(ident, get_message(kind)) for ident,name,kind in connection.execute('select id,name,type from topics')}

def messages(topic):
    if topic not in topics: return []
    ident, kind = topics[topic]
    return [(stamp, deserialize_message(data, kind)) for stamp,data in connection.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp', (ident,))]

def nearest_before(topic, stamp):
    ident, kind = topics[topic]
    row = connection.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',
                             (ident, stamp)).fetchone()
    return (row[0],deserialize_message(row[1],kind)) if row else (None,None)

errors = [(t,m) for t,m in messages('/rosout') if 'collision ahead' in m.msg]
report = {'attempt':sys.argv[1], 'collision_errors':[]}
for stamp, message in errors:
    item = {'time':stamp/1e9, 'message':message.msg}
    ot, odom = nearest_before('/spot1/odom', stamp)
    point, heading = xy(odom.pose.pose), yaw(odom.pose.pose)
    item.update(position=point, heading=heading, odom_receive_age=(stamp-ot)/1e9)
    at, arc = nearest_before('/lookahead_collision_arc', stamp)
    if arc and arc.poses:
        item['arc_end'] = list(xy(arc.poses[-1].pose))
        # RPP fills only arc positions, leaving their orientation unset.
        # Use the last chord tangent as an approximate heading, not yaw=0.
        a,c = xy(arc.poses[-2].pose),xy(arc.poses[-1].pose)
        tangent = math.atan2(c[1]-a[1],c[0]-a[0])
        if math.cos(tangent-heading)<0: tangent += math.pi
        item['arc_end_heading'] = tangent
        item['arc_end_heading_is_approximate'] = True
        item['arc_poses'] = len(arc.poses)
    for topic, label in [('/global_costmap/costmap','global'),('/local_costmap/costmap','local')]:
        gt, grid = nearest_before(topic, stamp)
        if grid is None: continue
        value = dict(receive_age=(stamp-gt)/1e9, center_cost=cell_cost(grid,*point),
                     blockers=footprint_blockers(grid,point,heading), resolution=grid.info.resolution)
        if 'arc_end' in item:
            value['arc_end_blockers'] = footprint_blockers(grid,item['arc_end'],item['arc_end_heading'])
        pts=[]
        for i,cost in enumerate(grid.data):
            if cost==100:
                x=grid.info.origin.position.x+(i%grid.info.width+.5)*grid.info.resolution
                y=grid.info.origin.position.y+(i//grid.info.width+.5)*grid.info.resolution
                dx,dy=x-point[0],y-point[1]
                f,l=math.cos(heading)*dx+math.sin(heading)*dy,-math.sin(heading)*dx+math.cos(heading)*dy
                if abs(f)<5 and abs(l)<3: pts.append([f,l,x,y])
        value['nearby_lethal_cells'] = sorted(pts,key=lambda p:math.hypot(p[0],p[1]))[:25]
        item[label]=value
    ct,cloud=nearest_before('/wildos/points',stamp)
    if cloud:
        points=decode_xyz(cloud)
        near=points[(np.abs(points[:,0])<5)&(np.abs(points[:,1])<3)]
        item['marking_receive_age']=(stamp-ct)/1e9
        item['nearby_marking_points']=near[np.argsort(np.linalg.norm(near[:,:2],axis=1))][:25].tolist()
    report['collision_errors'].append(item)
(folder/'collision_analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
