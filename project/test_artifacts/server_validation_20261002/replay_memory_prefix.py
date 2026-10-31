"""Replay the accepted memory prefix against recorded and held costmaps; no motion."""
from pathlib import Path
import json,sys,subprocess,sqlite3,math
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import validate_vehicle_path,footprint_blockers,cell_cost,distance
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
out=ROOT/'mechanism_audit'
mechanism=json.loads((out/'mechanism.json').read_text())
paths=mechanism['bag']['/wildos/full_planned_path']
db=sqlite3.connect('file:'+str(ROOT/'body_collision_full_trial1/rosbag/rosbag_0.db3')+'?mode=ro',uri=True)
topics={name:(tid,typ) for tid,name,typ in db.execute('select id,name,type from topics')}
def latest(topic,ts):
    tid,typ=topics[topic]
    entry=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,ts)).fetchone()
    return deserialize_message(entry[1],get_message(typ)),entry[0]
def inspect(path,grid,odom,route):
    reason=validate_vehicle_path(path['points'],path['headings'],route,grid,path['points'][-1],
        start=xy(odom.pose.pose),minimum_turning_radius=8.,heading_quantization_bins=72)
    collisions=[]
    for index,(p,h) in enumerate(zip(path['points'],path['headings'])):
        b=footprint_blockers(grid,p,h,limit=8)
        if b:collisions.append(dict(index=index,point=p,heading=h,blockers=b,center_cost=cell_cost(grid,*p)))
    return dict(validation=reason,start=list(xy(odom.pose.pose)),heading=yaw(odom.pose.pose),
        grid_origin=[grid.info.origin.position.x,grid.info.origin.position.y],start_blockers=footprint_blockers(grid,xy(odom.pose.pose),yaw(odom.pose.pose)),
        collisions=collisions)
report={'accepted_prefixes':[],'graph_routes':[]}
for p in paths:
    grid,tg=latest('/global_costmap/costmap',p['timestamp'])
    odom,to=latest('/spot1/odom',p['timestamp'])
    graph,tr=latest('/spot1/graphnav_planner/path',p['timestamp'])
    route=[xy(odom.pose.pose)]+[xy(ps.pose) for ps in graph.poses]
    recorded=inspect(p,grid,odom,route)
    lastgrid,_=latest('/global_costmap/costmap',2**63-1)
    lastodom,_=latest('/spot1/odom',2**63-1)
    held=inspect(p,lastgrid,lastodom,route)
    report['accepted_prefixes'].append(dict(path=p,recorded=recorded,held=held,
        grid_age=(p['timestamp']-tg)/1e9,odom_age=(p['timestamp']-to)/1e9,
        path_length=sum(distance(a,b) for a,b in zip(p['points'],p['points'][1:])),
        path_first_pose_offset=distance(p['points'][0],xy(odom.pose.pose))))
    # Export exact CDR inputs for reproducible offline validation.
    import numpy as np
    np.savez_compressed(out/'accepted_memory_prefix_snapshot.npz',points=p['points'],headings=p['headings'],
        grid=np.array(grid.data).reshape(grid.info.height,grid.info.width),
        origin=[grid.info.origin.position.x,grid.info.origin.position.y],resolution=grid.info.resolution,
        held_grid=np.array(lastgrid.data).reshape(lastgrid.info.height,lastgrid.info.width),
        held_origin=[lastgrid.info.origin.position.x,lastgrid.info.origin.position.y],route=route,start=xy(odom.pose.pose))
for p in mechanism['bag']['/spot1/graphnav_planner/path']:
    report['graph_routes'].append(dict(after_switch=p['after_switch'],points=p['points']))
# Compact initial and later route changes.
(out/'prefix_replay.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
db.close()

