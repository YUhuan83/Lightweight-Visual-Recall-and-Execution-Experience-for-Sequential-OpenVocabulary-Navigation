"""Replay the saved failed scan through each production filtering stage."""
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nav2_adapter'))
sys.path.insert(0,str(ROOT/'official_adapter'))
from wildos_nav2_adapter.pointcloud import filter_robot_footprint,filter_height,local_ground_plane,split_costmap_points
from beamng_wildos_adapter.geometry import quaternion_matrix
folder=Path(__file__).parent/'stability_frontier_trial5'
audit=json.loads((folder/'audit.json').read_text());failure=audit['failures'][0]
raw=np.load(folder/'187.97_raw_cloud.npz')['points']
offset=np.array([-2.172,.330,0.]);base=raw-offset
rotation=quaternion_matrix(failure['orientation']);rear=np.array([*failure['rear'],failure['height']])
def curb_roi(points):
    world=points@rotation.T+rear
    return ((world[:,0]>=725.3)&(world[:,0]<=727.)&(world[:,1]>=-15.)&(world[:,1]<=-12.)&(world[:,2]>=51.7)&(world[:,2]<=52.4))
outside=filter_robot_footprint(base,3.4,1.05,1.05)
height=filter_height(outside,.1,3.)
plane=local_ground_plane(outside,8.)
marking,clearing=split_costmap_points(base,min_height=.1,max_height=3.,filter_footprint=True,
    front_extent=3.4,rear_extent=1.05,half_width=1.05,voxel=.2,max_points=20000,
    ground_plane_radius=8.,ground_clearance=.25)
def stats(points):
    selected=points[curb_roi(points)]
    result={'scan_points':len(points),'curb_region_points':len(selected)}
    if len(selected):
        result['base_z_p05_p50_p95']=np.percentile(selected[:,2],[5,50,95]).tolist()
        if plane is not None:
            height_above_road=selected[:,2]-selected[:,:2]@plane[:2]-plane[2]
            result['road_clearance_p05_p50_p95']=np.percentile(height_above_road,[5,50,95]).tolist()
    return result
result={'region_world':{'x':[725.3,727.],'y':[-15.,-12.],'z':[51.7,52.4]},
    'road_plane_base':plane.tolist() if plane is not None else None,
    'stages':{k:stats(v) for k,v in [('input',base),('outside_body',outside),('absolute_height_pass',height),('marking',marking),('clearing',clearing)]},
    'limitation':'The region contains both road and curb returns; does not label every point as curb. Saved cloud and marking stamps differ slightly.'}
(folder/'low_curb_filter_replay.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
