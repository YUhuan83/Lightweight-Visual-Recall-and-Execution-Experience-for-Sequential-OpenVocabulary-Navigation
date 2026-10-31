"""Endpoint-only counterfactual on the preserved first failed snapshot."""
import json,math,sys
from pathlib import Path
import numpy as np
root=Path(__file__).parent.parent/'拟真/test_artifacts/integrated_progress_20261025/candidate'
sys.path[:0]=[str(root/'tools'),str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from test_confirmed_progress_ros import ConfirmedProgressTests
from beamng_wildos_adapter.geometry import quaternion_matrix
from beamng_wildos_adapter.mapping import safe_travel_distance
test=ConfirmedProgressTests();test.setUp();meta,target=test.snapshot();node=test.node
pose=node.occupancy_odom.pose.pose;q=pose.orientation
R=quaternion_matrix([q.x,q.y,q.z,q.w]);origin=np.array([pose.position.x,pose.position.y])
world=np.array([target.pose.position.x-pose.position.x,target.pose.position.y-pose.position.y,0.])
local=world@R;radius=node.approach_distance()[0]
grid=np.asarray(node.occupancy.data).reshape(node.occupancy.info.height,node.occupancy.info.width)
info=node.occupancy.info;base=math.atan2(local[1],local[0]);accept=node.observation_pose_filter(target=meta['confirmation']['object_position'][:2])
rows=[]
for degrees in (0,15,-15,30,-30,45,-45,60,-60,75,-75,90,-90,105,-105,120,-120,135,-135,150,-150,165,-165,180):
    angle=base+math.pi+math.radians(degrees)
    point=local[:2]+radius*np.array([math.cos(angle),math.sin(angle)])
    projected=origin+(R@np.array([point[0],point[1],0.]))[:2]
    safe=safe_travel_distance(grid,info.resolution,(info.origin.position.x,info.origin.position.y),point,np.linalg.norm(point),2.,max(info.resolution,.5))
    x=math.floor((point[0]-info.origin.position.x)/info.resolution);y=math.floor((point[1]-info.origin.position.y)/info.resolution)
    value=int(grid[y,x]) if 0<=x<info.width and 0<=y<info.height else None
    rows.append(dict(angle=degrees,point=projected.tolist(),global_endpoint_accept=bool(accept(projected)),local_endpoint_value=value,ray_safe_m=float(safe),ray_required_m=float(np.linalg.norm(point))))
test.tearDown()
report=dict(snapshot='v28 trial_20261025_200844 first confirmed target',accepted_endpoints=sum(r['global_endpoint_accept'] for r in rows),rows=rows)
out=root.parent/'ring_endpoint_probe.json';out.write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
