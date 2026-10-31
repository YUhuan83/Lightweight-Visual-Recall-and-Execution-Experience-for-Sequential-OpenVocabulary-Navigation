"""Compare recorded collision poses, padded raster footprints and marking clouds."""
from pathlib import Path
import json
import math
import sys
from types import SimpleNamespace as NS
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nav2_adapter'))
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.pointcloud import local_ground_plane
from wildos_nav2_adapter.graph_route import footprint_blockers

folder=Path(__file__).resolve().parent/'failure_audit/house_geometry_rviz_attempt2'
audit=json.loads((folder/'audit.json').read_text())
result={'collision_replay':[],'clouds':[]}

def pixels(a,b):
    # Match nav2_util::LineIterator's integer division and tie handling.
    x,y=map(int,a);tx,ty=map(int,b);dx,dy=abs(tx-x),abs(ty-y)
    xi1=xi2=1 if tx>=x else -1;yi1=yi2=1 if ty>=y else -1
    if dx>=dy:xi1=0;yi2=0;den=dx;num=dx//2;add=dy;count=dx
    else:xi2=0;yi1=0;den=dy;num=dy//2;add=dx;count=dy
    for _ in range(count+1):
        yield x,y
        num+=add
        if num>=den:num-=den;x+=xi1;y+=yi1
        x+=xi2;y+=yi2

def raster_footprint(grid,position,heading,padding=.01):
    data=grid['data'].reshape(*grid['shape']);origin=grid['origin'];res=float(grid['resolution'])
    c,s=math.cos(heading),math.sin(heading)
    body=np.array([[3.4+padding,1.05+padding],[3.4+padding,-1.05-padding],[-1.05-padding,-1.05-padding],[-1.05-padding,1.05+padding]])
    world=body@np.array([[c,s],[-s,c]])+position
    corners=np.floor((world-origin)/res).astype(int)
    blocked=[]
    for a,b in zip(corners,np.roll(corners,-1,axis=0)):
        for x,y in pixels(tuple(a),tuple(b)):
            if not (0<=y<data.shape[0] and 0<=x<data.shape[1]):continue
            cost=int(data[y,x])
            if cost>=254:blocked.append({'cell':[int(x),int(y)],'world':(origin+(np.array([x,y])+.5)*res).tolist(),'cost':cost})
    return list({tuple(e['cell']):e for e in blocked}.values())

for failure in audit['failures']:
    stem=str(round(failure['elapsed'],2));point=failure['rear'];heading=failure['heading']
    if failure['status']['status']=='stale_costmap':continue
    row={'elapsed':failure['elapsed'],'current':{},'arc_end':{}}
    for label in ['global_raw','local_raw','global_raw_after','local_raw_after']:
        grid=np.load(folder/(stem+'_'+label+'.npz'))
        row['current'][label]=raster_footprint(grid,point,heading)
        if failure.get('last_arc') and failure['last_arc']['age']<.2:
            arc=failure['last_arc'];pts=arc['points'];h=arc['end_heading_approx']
            if len(pts)>=3:
                ha=math.atan2(pts[-2][1]-pts[-3][1],pts[-2][0]-pts[-3][0])
                hb=math.atan2(pts[-1][1]-pts[-2][1],pts[-1][0]-pts[-2][0])
                h+=math.atan2(math.sin(hb-ha),math.cos(hb-ha))
            row['arc_end'][label]={'position':pts[-1],'heading_inferred':h,
                'blockers':raster_footprint(grid,pts[-1],h),
                'unpadded_raster_blockers':raster_footprint(grid,pts[-1],h,padding=0.)}
            if label in ('global_raw','local_raw'):
                occupancy=np.load(folder/(stem+'_'+label.replace('_raw','')+'.npz'))
                g=NS(data=occupancy['data'].tolist(),info=NS(width=int(occupancy['shape'][1]),height=int(occupancy['shape'][0]),
                    resolution=float(occupancy['resolution']),origin=NS(position=NS(x=occupancy['origin'][0],y=occupancy['origin'][1]),
                    orientation=NS(x=0.,y=0.,z=0.,w=1.))))
                row['arc_end'][label]['production_sample_blockers']=footprint_blockers(g,pts[-1],h)
    result['collision_replay'].append(row)
    raw=np.load(folder/(stem+'_raw_cloud.npz'))['points']
    # The official virtual lidar frame preserves the old StateSensor reference.
    rawbase=raw+np.array([2.172,-.330,0.])
    mark=np.load(folder/(stem+'_marking.npz'))['points']
    meta=failure['marking']['odom'];rot=np.asarray(matrix_from_quaternion(meta['orientation']))
    world=mark@rot.T+meta['position']
    near=mark[np.hypot(mark[:,0],mark[:,1])<8]
    plane=local_ground_plane(rawbase,8)
    row={'elapsed':failure['elapsed'],'plane':None if plane is None else plane.tolist(),
         'near_marking_count':len(near),'height_range':None if not len(near) else [float(near[:,2].min()),float(near[:,2].max())]}
    if plane is not None and len(near):
        residual=near[:,2]-(near[:,0]*plane[0]+near[:,1]*plane[1]+plane[2])
        row['road_relative_height_range']=[float(residual.min()),float(residual.max())]
        row['near_marking_below_ground_clearance']=int(sum(residual<.25))
    np.savez_compressed(folder/(stem+'_world_marking.npz'),points=world)
    result['clouds'].append(row)
(folder/'geometry_replay.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
