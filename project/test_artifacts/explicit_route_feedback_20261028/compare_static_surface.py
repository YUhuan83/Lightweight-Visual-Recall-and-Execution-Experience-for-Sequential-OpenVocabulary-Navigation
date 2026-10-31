"""Compare closed LiDAR observations against an independent static-world survey."""
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import RegularGridInterpolator
from wildos_nav2_adapter.pointcloud import local_ground_plane,filter_robot_footprint
root=Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback')
out=root/'request_audit'
rows=json.loads((root/'static_20261029_150013/surface_samples.json').read_text())
xs=sorted(set(r['x'] for r in rows));ys=sorted(set(r['y'] for r in rows))
surface=np.array([r['staticZ'] for r in rows]).reshape(len(xs),len(ys))
terrain=np.array([r['terrainZ'] for r in rows]).reshape(len(xs),len(ys))
height=RegularGridInterpolator((xs,ys),surface,bounds_error=False,fill_value=np.nan)
ground=RegularGridInterpolator((xs,ys),terrain,bounds_error=False,fill_value=np.nan)
gx,gy=np.gradient(surface,.25,.25);slope=RegularGridInterpolator((xs,ys),np.hypot(gx,gy),bounds_error=False,fill_value=np.nan)
summary=[]
for serial in (24,34):
    marks=np.load(out/f'request_{serial}__wildos_points.npz');p=marks['body'];w=marks['world']
    raw=np.load(out/f'request_{serial}__spot1_ouster_front_points_filtered.npz')['body']
    raw=filter_robot_footprint(raw,3.4,1.05,1.05);plane=local_ground_plane(raw,8.)
    low=(w[:,0]>728)&(w[:,0]<740)&(w[:,1]>-82)&(w[:,1]<-71)
    gap=w[:,2]-height(w[:,:2]);s=slope(w[:,:2])
    road=low&(np.abs(gap)<.05)&(s<.35)
    residual=p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@plane
    summary.append(dict(request=serial,plane=plane.tolist(),region_marks=int(low.sum()),marks_on_smooth_static_surface=int(road.sum()),examples=[dict(world=w[i].tolist(),body=p[i].tolist(),surface_gap=float(gap[i]),static_slope=float(s[i]),range=float(np.linalg.norm(p[i,:2])),plane_residual=float(residual[i])) for i in np.flatnonzero(road)[::max(1,int(road.sum())//5)][:5]],scope='Smooth static first-hit agreement, not semantic asset labels or complete vehicle traversability'))
(out/'static_comparison.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
