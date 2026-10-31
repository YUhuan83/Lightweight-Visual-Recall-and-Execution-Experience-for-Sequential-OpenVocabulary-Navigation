"""Old/new real cloud replay; independent static geometry bounds the claims."""
import importlib.util,json,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import RegularGridInterpolator
root=Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback')
data=root/'request_audit'
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
old=module('old_cloud',root/'candidate/nav2_adapter/wildos_nav2_adapter/pointcloud.py')
new=module('new_cloud',root/'pointcloud_candidate.py')
samples=json.loads((root/'static_20261029_150013/surface_samples.json').read_text())
xs=sorted(set(r['x'] for r in samples));ys=sorted(set(r['y'] for r in samples))
z=np.array([r['staticZ'] for r in samples]).reshape(len(xs),len(ys));height=RegularGridInterpolator((xs,ys),z,bounds_error=False,fill_value=np.nan)
dx,dy=np.gradient(z,.25,.25);slope=RegularGridInterpolator((xs,ys),np.hypot(dx,dy),bounds_error=False,fill_value=np.nan)
options=dict(min_height=.1,max_height=1.5,filter_footprint=True,front_extent=3.4,rear_extent=1.05,half_width=1.05,voxel=.2,max_points=20000,ground_plane_radius=8.,obstacle_min_height=.1)
report=[]
for f in sorted(data.glob('request_*__spot1_ouster_front_points_filtered.npz')):
    raw=np.load(f);p=raw['body'];w=raw['world'];within=np.linalg.norm(p,axis=1)<=40.;p=p[within];w=w[within];tree=cKDTree(p);runs=[];clears=[]
    for label,cloud in [('old',old),('new',new)]:
        for repeat in range(2):
            began=time.perf_counter();marked,clearing=cloud.split_costmap_points(p,**options);elapsed=time.perf_counter()-began
            gap,index=tree.query(marked);assert np.max(gap)<1.e-5;world=w[index]
            region=(world[:,0]>728)&(world[:,0]<740)&(world[:,1]>-82)&(world[:,1]<-71)
            dz=world[:,2]-height(world[:,:2]);gradient=slope(world[:,:2])
            smooth=region&(np.abs(dz)<.05)&(gradient<.35)
            raised=region&(dz>=.10)
            runs.append(dict(version=label,repeat=repeat,elapsed_sec=elapsed,marks=len(marked),region_marks=int(region.sum()),smooth_static_marks=int(smooth.sum()),raised_above_static_marks=int(raised.sum()),raised_examples=world[raised].tolist()))
            clears.append(clearing)
            np.savez_compressed(data/f'{f.stem}_{label}_{repeat}_result.npz',marked_body=marked,marked_world=world,clearing=clearing)
    assert all(np.array_equal(clears[0],c) for c in clears)
    report.append(dict(file=f.name,runs=runs,clearing_exactly_equal=True))
(data/'segmentation_replay.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
