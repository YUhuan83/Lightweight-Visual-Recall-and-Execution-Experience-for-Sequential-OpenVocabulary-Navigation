import importlib.util,sys,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
root=Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback');data=root/'request_audit'
spec=importlib.util.spec_from_file_location('new_cloud',root/'pointcloud_candidate.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
raw=np.load(data/'request_24__spot1_ouster_front_points_filtered.npz');p=raw['body'];w=raw['world']
mask=(np.linalg.norm(p,axis=1)<=40)&~((p[:,0]>=-1.05)&(p[:,0]<=3.4)&(abs(p[:,1])<=1.05));p=p[mask];w=w[mask]
target=np.array([738.2435300777913,-79.62799043431303,52.822536491613704]);_,index=cKDTree(w).query(target)
plane=m.local_ground_plane(p,8.);captured={}
def trace(frame,event,arg):
    if frame.f_code.co_name=='supported_ground_patches' and event=='return':captured.update(frame.f_locals)
    return trace
sys.settrace(trace);result=m.supported_ground_patches(p,plane,8.,.1,.35);sys.settrace(None)
slot=int(np.flatnonzero(captured['eligible']==index)[0]);cell=captured['inverse'][slot]
coeff=captured['coefficients'][cell];known=captured['known'];support=captured['support']
print(json.dumps(dict(world=w[index].tolist(),body=p[index].tolist(),ground=bool(result[index]),plane=plane.tolist(),accepted_plane=coeff.tolist(),cell_minimum=support[cell].tolist(),residual=float(p[index,2]-np.r_[p[index,:2],1.]@coeff))))
for patch_id,(matched,local) in enumerate(captured['patches']):
    if cell in matched and captured['activated'][patch_id]:
        centered=support[matched,:2]-support[matched,:2].mean(axis=0)
        print(json.dumps(dict(patch=patch_id,plane=local.tolist(),cells=len(matched),shared=int(known[matched].sum()),xy_singular=np.linalg.svd(centered,compute_uv=False).tolist(),support=support[matched].tolist())))
