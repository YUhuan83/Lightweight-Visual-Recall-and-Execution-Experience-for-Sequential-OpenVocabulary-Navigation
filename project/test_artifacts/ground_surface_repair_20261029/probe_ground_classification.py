"""Compare segmentation prototypes against existing tests and recorded clouds."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
from patch_prototype import supported_ground
source=Path(sys.argv[1]);data=Path(sys.argv[2])
spec=importlib.util.spec_from_file_location('cloud',source);cloud=importlib.util.module_from_spec(spec);spec.loader.exec_module(cloud)
cloud.connected_ground_returns=lambda p,plane,r,h,s:supported_ground(p,plane,r,h,s)
cloud.supported_far_ground_returns=lambda p,plane,c,r,h,s:supported_ground(p,plane,r,h,s)
for f in sorted(data.glob('request_*__spot1_ouster_front_points_filtered.npz')):
    raw=np.load(f);p=raw['body'];w=raw['world'];serial=f.name.split('_')[1]
    marked,clear=cloud.split_costmap_points(p,min_height=.1,max_height=1.5,
        filter_footprint=True,front_extent=3.4,rear_extent=1.05,half_width=1.05,
        voxel=.2,max_points=20000,ground_plane_radius=8.,obstacle_min_height=.1)
    # Uniform translation/rotation preserves nearest-neighbor matching;
    # selected float32 output samples are original measured returns.
    from scipy.spatial import cKDTree
    tree=cKDTree(p);gap,index=tree.query(marked);assert np.max(gap)<1.e-5
    mw=w[index];region=(mw[:,0]>728)&(mw[:,0]<740)&(mw[:,1]>-82)&(mw[:,1]<-71)
    print(json.dumps(dict(request=serial,marks=len(marked),region_marks=int(region.sum()))))
