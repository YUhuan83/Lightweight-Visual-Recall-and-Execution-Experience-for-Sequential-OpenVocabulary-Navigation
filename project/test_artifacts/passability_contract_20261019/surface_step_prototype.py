"""Diagnostic witnessed height discontinuities; not a complete ground/free-space model."""
import numpy as np


def witnessed_steps(points, *, max_step=.1, max_slope=.35, resolution=.1, minimum_returns=2):
    points=np.asarray(points,dtype=np.float32)
    if not len(points):return points.copy()
    cells,inverse,counts=np.unique(np.floor(points[:,:2]/resolution).astype(np.int64),axis=0,return_inverse=True,return_counts=True)
    count=len(cells)
    minimum=np.full(count,np.inf);maximum=np.full(count,-np.inf)
    np.minimum.at(minimum,inverse,points[:,2]);np.maximum.at(maximum,inverse,points[:,2])
    lo=np.full(count,len(points),dtype=np.int64);hi=lo.copy()
    is_lo=points[:,2]==minimum[inverse];is_hi=points[:,2]==maximum[inverse]
    np.minimum.at(lo,inverse[is_lo],np.flatnonzero(is_lo));np.minimum.at(hi,inverse[is_hi],np.flatnonzero(is_hi))
    supported=counts>=minimum_returns
    marked=np.zeros(len(points),dtype=bool)
    def compare(high_group,low_group):
        high=hi[high_group];low=lo[low_group]
        separation=np.linalg.norm(points[high,:2]-points[low,:2],axis=1)
        violation=(supported[high_group]&supported[low_group]&
                   ((points[high,2]-points[low,2])>max_step+max_slope*separation+1e-4))
        marked[high[violation]]=True
    indices=np.arange(count);compare(indices,indices)
    minimum_cell=cells.min(axis=0);stride=int(cells[:,1].max()-minimum_cell[1]+3)
    packed=(cells[:,0]-minimum_cell[0])*stride+(cells[:,1]-minimum_cell[1])
    assert np.all(np.diff(packed)>0)
    for offset in ((0,1),(1,-1),(1,0),(1,1)):
        target=cells+offset
        target_packed=(target[:,0]-minimum_cell[0])*stride+(target[:,1]-minimum_cell[1])
        positions=np.searchsorted(packed,target_packed)
        possible=positions<count
        a=indices[possible];b=positions[possible]
        exact=(cells[b]==target[a]).all(axis=1)
        a,b=a[exact],b[exact]
        compare(a,b);compare(b,a)
    return points[marked]


if __name__=='__main__':
    from pathlib import Path
    import json,sys,time
    root=Path(__file__).resolve().parents[2];out=Path(__file__).resolve().parent
    sys.path.insert(0,str(root/'nav2_adapter'))
    from wildos_nav2_adapter.pointcloud import filter_robot_footprint
    from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH,SBR_OBSTACLE_MAX_HEIGHT
    arrays=np.load(out/'replay_inputs.npz',allow_pickle=False)
    manifest=json.loads((out/'input_manifest.json').read_text(encoding='utf8'))
    xy=np.array([(x,y) for x in np.arange(.03,7.5,.04) for y in np.arange(-2.97,3.,.04)])
    controls={}
    for name,z in dict(plane=.24*xy[:,0]+.24*xy[:,1]-.45,
                       convex=.015*xy[:,0]**2-.5,
                       crest=-.015*(xy[:,0]-4.)**2-.4,
                       curb=np.where(xy[:,0]>=4.,.17,0.)-.5).items():
        controls[name]=len(witnessed_steps(np.c_[xy,z]))
    rows=[]
    for sample in manifest['snapshots']:
        key=sample['key'];points=arrays[key+'_body']
        points=filter_robot_footprint(points,SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH)
        points=points[points[:,2]<=SBR_OBSTACLE_MAX_HEIGHT]
        begin=time.perf_counter();marked=witnessed_steps(points);elapsed=time.perf_counter()-begin
        close=np.linalg.norm(marked[:,:2]-arrays[key+'_contact_body'][:2],axis=1)<=.6
        rows.append(dict(key=key,engine_seconds=sample['engine_seconds'],marked=len(marked),
                         contact_neighborhood_marked=int(close.sum()),elapsed_ms=elapsed*1000.))
    report=dict(scope=__doc__,controls=controls,scans=rows,production_modified=False,complete_navigation_passed=False)
    (out/'surface_step_prototype.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,indent=2))
