"""Measured overlapping surface patches; diagnostic prototype only."""
from collections import deque
import numpy as np

def supported_ground(points,plane,radius,obstacle_height,max_slope,fit=None):
    points=np.asarray(points,dtype=float);tolerance=min(.05,obstacle_height/2.)
    eligible=np.flatnonzero((points[:,2]>=-2.)&(points[:,2]<=1.2))
    result=np.zeros(len(points),dtype=bool)
    if not len(eligible):return result
    p=points[eligible];order=np.lexsort((p[:,2],p[:,1],p[:,0]));p=p[order];eligible=eligible[order]
    cells,inverse,counts=np.unique(np.floor(p[:,:2]/.5).astype(np.int64),axis=0,return_inverse=True,return_counts=True)
    lows=np.full(len(cells),np.inf);np.minimum.at(lows,inverse,p[:,2])
    selected=np.full(len(cells),len(p),dtype=np.int64);minimum=p[:,2]==lows[inverse]
    np.minimum.at(selected,inverse[minimum],np.flatnonzero(minimum));support=p[selected]
    def fitted(indices):
        if len(indices)<12 or counts[indices].sum()<100:return None
        samples=support[indices];matrix=np.c_[samples[:,:2],np.ones(len(samples))]
        keep=np.ones(len(indices),dtype=bool)
        for _ in range(4):
            if keep.sum()<12:return None
            value,_,rank,_=np.linalg.lstsq(matrix[keep],samples[keep,2],rcond=None)
            if rank<3:return None
            error=samples[:,2]-matrix@value
            keep=(error>=np.quantile(error,.05)-.05)&(error<=np.quantile(error,.65)+.05)
        if keep.sum()<12 or np.linalg.norm(value[:2])>.6 or np.percentile(abs(error),90)>tolerance:return None
        return value
    residual=support[:,2]-(support[:,:2]@plane[:2]+plane[2])
    seed=(np.linalg.norm(support[:,:2],axis=1)<=min(4.,radius))&(abs(residual)<tolerance)
    patches=[];members=[[] for _ in cells]
    tiles=np.unique(np.floor(support[:,:2]/2.).astype(np.int64),axis=0)
    for tile in tiles:
        center=(tile+.5)*2.;indices=np.flatnonzero(np.linalg.norm(support[:,:2]-center,axis=1)<=4.)
        local=fitted(indices)
        if local is None:continue
        error=abs(support[indices,2]-(support[indices,:2]@local[:2]+local[2]))
        matched=indices[error<tolerance]
        patches.append((matched,local))
        for i in matched:members[i].append(len(patches)-1)
    known=seed.copy();queue=deque(np.flatnonzero(seed));activated=np.zeros(len(patches),dtype=bool)
    count=np.zeros(len(patches),dtype=int)
    coefficients=np.tile(plane,(len(cells),1));best=abs(residual)
    while queue:
        index=queue.popleft()
        for patch_id in members[index]:
            if activated[patch_id]:continue
            count[patch_id]+=1
            if count[patch_id]<12:continue
            matched,local=patches[patch_id]
            shared=matched[known[matched]]
            if np.linalg.matrix_rank(np.c_[support[shared,:2],np.ones(len(shared))])<3:continue
            if np.any(np.linalg.norm(coefficients[shared,:2]-local[:2],axis=1)>max_slope):continue
            activated[patch_id]=True
            error=abs(support[matched,2]-(support[matched,:2]@local[:2]+local[2]))
            improve=error<best[matched];coefficients[matched[improve]]=local;best[matched[improve]]=error[improve]
            new=matched[~known[matched]];known[new]=True;queue.extend(new)
    value=coefficients[inverse]
    result[eligible]=known[inverse]&(abs(p[:,2]-(np.sum(p[:,:2]*value[:,:2],axis=1)+value[:,2]))<tolerance)
    return result

if __name__=='__main__':
    import importlib.util,time,json,sys
    from pathlib import Path
    source=Path(sys.argv[1]);data=Path(sys.argv[2])
    spec=importlib.util.spec_from_file_location('cloud',source);cloud=importlib.util.module_from_spec(spec);spec.loader.exec_module(cloud)
    for f in sorted(data.glob('request_*__spot1_ouster_front_points_filtered.npz')):
        p=np.load(f)['body'];p=cloud.filter_robot_footprint(p,3.4,1.05,1.05);plane=cloud.local_ground_plane(p,8.)
        began=time.perf_counter();g=supported_ground(p,plane,8.,.1,.35)
        print(json.dumps(dict(file=f.name,points=len(p),confirmed=int(g.sum()),elapsed_s=time.perf_counter()-began)))
        np.savez_compressed(f.with_name(f.stem+'_patch.npz'),body=p,ground=g)

