"""Isolated per-scan ground segmentation prototype, not deployed."""
import numpy as np

def supported_ground(points,plane,radius,obstacle_height,max_slope,fit):
    points=np.asarray(points,dtype=float)
    tolerance=min(.05,obstacle_height/2.)
    eligible=np.flatnonzero((points[:,2]>=-2.)&(points[:,2]<=1.2))
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
        if keep.sum()<12 or np.linalg.norm(value[:2])>.6 or np.percentile(abs(error[keep]),90)>.15:return None
        return value
    residual=support[:,2]-(support[:,:2]@plane[:2]+plane[2])
    ground=(np.linalg.norm(support[:,:2],axis=1)<=min(4.,radius))&(abs(residual)<tolerance)
    coefficients=np.tile(plane,(len(cells),1));best=abs(residual)
    tiles=np.unique(np.floor(support[:,:2]/2.).astype(np.int64),axis=0)
    centers=(tiles+.5)*2.
    centers=centers[np.argsort(np.sum(centers**2,axis=1),kind='stable')]
    neighborhoods=[]
    for center in centers:
        indices=np.flatnonzero(np.linalg.norm(support[:,:2]-center,axis=1)<=4.)
        if len(indices)>=12 and counts[indices].sum()>=100:neighborhoods.append(indices)
    while True:
        before=int(ground.sum())
        for indices in neighborhoods:
            known=indices[ground[indices]]
            prior=fitted(known)
            if prior is None:continue
            patch=support[indices]
            possible=abs(patch[:,2]-(patch[:,:2]@prior[:2]+prior[2]))<obstacle_height
            local=fitted(indices[possible])
            if local is None or np.linalg.norm(local[:2]-prior[:2])>max_slope:continue
            error=abs(patch[:,2]-(patch[:,:2]@local[:2]+local[2]))
            supported=possible&(error<tolerance)
            improved=indices[supported&(error<best[indices])]
            coefficients[improved]=local;best[improved]=error[supported&(error<best[indices])]
            ground[indices[supported]]=True
        if int(ground.sum())==before:break
    result=np.zeros(len(points),dtype=bool)
    value=coefficients[inverse]
    result[eligible]=ground[inverse]&(abs(p[:,2]-(np.sum(p[:,:2]*value[:,:2],axis=1)+value[:,2]))<tolerance)
    return result

if __name__=='__main__':
    import importlib.util,time,json,sys
    from pathlib import Path
    source=Path(sys.argv[1]);data=Path(sys.argv[2])
    spec=importlib.util.spec_from_file_location('cloud',source);cloud=importlib.util.module_from_spec(spec);spec.loader.exec_module(cloud)
    for f in sorted(data.glob('request_*__spot1_ouster_front_points_filtered.npz')):
        p=np.load(f)['body'];p=cloud.filter_robot_footprint(p,3.4,1.05,1.05)
        plane=cloud.local_ground_plane(p,8.)
        began=time.perf_counter();g=supported_ground(p,plane,8.,.1,.35,cloud.local_ground_plane)
        elapsed=time.perf_counter()-began
        print(json.dumps(dict(file=f.name,points=len(p),confirmed=int(g.sum()),elapsed_s=elapsed)))
        np.savez_compressed(f.with_name(f.stem+'_prototype.npz'),body=p,ground=g)
