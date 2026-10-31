"""Export fixed-source cloud inputs and independent surface checks."""
exec(compile(__import__('pathlib').Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback/review_recorded_ground.py').read_text().split('report=[]')[0],'<replay setup>','exec'))
out=root/'request_audit/regression';out.mkdir(exist_ok=True)
for serial in (24,34):
    raw=np.load(data/f'request_{serial}__spot1_ouster_front_points_filtered.npz');p=raw['body'];w=raw['world']
    within=np.linalg.norm(p,axis=1)<=40.;p=p[within];w=w[within]
    region=(w[:,0]>728)&(w[:,0]<740)&(w[:,1]>-82)&(w[:,1]<-71)
    dz=w[:,2]-height(w[:,:2]);smooth=region&(abs(dz)<.05)&(slope(w[:,:2])<.35)
    old_marks,_=old.split_costmap_points(p,**options);_,indices=cKDTree(p).query(old_marks)
    marked=np.zeros(len(p),bool);marked[indices]=True
    np.savez_compressed(out/f'request_{serial}.npz',points=p.astype(np.float32),old_smooth_marks=smooth&marked,old_raised_marks=region&(dz>=.1)&marked)
print(out)
