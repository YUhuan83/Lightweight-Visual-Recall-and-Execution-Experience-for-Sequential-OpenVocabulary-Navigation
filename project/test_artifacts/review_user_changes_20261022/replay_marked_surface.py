"""Replay actual source clouds twice, compare body and level classification."""
from pathlib import Path
import json,sys
import numpy as np
from wildos_nav2_adapter.pointcloud import split_costmap_points,local_ground_plane

prefix=Path(__file__).with_name('trace_point_provenance.py').read_text().split("report={'trial':")[0]
exec(compile(prefix,'trace_point_prefix','exec'))
evidence=json.loads((trial/'point_provenance_nearby.json').read_text())
marked=next(e for e in evidence['evidence'] if e['topic']=='/wildos/points')
target=(origin+marked['source_elapsed'])*1e9
stamp,msg=min(rows('/spot1/ouster/front/points_filtered'),key=lambda row:abs(source_stamp(row[1])-target))
ns=source_stamp(msg);assert abs(ns-target)<1000
T,age=world(msg.header.frame_id,ns);base,base_age=world('spot1/base_link',ns)
w=decode_xyz(msg)@T[:3,:3].T+T[:3,3]
body=(w-base[:3,3])@base[:3,:3]
body=body[np.linalg.norm(body,axis=1)<=40].astype(np.float32)
yaw=np.arctan2(base[1,0],base[0,0]);c,s=np.cos(yaw),np.sin(yaw)
level=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
kwargs=dict(min_height=.1,max_height=1.5,filter_footprint=True,front_extent=3.4,rear_extent=1.05,half_width=1.05,voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25,obstacle_min_height=.1,ground_max_slope=.35)
def counts(p,R):
    marks,clears=split_costmap_points(p,**kwargs)
    wm=marks@R.T+base[:3,3]
    mask=(wm[:,0]>=736.5)&(wm[:,0]<738.25)&(wm[:,1]>=-27.25)&(wm[:,1]<-26.)
    return wm[mask],local_ground_plane(p,8.)
results=[]
for repeat in (1,2):
    original,plane=counts(body,base[:3,:3])
    assert len(original)==marked['points'],(len(original),marked['points'])
    observed=np.array(marked['world_points'])
    error=max(float(np.linalg.norm(original-p,axis=1).min()) for p in observed)
    assert error<1e-4,error
    level_points=(body@base[:3,:3].T@level).astype(np.float32)
    alternative,level_plane=counts(level_points,level)
    bp=(observed-base[:3,3])@base[:3,:3]
    residual=bp[:,2]-(bp[:,0]*plane[0]+bp[:,1]*plane[1]+plane[2]) if plane is not None else np.array([np.nan])
    results.append(dict(repeat=repeat,source_elapsed=ns/1e9-origin,original_marks=len(original),level_marks=len(alternative),recorded_match_max_error_m=error,body_plane=plane.tolist() if plane is not None else None,mark_residual_to_near_ground_plane=[float(residual.min()),float(residual.max())],world_z_span=float(np.ptp(observed[:,2]))))
report=dict(trial=str(trial),source_tf_age=age,base_tf_age=base_age,results=results,scope='same acquired cloud and source pose, comparison only; level-frame reclassification is diagnostic, not a proposed global ground model')
(trial/'marked_surface_replay.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
