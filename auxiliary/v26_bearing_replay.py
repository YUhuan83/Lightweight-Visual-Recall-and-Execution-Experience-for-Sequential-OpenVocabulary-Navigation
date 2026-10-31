"""Replay recorded masks/poses through bearing geometry, without inventing LiDAR."""
import collections,hashlib,json,math,re,sqlite3
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from visual_navigation.utils.semantic_mask import largest_mask_component
from visual_navigation.explorfm_triangulation.localization_geometry import triangulate_bearings

root=Path('/home/ly/execution_continuity_review_20261022/v26_body_arrival')
config=yaml.safe_load((root/'candidate/official_adapter/config/beamng_triangulation.yaml').read_text())
def stamp(x):return x.sec*10**9+x.nanosec
def explain(views):
    if len(views)<2:return dict(reason='fewer_than_two_views')
    origins=np.asarray([v['origin'] for v in views]);rays=np.asarray([v['ray'] for v in views])
    rays=rays/np.linalg.norm(rays,axis=1)[:,None]
    pairs=[(i,j) for i in range(len(views)) for j in range(i) if np.linalg.norm(origins[i]-origins[j])>=config['min_view_distance']]
    parallax=max((math.acos(float(np.clip(rays[i]@rays[j],-1,1))) for i,j in pairs),default=0.)
    out=dict(views=len(views),max_baseline_m=max((float(np.linalg.norm(a-b)) for a in origins for b in origins),default=0.),max_parallax_deg=math.degrees(parallax))
    if not pairs or parallax<.035:return dict(out,reason='no_baseline_and_parallax_pair')
    matrices=np.eye(3)[None,:,:]-rays[:,:,None]*rays[:,None,:];matrix=matrices.sum(axis=0)
    out['condition_number']=float(np.linalg.cond(matrix))
    if out['condition_number']>4000:return dict(out,reason='ill_conditioned')
    position=np.linalg.solve(matrix,np.sum(np.einsum('nij,nj->ni',matrices,origins),axis=0))
    vectors=position-origins;depths=np.sum(vectors*rays,axis=1)
    residual=np.linalg.norm(np.einsum('nij,nj->ni',matrices,vectors),axis=1)
    errors=np.arctan2(residual,depths)
    out.update(position=position.tolist(),min_depth_m=float(depths.min()),max_depth_m=float(depths.max()),max_reprojection_error_deg=float(np.degrees(errors.max())),median_ray_distance_m=float(np.median(residual)))
    if np.any(depths<=0):return dict(out,reason='intersection_behind_camera')
    if np.any(depths>100):return dict(out,reason='depth_over_100m')
    if np.any(errors>.04):return dict(out,reason='reprojection_error')
    if np.median(residual)>1.5:return dict(out,reason='ray_residual')
    covariance=np.linalg.inv(matrix)*max(.01,float(np.mean(residual**2)))
    out['radius_m']=float(np.sqrt(np.max(np.linalg.eigvalsh(covariance[:2,:2]))))
    return dict(out,reason='uncertainty_radius' if out['radius_m']>3 else 'passed')

reports=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'));session=goal.parent
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    def messages(topic):
        tid,typ=topics[topic];cls=get_message(typ)
        return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    timings=[json.loads(m.data) for _,m in messages('/spot1/measurement_timing')]
    sync=[d for d in timings if d.get('sensor')=='mask_lidar_sync']
    paired={d['mask_stamp_ns']:d for d in sync if d['status']=='paired'}
    localizations={d['observation_stamp_ns']:d for _,m in messages('/spot1/object_localization') for d in [json.loads(m.data)]}
    anchor=np.asarray(next(d['position'] for d in localizations.values() if d.get('localization_valid')))
    masks=messages('/spot1/object_mask');db.close()
    log=(session/'triangulation.log').read_text()
    views=[];previous=None;rows=[]
    for recorded,m in masks:
        t=stamp(m.header.stamp)
        shape=[d.size for d in m.object_mask.layout.dim]
        raw=np.asarray(m.object_mask.data[m.object_mask.layout.data_offset:],dtype=np.uint8).reshape(shape)
        raw=raw[:,0,:,:]
        masks_by_camera=[largest_mask_component(a.astype(bool)) for a in raw]
        camera=max(range(len(masks_by_camera)),key=lambda i:np.count_nonzero(masks_by_camera[i]))
        mask=masks_by_camera[camera]
        row=dict(stamp_ns=t,recorded_ns=recorded,positive_pixels=int(mask.sum()),paired=t in paired,
            source_age_at_pair_s=(paired[t]['received_ros_ns']-t)/1e9 if t in paired else None)
        p=m.odom.pose.pose.position
        position=np.asarray([p.x,p.y,p.z])
        row['robot_position']=position.tolist()
        if mask.any():
            tf=m.cam_transforms.transforms[camera].transform
            q=tf.rotation;o=tf.translation
            r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            k=np.asarray(m.cam_infos[camera].k).reshape(3,3)
            v,u=np.median(np.argwhere(mask),axis=0)
            row['camera_origin']=[o.x,o.y,o.z]
            row['ray']=(r@np.linalg.solve(k,[u,v,1.])).tolist()
            projected=r.T@(anchor-np.asarray([o.x,o.y,o.z]))
            uv=k@projected
            inside=False
            if projected[2]>.1:
                u0,v0=uv[:2]/uv[2]
                if 0<=u0<mask.shape[1] and 0<=v0<mask.shape[0]:inside=bool(mask[int(v0),int(u0)])
            row['first_recorded_proposal_inside_mask']=inside
        if t not in paired:row['reason']='not_paired';rows.append(row);continue
        old=len(views);views=[v for v in views if 0<=t-v['stamp_ns']<=config['observation_timeout_sec']*1e9]
        row['pruned_views']=old-len(views)
        if not views:previous=None
        outcome=localizations.get(t,{})
        if outcome.get('source')=='lidar' and outcome.get('localization_valid'):
            row['reason']='lidar_localized';rows.append(row);continue
        if outcome.get('reason')=='ambiguous_depth_surfaces':
            views=[];previous=None;row['reason']='depth_ambiguity_reset';rows.append(row);continue
        row['baseline_since_last_added_m']=float(np.linalg.norm(position-previous)) if previous is not None else None
        if previous is not None and np.linalg.norm(position-previous)<config['min_view_distance']:
            row['reason']='view_too_close';rows.append(row);continue
        if mask.any():
            tf=m.cam_transforms.transforms[camera].transform
            q=tf.rotation;o=tf.translation
            r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            k=np.asarray(m.cam_infos[camera].k).reshape(3,3)
            v,u=np.median(np.argwhere(mask),axis=0)
            ray=r@np.linalg.solve(k,[u,v,1.])
            views.append(dict(origin=np.asarray([o.x,o.y,o.z]),ray=ray,stamp_ns=t,angular_tolerance=.04))
            views[-1]['anchor_inside']=row['first_recorded_proposal_inside_mask']
        previous=position;views=views[-config['max_views']:]
        diagnostic=explain(views);valid=triangulate_bearings(views,config['min_view_distance']) is not None
        row['all_views_include_recorded_anchor']=len(views)>=2 and all(v.get('anchor_inside') for v in views)
        assert valid==(diagnostic['reason']=='passed'),diagnostic
        row.update(diagnostic);rows.append(row)
    report=dict(trial=str(trial),config=config,raw_mask_messages=len(masks),positive_masks=sum(r['positive_pixels']>0 for r in rows),
        sync_counts=dict(collections.Counter((d['status']+':'+d.get('reason','')) for d in sync)),
        logged_callbacks=len(re.findall('Received callback',log)),logged_too_close=len(re.findall('Current view is too close',log)),
        logged_insufficient_views=len(re.findall('Not enough views for triangulation',log)),logged_lidar_locks=len(re.findall('Object LOCK using LiDAR',log)),
        logged_multiview_successes=len(re.findall('Validated multi-view position',log)),
        replay_reasons=dict(collections.Counter(r['reason'] for r in rows)),rows=rows,
        caveat='Masks/poses replay only; recorded LiDAR outcomes supplied. Depth rejection gates are audited separately using raw paired clouds and verified odom transforms. Processing time/order/TF exceptions need log cross-check.')
    reports.append(report)
encoded=json.dumps(reports,sort_keys=True).encode();(root/'bearing_geometry_audit.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=[{k:v for k,v in r.items() if k not in ('rows','config')} for r in reports])))
