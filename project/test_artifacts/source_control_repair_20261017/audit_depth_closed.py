"""Diagnose exact paired closed masks/LiDAR and deterministic bearing gates."""
import json,math,sqlite3,sys,collections
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from scipy.spatial.transform import Rotation
import cv2
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
from tf2_ros.buffer import Buffer

source=Path('/home/ly/source_control_repair_20261017/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation')
sys.path.insert(0,str(source))
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth,triangulate_bearings
from visual_navigation.utils.semantic_mask import largest_mask_component
root=Path('/home/ly/source_control_repair_20261017')
s=root/'field_results/session_20261017_151623'
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
goal=sqlite3.connect((s/'goal_001/interfaces/interfaces_0.db3').as_uri()+'?mode=ro',uri=True)
raw=sqlite3.connect((s/'sensor_interfaces/sensor_interfaces_0.db3').as_uri()+'?mode=ro',uri=True)
origin=1792221435.6587815
def ns(stamp):return stamp.sec*10**9+stamp.nanosec
def read(c,name,low=0,high=2**63-1):
    topic=c.execute('select id,type from topics where name=?',(name,)).fetchone()
    if topic is None:return
    cls=get_message(topic[1])
    for t,b in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(topic[0],low,high)):
        yield t,deserialize_message(b,cls)
pairing={v['mask_stamp_ns']:v['lidar_stamp_ns'] for _,m in read(goal,'/spot1/measurement_timing')
         for v in [json.loads(m.data)] if v.get('sensor')=='mask_lidar_sync' and v['status']=='paired'}
masks=list(read(goal,'/spot1/object_mask'))
buffer=Buffer(cache_time=Duration(seconds=3600))
for _,m in read(raw,'/tf_static'):
    for t in m.transforms:buffer.set_transform_static(t,'closed_bag')
for _,m in read(raw,'/tf',int((origin+370)*1e9),int((origin+430)*1e9)):
    for t in m.transforms:buffer.set_transform(t,'closed_bag')
views=[];previous=None;report=[]
for received,m in masks:
    stamp=ns(m.header.stamp);lidar_stamp=pairing[stamp]
    candidates=[cloud for _,cloud in read(raw,'/spot1/ouster/front/points_filtered',lidar_stamp-100_000_000,lidar_stamp+1_000_000_000)
                if ns(cloud.header.stamp)==lidar_stamp]
    assert len(candidates)==1,(stamp,len(candidates))
    cloud=candidates[0]
    dims=tuple(d.size for d in m.object_mask.layout.dim)
    mask=largest_mask_component(np.asarray(m.object_mask.data[m.object_mask.layout.data_offset:],dtype=np.uint8).reshape(dims)[0,0].astype(bool))
    cam=m.cam_transforms.transforms[0].transform;q=cam.rotation
    rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    cam_origin=np.array([cam.translation.x,cam.translation.y,cam.translation.z])
    k=np.asarray(m.cam_infos[0].k).reshape(3,3)
    transform=buffer.lookup_transform('spot1/odom','spot1/ouster/front/os_lidar',Time.from_msg(cloud.header.stamp))
    t=transform.transform;q=t.rotation
    lidar_rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    points=np.asarray([list(p) for p in point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True)],float)
    world=points@lidar_rotation.T+[t.translation.x,t.translation.y,t.translation.z]
    camera=(world-cam_origin)@rotation
    valid=camera[camera[:,2]>.1];pixels=valid@k.T;pixels=pixels[:,:2]/pixels[:,2:]
    height,width=mask.shape
    inside=(pixels[:,0]>=0)&(pixels[:,0]<width)&(pixels[:,1]>=0)&(pixels[:,1]<height)
    pixels=pixels[inside].astype(int);valid=valid[inside]
    hits=mask[pixels[:,1],pixels[:,0]];valid=valid[hits];pixels=pixels[hits]
    quality=foreground_depth(camera,k,mask,20,8)
    v,u=np.median(np.argwhere(mask),axis=0);ray=rotation@np.linalg.solve(k,[u,v,1.]);ray/=np.linalg.norm(ray)
    current=np.array([m.odom.pose.pose.position.x,m.odom.pose.pose.position.y,m.odom.pose.pose.position.z])
    views=[v for v in views if 0<=stamp-v['stamp_ns']<=5e9]
    if not views:previous=None
    baseline=None if previous is None else float(np.linalg.norm(current-previous))
    added=previous is None or baseline>=1.
    if added:
        views.append(dict(origin=cam_origin,ray=ray,stamp_ns=stamp,angular_tolerance=.04));previous=current
    bearing_gate={}
    def trace(frame,event,arg):
        if frame.f_code is triangulate_bearings.__code__ and event=='return':
            local=frame.f_locals
            bearing_gate['return_line']=frame.f_lineno
            bearing_gate['pair_count']=len(local.get('pairs',[]))
            if 'matrix' in local:bearing_gate['condition_number']=float(np.linalg.cond(local['matrix']))
            if 'depths' in local:bearing_gate['depths']=local['depths'].tolist()
            if 'errors' in local:bearing_gate['angular_error_degrees']=np.degrees(local['errors']).tolist()
            if 'residual' in local:bearing_gate['median_residual_m']=float(np.median(local['residual']))
            if 'radius' in local:bearing_gate['radius_m']=local['radius']
        return trace
    bearing_result=None
    if added:
        sys.settrace(trace)
        try:bearing_result=triangulate_bearings(views,1.)
        finally:sys.settrace(None)
    parallax=max((math.acos(float(np.clip(a['ray']@b['ray'],-1,1))) for i,a in enumerate(views) for b in views[:i]),default=0.)
    report.append(dict(relative_sec=stamp/1e9-origin,lidar_points=len(points),mask_pixels=int(np.count_nonzero(mask)),
        points_projected_inside_mask=len(valid),unique_pixels=len(np.unique(pixels[:,1]*width+pixels[:,0])),
        projected_depth_range=([float(valid[:,2].min()),float(valid[:,2].max())] if len(valid) else None),
        lidar_quality=(None if quality is None else {key:quality[key] for key in ('association_validated','association_status','points','depth_sigma')}),
        independent_views=len(views),view_added=added,distance_from_previous_view=baseline,
        max_ray_angle_degrees=math.degrees(parallax),bearing_valid=bearing_result is not None,bearing_gate=bearing_gate,
        tf_camera_origin=cam_origin.tolist()))
out=root/'depth_closed_review.json'
out.write_text(json.dumps(dict(scope='Deterministic geometry from exact recorded paired LiDAR stamps and recorded TF. Not an end-to-end replay.',rows=report),indent=2))
print(json.dumps(dict(output=str(out),masks=len(report),lidar_valid=sum(x['lidar_quality'] is not None for x in report),
    hit_counts=collections.Counter(x['points_projected_inside_mask'] for x in report),
    added_views=sum(x['view_added'] for x in report),bearing_valid=sum(x['bearing_valid'] for x in report))))
