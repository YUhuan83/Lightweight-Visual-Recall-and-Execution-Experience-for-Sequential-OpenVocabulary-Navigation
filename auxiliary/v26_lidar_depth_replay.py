"""Exact projection and production foreground helper on recorded paired clouds."""
import collections,hashlib,inspect,json,math,sqlite3,sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from sensor_msgs_py import point_cloud2
from visual_navigation.utils.semantic_mask import largest_mask_component
from visual_navigation.explorfm_triangulation import localization_geometry as geometry
root=Path('/home/ly/execution_continuity_review_20261022/v26_body_arrival')
config=yaml.safe_load((root/'candidate/official_adapter/config/beamng_triangulation.yaml').read_text())
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def convert(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:convert(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [convert(v) for v in value]
    return value
reports=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    def messages(topic):
        tid,typ=topics[topic];cls=get_message(typ)
        for stamp,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
            yield stamp,deserialize_message(blob,cls)
    timings=[json.loads(m.data) for _,m in messages('/spot1/measurement_timing')]
    paired={d['mask_stamp_ns']:d for d in timings if d.get('sensor')=='mask_lidar_sync' and d['status']=='paired'}
    wanted={d['lidar_stamp_ns'] for d in paired.values()}
    clouds={ns(m):m for _,m in messages('/spot1/ouster/front/points_filtered') if ns(m) in wanted}
    masks=list(messages('/spot1/object_mask'))
    actual={d['observation_stamp_ns']:d for _,m in messages('/spot1/object_localization') for d in [json.loads(m.data)]}
    buffer=Buffer(cache_time=Duration(seconds=300.))
    for _,m in messages('/tf_static'):
        for tf in m.transforms:buffer.set_transform_static(tf,'recorded')
    from geometry_msgs.msg import TransformStamped
    recorded={ns(tf):tf for _,m in messages('/tf') for tf in m.transforms if tf.child_frame_id=='spot1/base_link'}
    exact_matches=[]
    for _,odom in messages('/spot1/odom'):
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=odom.child_frame_id
        tf.transform.translation.x=odom.pose.pose.position.x
        tf.transform.translation.y=odom.pose.pose.position.y
        tf.transform.translation.z=odom.pose.pose.position.z
        tf.transform.rotation=odom.pose.pose.orientation
        if ns(odom) in recorded:
            a=tf.transform;b=recorded[ns(odom)].transform
            exact_matches.append(max(abs(getattr(getattr(a,p),k)-getattr(getattr(b,p),k)) for p,ks in [('translation','xyz'),('rotation','xyzw')] for k in ks))
        buffer.set_transform(tf,'verified_odom')
    assert exact_matches and max(exact_matches)==0.,exact_matches[:5]
    db.close();rows=[]
    for _,m in masks:
        stamp=ns(m)
        if stamp not in paired:continue
        row=dict(mask_stamp_ns=stamp,lidar_stamp_ns=paired[stamp]['lidar_stamp_ns'])
        cloud=clouds.get(row['lidar_stamp_ns'])
        if cloud is None:row['result']='missing_recorded_cloud';rows.append(row);continue
        try:tf=buffer.lookup_transform('spot1/odom',cloud.header.frame_id,Time.from_msg(cloud.header.stamp)).transform
        except Exception as e:row.update(result='missing_transform',detail=str(e));rows.append(row);continue
        q=tf.rotation;p=tf.translation
        lidar_rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        raw=np.asarray([list(pt) for pt in point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True)],dtype=float).reshape(-1,3)
        world=raw@lidar_rotation.T+[p.x,p.y,p.z]
        shape=[d.size for d in m.object_mask.layout.dim]
        masks_array=np.asarray(m.object_mask.data[m.object_mask.layout.data_offset:],dtype=np.uint8).reshape(shape)[:,0,:,:]
        masks_array=[largest_mask_component(a.astype(bool)) for a in masks_array]
        camera=max(range(len(masks_array)),key=lambda i:np.count_nonzero(masks_array[i]))
        tf=m.cam_transforms.transforms[camera].transform;q=tf.rotation;p=tf.translation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();origin=np.asarray([p.x,p.y,p.z])
        points=(world-origin)@rotation;k=np.asarray(m.cam_infos[camera].k).reshape(3,3)
        trace=[]
        def tracer(frame,event,arg):
            if event=='return' and frame.f_code in (geometry.foreground_depth.__code__,geometry._surface_quality.__code__):
                local=frame.f_locals
                entry=dict(function=frame.f_code.co_name,line=frame.f_lineno,returned_none=arg is None)
                for key in ('points','chosen','projected','eligible','candidates'):
                    if key in local:entry[key+'_count']=len(local[key])
                for key in ('confidence','depth_sigma','support'):
                    if key in local:entry[key]=float(local[key])
                if 'projected' in local and len(local['projected']):entry['pixel_span']=np.ptp(local['projected'],axis=0).tolist()
                trace.append(entry)
            return tracer
        sys.settrace(tracer)
        try:quality=geometry.foreground_depth(points,k,masks_array[camera],config['min_lidar_points'],config['sparse_lidar_points'])
        finally:sys.settrace(None)
        row.update(raw_cloud_points=len(raw),trace=trace)
        if quality is None:row['result']='no_supported_depth'
        elif not quality['association_validated']:row['result']='ambiguous_depth'
        else:
            position=rotation@quality['position']+origin
            row.update(result='lidar_localized',replayed_position=position.tolist(),quality=convert(quality))
            expected=actual.get(stamp)
            row['recorded_valid_lidar']=bool(expected and expected.get('source')=='lidar' and expected.get('localization_valid'))
            row['position_error_m']=float(np.linalg.norm(position-np.asarray(expected['position']))) if row['recorded_valid_lidar'] else None
        rows.append(row)
    report=dict(trial=str(trial),paired=len(paired),clouds_found=len(clouds),results=dict(collections.Counter(r['result'] for r in rows)),
        odom_tf_exact_matches=len(exact_matches),sync_counts=dict(collections.Counter(d['status']+':'+d.get('reason','') for d in timings if d.get('sensor')=='mask_lidar_sync')),foreground_return_lines=dict(collections.Counter(str(t['line']) for r in rows for t in r.get('trace',[]) if t['function']=='foreground_depth' and t['returned_none'])),
        surface_return_lines=dict(collections.Counter(str(t['line']) for r in rows for t in r.get('trace',[]) if t['function']=='_surface_quality' and t['returned_none'])),rows=rows)
    reports.append(report)
source=Path(geometry.__file__)
output=dict(geometry_source=str(source),geometry_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    source_lines=source.read_text().splitlines(),rounds=reports,
    caveat='Recorded acquisition-stamped TF interpolation and exact paired clouds. Must match positive outputs before attributing negative gates; does not prove masks belong to the requested physical object.')
encoded=json.dumps(output,sort_keys=True).encode();(root/'lidar_depth_audit.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),geometry_source=str(source),rounds=[{k:v for k,v in r.items() if k!='rows'} for r in reports])))
