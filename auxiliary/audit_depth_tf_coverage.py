import bisect, collections, hashlib, json, sqlite3, types
from pathlib import Path
import numpy as np
import yaml
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from geometry_msgs.msg import TransformStamped
from visual_navigation.explorfm_triangulation.obj_mask_triangulation import ObjectMaskTriangulator as Node

ROOT=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
OUT=ROOT.parent/'target_contract_20261025'
cfg=yaml.safe_load((ROOT/'candidate/official_adapter/config/beamng_triangulation.yaml').read_text())
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
reports=[]
for trial in sorted((ROOT/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'));db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    def msgs(name):
        tid,typ=topics[name];cls=get_message(typ)
        for bt,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):yield bt,deserialize_message(data,cls)
    paired={d['mask_stamp_ns']:d for _,m in msgs('/spot1/measurement_timing') for d in [json.loads(m.data)] if d.get('sensor')=='mask_lidar_sync' and d.get('status')=='paired'}
    wanted={d['lidar_stamp_ns'] for d in paired.values()}
    clouds={ns(m):m for _,m in msgs('/spot1/ouster/front/points_filtered') if ns(m) in wanted}
    masks=list(msgs('/spot1/object_mask'));actual={d['observation_stamp_ns']:d for _,m in msgs('/spot1/object_localization') for d in [json.loads(m.data)]}
    tf_by_child=collections.defaultdict(list)
    for bt,m in msgs('/tf'):
        for tf in m.transforms:tf_by_child[tf.child_frame_id].append((bt,tf))
    statics=[tf for _,m in msgs('/tf_static') for tf in m.transforms]
    odoms=list(msgs('/spot1/odom')); db.close()
    real=sorted(tf_by_child['spot1/base_link'],key=lambda x:ns(x[1]));stamps=[ns(tf) for _,tf in real]
    real_by_stamp={ns(tf):tf for _,tf in real}
    buffer=Buffer(cache_time=Duration(seconds=300.));matches=[]
    for tf in statics:buffer.set_transform_static(tf,'recorded')
    for _,m in sorted(odoms,key=lambda x:ns(x[1])):
        tf=TransformStamped();tf.header=m.header;tf.child_frame_id=m.child_frame_id
        tf.transform.translation.x=m.pose.pose.position.x;tf.transform.translation.y=m.pose.pose.position.y;tf.transform.translation.z=m.pose.pose.position.z;tf.transform.rotation=m.pose.pose.orientation
        buffer.set_transform(tf,'odom-derived')
        if ns(m) in real_by_stamp:
            ref=real_by_stamp[ns(m)];a=tf.transform;b=ref.transform
            matches.append(max(abs(getattr(getattr(a,p),k)-getattr(getattr(b,p),k)) for p,ks in [('translation','xyz'),('rotation','xyzw')] for k in ks))
    node=types.SimpleNamespace(num_cameras=cfg['num_cameras'],global_frame=cfg['parent_frame'],cam_tf_frame=cfg['cam_frame'],lidar_tf_frame=cfg['lidar_frame'],camera_mask_max_skew_ns=int(cfg['camera_mask_max_skew_sec']*1e9),min_lidar_points=cfg['min_lidar_points'],sparse_lidar_points=cfg['sparse_lidar_points'],tf_buffer=buffer)
    node.stamp_ns=Node.stamp_ns;node.get_objmask_from_multiarray=types.MethodType(Node.get_objmask_from_multiarray,node)
    rows=[]
    for _,m in masks:
        stamp=ns(m)
        if stamp not in paired:continue
        lidar=paired[stamp]['lidar_stamp_ns'];idx=bisect.bisect_left(stamps,lidar)
        row=dict(stamp=stamp,lidar_stamp=lidar,actual=actual.get(stamp),tf_gap_s=None)
        if 0<idx<len(stamps):row['tf_gap_s']=(stamps[idx]-stamps[idx-1])/1e9;row['tf_bracket_ns']=[stamps[idx-1],stamps[idx]]
        cloud=clouds.get(lidar)
        if cloud is None:row['error']='missing_cloud';rows.append(row);continue
        node.active_query=' '.join(m.query.casefold().split());node.localization_quality=None;node.triangulated_position=None;node.found_lidar_in_mask=False
        try:
            Node.check_lidar_in_mask(node,Node.extract_data_from_obj_mask_msg(node,m),cloud)
            row['found']=node.found_lidar_in_mask
            row['position']=node.triangulated_position.tolist() if node.found_lidar_in_mask else None
            if row.get('actual') and row['found']:row['error_m']=float(np.linalg.norm(node.triangulated_position-row['actual']['position']))
        except Exception as e:row['error']=str(e)
        rows.append(row)
    report=dict(trial=trial.name,tf_children={k:len(v) for k,v in tf_by_child.items()},odoms=len(odoms),odom_tf_exact_stamps=len(matches),max_odom_tf_error=max(matches,default=None),tf_gaps_over_point_two=[dict(begin=a,end=b,gap_s=(b-a)/1e9) for a,b in zip(stamps,stamps[1:]) if b-a>200_000_000],results=dict(found=sum(r.get('found',False) for r in rows),matches=sum(r.get('error_m',100)<1e-6 for r in rows),errors=sum('error' in r for r in rows)),rows=rows)
    reports.append(report)
encoded=json.dumps(reports,sort_keys=True).encode();(OUT/'depth_tf_coverage.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=[{k:v for k,v in r.items() if k not in ('rows','tf_gaps_over_point_two')} for r in reports])))
