"""Exact geometry/process replay of CLOSED v37 pairs; no simulated detections."""
import ast,collections,hashlib,json,sqlite3,sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import rclpy,yaml
from scipy.spatial.transform import Rotation as R
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from visual_navigation.utils.semantic_mask import largest_mask_component
from visual_navigation.explorfm_triangulation import localization_geometry as geometry
root=Path('/home/ly/execution_continuity_review_20261022/v46_approach_feedback')
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
config=yaml.safe_load((root/'candidate/official_adapter/config/beamng_triangulation.yaml').read_text())
source=root/'candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
tree=ast.parse(source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObjectMaskTriangulator')
cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('process_observation','prune_views','stamp_ns','bearing_observation','check_lidar_in_mask')]
env=dict(np=np,R=R,rclpy=rclpy,Duration=Duration,point_cloud2=point_cloud2,
    foreground_depth=geometry.foreground_depth,triangulate_bearings=geometry.triangulate_bearings,ground_like_surface=geometry.ground_like_surface)
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(source),'exec'),env)
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
reports=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
    def messages(name):
        i,c=topics[name]
        for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):
            yield t,deserialize_message(b,c)
    timings=[json.loads(m.data) for _,m in messages('/spot1/measurement_timing')]
    paired={d['mask_stamp_ns']:d for d in timings if d.get('sensor')=='mask_lidar_sync' and d.get('status')=='paired'}
    wanted={d['lidar_stamp_ns'] for d in paired.values()}
    clouds={ns(m):m for _,m in messages('/spot1/ouster/front/points_filtered') if ns(m) in wanted}
    actual={d['observation_stamp_ns']:d for _,m in messages('/spot1/object_localization') for d in [json.loads(m.data)] if d.get('localization_valid')}
    n=env['ObjectMaskTriangulator']();n.active_query='green residential house';n.query_started_ns=n.last_observation_ns=0
    n.views=[];n.bearing_views=[];n.prev_view_pos=None;n.depth_reference=None
    n.observation_timeout=float(config['observation_timeout_sec']);n.min_view_distance=float(config['min_view_distance']);n.max_views=int(config['max_views'])
    n.min_lidar_points=int(config['min_lidar_points']);n.sparse_lidar_points=int(config['sparse_lidar_points'])
    n.global_frame='spot1/odom';n.get_logger=Mock();n.publish_navigation_goal_and_marker=Mock();n.publish_goal_hypotheses=Mock();n.publish_localization_rejection=Mock()
    n.tf_buffer=Buffer(cache_time=Duration(seconds=400.))
    for _,m in messages('/tf_static'):
        for tf in m.transforms:n.tf_buffer.set_transform_static(tf,'recorded')
    recorded={ns(tf):tf for _,m in messages('/tf') for tf in m.transforms if tf.child_frame_id=='spot1/base_link'}
    errors=[]
    for _,odom in messages('/spot1/odom'):
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=odom.child_frame_id
        tf.transform.translation.x=odom.pose.pose.position.x;tf.transform.translation.y=odom.pose.pose.position.y;tf.transform.translation.z=odom.pose.pose.position.z
        tf.transform.rotation=odom.pose.pose.orientation
        if ns(odom) in recorded:
            a=tf.transform;b=recorded[ns(odom)].transform
            errors.append(max(abs(getattr(getattr(a,p),k)-getattr(getattr(b,p),k)) for p,ks in [('translation','xyz'),('rotation','xyzw')] for k in ks))
        n.tf_buffer.set_transform(tf,'verified_odom')
    assert errors and max(errors)==0.,errors[:5]
    masks=list(messages('/spot1/object_mask'));db.close();rows=[]
    for recorded_at,m in masks:
        stamp=ns(m);row=dict(stamp_ns=stamp,paired=stamp in paired)
        if stamp not in paired:row['result']='not_paired';rows.append(row);continue
        d=paired[stamp];cloud=clouds.get(d['lidar_stamp_ns'])
        if cloud is None:row['result']='missing_cloud';rows.append(row);continue
        shape=[int(d.size) for d in m.object_mask.layout.dim]
        raw=np.asarray(m.object_mask.data[m.object_mask.layout.data_offset:],dtype=np.uint8).reshape(shape)[:,0]
        cams=[]
        for i,mask in enumerate(raw):
            tf=m.cam_transforms.transforms[i];q=tf.transform.rotation;p=tf.transform.translation;info=m.cam_infos[i]
            assert tf.header.frame_id=='spot1/odom' and tf.child_frame_id.lstrip('/')==info.header.frame_id.lstrip('/')
            assert abs(ns(tf)-stamp)<=50_000_000 and abs(ns(info)-stamp)<=50_000_000
            cams.append(dict(object_mask=largest_mask_component(mask.astype(bool)),K=np.asarray(info.k).reshape(3,3),
                R_wc=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),t_wc=np.asarray([p.x,p.y,p.z]).reshape(3,1),width=info.width,height=info.height))
        n.lidar_tf_frame=cloud.header.frame_id;n.get_clock=lambda d=d:S(now=lambda:S(nanoseconds=d['received_ros_ns']))
        n.extract_data_from_obj_mask_msg=lambda _,cams=cams:cams
        def add(cameras):
            n.views.append({});n.bearing_views.append(n.bearing_observation(cameras))
            n.views=n.views[-n.max_views:];n.bearing_views=n.bearing_views[-n.max_views:]
        n.add_views=add
        n.msg_buffer=S(buffer=[1],pop_oldest_msg=lambda m=m,cloud=cloud:(dict(obj_mask=m,lidar=cloud),None,None))
        trace=[]
        def tracer(frame,event,arg):
            if event=='return' and frame.f_code in (geometry.foreground_depth.__code__,geometry._surface_quality.__code__,geometry.triangulate_bearings.__code__):
                l=frame.f_locals;entry=dict(function=frame.f_code.co_name,line=frame.f_lineno,rejected=arg is None)
                for k in ('points','chosen','projected','views','pairs'):
                    if k in l:entry[k]=len(l[k])
                for k in ('radius','depth_sigma','confidence'):
                    if k in l:entry[k]=float(l[k])
                if 'projected' in l and len(l['projected']):entry['pixel_span']=np.ptp(l['projected'],axis=0).tolist()
                if frame.f_code==geometry.foreground_depth.__code__:
                    entry['clusters']=[len(c) for c in l.get('clusters',[])]
                    entry['eligible']=[len(c) for c in l.get('eligible',[])]
                if frame.f_code==geometry.triangulate_bearings.__code__:
                    for name in ('depths','errors','limits','residual'):
                        if name in l:
                            a=np.asarray(l[name]);entry[name]=a.tolist()
                    if 'origins' in l:
                        o=l['origins'];entry['max_baseline_m']=max((float(np.linalg.norm(a-b)) for a in o for b in o),default=0.)
                    if 'position' in l:entry['position']=np.asarray(l['position']).tolist()
                trace.append(entry)
            return tracer
        sys.settrace(tracer)
        try:n.process_observation()
        except Exception as e:row.update(result='replay_error',error=repr(e))
        finally:sys.settrace(None)
        row.update(source_age_s=(d['received_ros_ns']-stamp)*1e-9,trace=trace,bearing_keyframes=len(n.bearing_views))
        if 'result' not in row:
            row['result']='lidar' if n.found_lidar_in_mask else 'multiview' if n.triangulated_position is not None else 'unlocalized'
            if n.triangulated_position is not None:
                row['position']=n.triangulated_position.tolist();expected=actual.get(stamp)
                row['recorded_match']=expected is not None and expected['source']==row['result']
                row['position_error_m']=float(np.linalg.norm(n.triangulated_position-np.asarray(expected['position']))) if expected else None
        rows.append(row)
    reports.append(dict(trial=trial.name,mask_count=len(masks),paired=len(paired),recorded_valid=len(actual),
        results=dict(collections.Counter(r['result'] for r in rows)),
        rejection_gates=dict(collections.Counter(t['function']+':'+str(t['line']) for r in rows for t in r.get('trace',[]) if t['rejected'])),rows=rows))
report=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),geometry_sha256=hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
    scope='Production process/depth/bearing/prune methods; recorded exact pairs and odom-verified TF. No semantic ground truth claim.',rounds=reports)
encoded=json.dumps(report,sort_keys=True).encode();(root/'localization_failure_replay.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=[{k:v for k,v in r.items() if k!='rows'} for r in reports]),indent=2))
