"""Compare old/new angular uncertainty on one closed multiview-only round."""
import ast,collections,importlib.util,json,sqlite3,sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from visual_navigation.utils.semantic_mask import largest_mask_component
base=Path('/home/ly/execution_continuity_review_20261022')
root=base/'v32_retry_admission';receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def messages(name):
    idx,cls=topics[name]
    for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(idx,)):
        yield t,deserialize_message(b,cls)
actual={d['observation_stamp_ns']:d for _,m in messages('/spot1/object_localization') for d in [json.loads(m.data)] if d.get('localization_valid') is True}
assert actual and all(d['source']=='multiview' for d in actual.values())
paired={d['mask_stamp_ns']:d for _,m in messages('/spot1/measurement_timing') for d in [json.loads(m.data)] if d.get('sensor')=='mask_lidar_sync' and d.get('status')=='paired'}
inputs=[]
for recorded,m in messages('/spot1/object_mask'):
    t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    if t not in paired:continue
    shape=[d.size for d in m.object_mask.layout.dim]
    raw=np.asarray(m.object_mask.data,dtype=np.uint8).reshape(shape)[:,0]
    masks=[largest_mask_component(mask.astype(bool)) for mask in raw]
    cam_id=max(range(len(masks)),key=lambda i:np.count_nonzero(masks[i]));tf=m.cam_transforms.transforms[cam_id].transform
    q=tf.rotation;o=tf.translation;p=m.odom.pose.pose.position
    cam=dict(object_mask=masks[cam_id],K=np.asarray(m.cam_infos[cam_id].k).reshape(3,3),R_wc=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),t_wc=np.asarray([o.x,o.y,o.z]).reshape(3,1),width=m.cam_infos[cam_id].width)
    inputs.append((paired[t]['received_ros_ns'],m,cam))
db.close();reports={}
tri=root/'candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
for label,geometry in [('v32',tri.with_name('localization_geometry.py')),('corrected',base/'v29_range_consistency_r2/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py')]:
    spec=importlib.util.spec_from_file_location('geometry_'+label,geometry);geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)
    tree=ast.parse(tri.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObjectMaskTriangulator')
    cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('process_observation','prune_views','stamp_ns','bearing_observation')]
    env=dict(np=np,R=Rotation,triangulate_bearings=geo.triangulate_bearings)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(tri),'exec'),env)
    n=env['ObjectMaskTriangulator']();n.active_query='green residential house';n.query_started_ns=n.last_observation_ns=0
    n.views=[];n.bearing_views=[];n.prev_view_pos=None;n.observation_timeout=5.;n.min_view_distance=1.;n.max_views=12
    n.get_logger=Mock();n.publish_navigation_goal_and_marker=Mock();n.publish_goal_hypotheses=Mock();n.publish_localization_rejection=Mock()
    n.check_lidar_in_mask=lambda *_:None
    rays_used=[]
    def solve(views,*args,**kw):
        rays_used[:]=deepcopy(views)
        return geo.triangulate_bearings(views,*args,**kw)
    env['triangulate_bearings']=solve
    rows=[]
    for received,m,cam in inputs:
        n.get_clock=lambda received=received:S(now=lambda:S(nanoseconds=received))
        n.extract_data_from_obj_mask_msg=lambda _,cam=cam:[cam]
        def add(cameras):
            n.views.append(deepcopy(cam));n.bearing_views.append(n.bearing_observation(cameras))
            n.views=n.views[-12:];n.bearing_views=n.bearing_views[-12:]
        n.add_views=add
        n.msg_buffer=S(buffer=[1],pop_oldest_msg=lambda m=m:(dict(obj_mask=m,lidar=None),None,None))
        n.process_observation();t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if n.triangulated_position is not None:
            p=n.triangulated_position
            rows.append(dict(stamp_ns=t,position=p.tolist(),radius=n.localization_quality['radius'],
                actual_error_m=float(np.linalg.norm(p-np.asarray(actual[t]['position']))) if t in actual else None,
                views=[dict(origin=v['origin'].tolist(),ray=v['ray'].tolist(),angular_tolerance=v.get('angular_tolerance')) for v in rays_used]))
    reports[label]=rows
report=dict(scope='Actual process/bearing/prune callbacks with recorded paired masks. LiDAR branch disabled: all recorded valid outputs are multiview; old outputs must match before attributing differences.',input_frames=len(inputs),recorded_valid=len(actual),replay=reports)
out=root/'bearing_geometry_comparison.json';out.write_text(json.dumps(report,indent=2))
print(json.dumps(dict(input_frames=len(inputs),recorded=len(actual),old=len(reports['v32']),corrected=len(reports['corrected']),old_max_error=max((r['actual_error_m'] for r in reports['v32'] if r['actual_error_m'] is not None),default=None)),indent=2))
