"""Replay actual localization callbacks with recorded masks/poses and depth subsets."""
import ast,importlib.util,json,sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation

art=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
evidence=art/'target_identity_repair_20261025'
candidate=Path(sys.argv[1]) if len(sys.argv)>1 else art/'execution_handoff_repair_20261025/candidate'
vis=candidate/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation'
spec=importlib.util.spec_from_file_location('review_geometry',vis/'localization_geometry.py')
geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)
tree=ast.parse((vis/'obj_mask_triangulation.py').read_text(encoding='utf8'))
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObjectMaskTriangulator')
cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in
    ('process_observation','prune_views','stamp_ns','bearing_observation','check_lidar_in_mask')]
env=dict(np=np,triangulate_bearings=geo.triangulate_bearings,foreground_depth=geo.foreground_depth,
    R=Rotation,point_cloud2=S(read_points=lambda msg,**_:msg.points),
    rclpy=S(time=S(Time=S(from_msg=lambda t:t))),Duration=lambda **_:None,
    ground_like_surface=geo.ground_like_surface)
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(vis/'obj_mask_triangulation.py'),'exec'),env)
n=env['ObjectMaskTriangulator']()
n.active_query='green residential house';n.query_started_ns=n.last_observation_ns=0
n.views=[];n.bearing_views=[];n.prev_view_pos=None;n.observation_timeout=5.;n.min_view_distance=1.;n.max_views=12
n.get_logger=Mock();n.publish_navigation_goal_and_marker=Mock();n.publish_goal_hypotheses=Mock();n.publish_localization_rejection=Mock()
n.min_lidar_points=20;n.sparse_lidar_points=8;n.global_frame='spot1/odom';n.lidar_tf_frame='audit/world'
n.tf_buffer=S(lookup_transform=lambda *_args,**_kw:S(transform=S(rotation=S(x=0.,y=0.,z=0.,w=1.),translation=S(x=0.,y=0.,z=0.))))
audit=json.loads((evidence/'mask_identity_audit.json').read_text())
actual={d['observation_stamp_ns']:d for d in audit['localizations']}
fixture=np.load(evidence/'mask_identity_fixture.npz')
rows=[]
for i,t in enumerate(fixture['stamps']):
    t=int(t);position=fixture['body_positions'][i]
    msg=S(query=n.active_query,header=S(stamp=S(sec=t//10**9,nanosec=t%10**9)),
        odom=S(pose=S(pose=S(position=S(x=position[0],y=position[1],z=position[2])))))
    recorded=audit['rows'][i]['recorded_ns']
    n.get_clock=lambda:S(now=lambda:S(nanoseconds=recorded))
    cam=dict(object_mask=fixture['masks'][i],K=fixture['intrinsics'][i],R_wc=fixture['rotations'][i],
        t_wc=fixture['origins'][i].reshape(3,1),width=fixture['masks'][i].shape[1])
    f=evidence/('depth_fixture_'+str(t)+'.npz')
    cloud_points=np.empty((0,3))
    if f.exists():
        with np.load(f) as sample:cloud_points=sample['points_cam']@cam['R_wc'].T+cam['t_wc'].ravel()
    cloud=S(points=cloud_points,header=S(stamp=msg.header.stamp))
    n.msg_buffer=S(buffer=[1],pop_oldest_msg=lambda:(dict(obj_mask=msg,lidar=cloud),None,None))
    n.extract_data_from_obj_mask_msg=lambda _:[cam]
    def add(cameras):
        n.views.append(deepcopy(cam));n.bearing_views.append(n.bearing_observation(cameras))
        n.views=n.views[-12:];n.bearing_views=n.bearing_views[-12:]
    n.add_views=add
    n.process_observation()
    if n.triangulated_position is None:continue
    q=n.localization_quality;p=n.triangulated_position
    raycam=cam['R_wc'].T@(p-cam['t_wc'].ravel());uv=cam['K']@raycam;uv=uv[:2]/uv[2]
    inside=bool(raycam[2]>.1 and 0<=uv[0]<cam['object_mask'].shape[1]
        and 0<=uv[1]<cam['object_mask'].shape[0] and cam['object_mask'][int(uv[1]),int(uv[0])])
    row=dict(stamp_ns=t,source='lidar' if n.found_lidar_in_mask else 'multiview',position=p.tolist(),
        representative_kind=q.get('representative_kind'),
        inside_current_mask=inside,recorded_position_error_m=float(np.linalg.norm(p-np.array(actual[t]['position']))) if t in actual else None)
    if not n.found_lidar_in_mask:
        views=[v for v in n.bearing_views if np.linalg.norm(v['origin']-cam['t_wc'].ravel())>=n.min_view_distance]+[n.bearing_observation([cam])]
        # For an independent frame the actual solver used all stored views.
        if n.prev_view_pos is not None and np.linalg.norm(position-n.prev_view_pos)<1e-9:views=n.bearing_views
        origins=np.array([v['origin'] for v in views]);rays=np.array([v['ray'] for v in views]);rays/=np.linalg.norm(rays,axis=1)[:,None]
        matrices=np.eye(3)[None]-rays[:,:,None]*rays[:,None,:]
        depths=np.sum((p-origins)*rays,axis=1);limits=np.array([v.get('angular_tolerance',.035) for v in views])
        variances=np.maximum(.01,(depths*np.tan(limits))**2)
        covariance=np.linalg.inv(np.sum(matrices/variances[:,None,None],axis=0))
        row.update(reported_radius=q['radius'],angular_noise_radius=float(np.sqrt(np.linalg.eigvalsh(covariance[:2,:2]).max())),
                   views=[dict(origin=v['origin'].tolist(),ray=v['ray'].tolist(),angular_tolerance=v['angular_tolerance']) for v in views])
    rows.append(row)
if candidate.name=='candidate' and candidate.parent.name=='execution_handoff_repair_20261025':
    assert len(rows)==7 and max(r['recorded_position_error_m'] for r in rows)<1e-7,rows
output=Path(sys.argv[2]) if len(sys.argv)>2 else evidence/'localization_before.json'
output.write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
