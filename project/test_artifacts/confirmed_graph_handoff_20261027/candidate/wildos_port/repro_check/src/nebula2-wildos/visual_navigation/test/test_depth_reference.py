"""Actual depth callbacks: references select current returns, never invent range."""
import ast
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth,triangulate_bearings,ground_like_surface

class DepthReferenceTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parents[1]/'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        tree=ast.parse(path.read_text(encoding='utf8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObjectMaskTriangulator')
        cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in
            ('on_query','check_lidar_in_mask','process_observation','prune_views','stamp_ns','bearing_observation')]
        env=dict(np=np,R=Rotation,foreground_depth=foreground_depth,triangulate_bearings=triangulate_bearings,
                 ground_like_surface=ground_like_surface,
                 point_cloud2=S(read_points=lambda msg,**_:msg.points),
                 rclpy=S(time=S(Time=S(from_msg=lambda t:t))),Duration=lambda **_:None)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),env)
        self.n=n=env['ObjectMaskTriangulator']()
        n.active_query='house';n.query_started_ns=n.last_observation_ns=0
        n.observation_timeout=5.;n.min_lidar_points=20;n.sparse_lidar_points=8;n.min_view_distance=1.;n.max_views=12
        n.views=[];n.bearing_views=[];n.prev_view_pos=None;n.depth_reference=None
        n.global_frame='spot1/odom';n.lidar_tf_frame='world'
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=100*10**9))
        n.get_logger=Mock();n.add_views=Mock()
        n.publish_navigation_goal_and_marker=Mock();n.publish_goal_hypotheses=Mock();n.publish_localization_rejection=Mock()
        n.tf_buffer=S(lookup_transform=lambda *_,**__:S(transform=S(rotation=S(x=0.,y=0.,z=0.,w=1.),translation=S(x=0.,y=0.,z=0.))))
        self.points=np.array([[x,y,10.] for x in np.linspace(-1.,1.,9) for y in np.linspace(-1.,1.,9)])
        self.cam=dict(object_mask=np.ones((100,100),bool),K=np.array([[100.,0.,50.],[0.,100.,50.],[0.,0.,1.]]),
                      R_wc=np.eye(3),t_wc=np.zeros((3,1)),width=100)
        n.extract_data_from_obj_mask_msg=lambda _:[self.cam]
        self.set_input(99*10**9,self.points)

    def set_input(self,t,points):
        self.msg=S(query=self.n.active_query,header=S(stamp=S(sec=t//10**9,nanosec=t%10**9)),
                   odom=S(pose=S(pose=S(position=S(x=0.,y=0.,z=0.)))))
        self.cloud=S(points=points,header=self.msg.header)
        self.n.msg_buffer=S(buffer=[1],pop_oldest_msg=lambda:(dict(obj_mask=self.msg,lidar=self.cloud),None,None),clear=Mock())

    def test_fresh_reference_chooses_current_supported_return(self):
        anchor=self.points[8].copy()
        self.n.depth_reference=dict(position=anchor,stamp_ns=98*10**9)
        self.n.process_observation()
        np.testing.assert_array_equal(self.n.triangulated_position,anchor)
        self.assertEqual(self.n.localization_quality['representative_kind'],'tracked_return')
        self.assertEqual(self.n.depth_reference['stamp_ns'],99*10**9)

    def test_expired_reference_does_not_bias_new_surface(self):
        self.n.depth_reference=dict(position=self.points[8],stamp_ns=93*10**9)
        self.n.process_observation()
        self.assertEqual(self.n.localization_quality['representative_kind'],'medoid_return')

    def test_empty_scan_does_not_replay_or_refresh_old_depth(self):
        self.n.process_observation();anchor=self.n.depth_reference.copy()
        self.set_input(99_500_000_000,np.empty((0,3)))
        self.n.process_observation()
        self.assertIsNone(self.n.triangulated_position)
        self.assertFalse(self.n.found_lidar_in_mask)
        self.assertEqual(self.n.depth_reference['stamp_ns'],anchor['stamp_ns'])

    def test_new_query_clears_reference(self):
        self.n.process_observation()
        self.n.on_query(S(data='gas station'))
        self.assertIsNone(self.n.depth_reference)

    def test_repeated_query_keeps_current_reference(self):
        self.n.process_observation();reference=self.n.depth_reference
        self.n.on_query(S(data=' HOUSE '))
        self.assertIs(self.n.depth_reference,reference)

    def test_duplicate_mask_does_not_refresh_depth_timestamp(self):
        self.n.process_observation();reference=self.n.depth_reference
        self.n.process_observation()
        self.assertIs(self.n.depth_reference,reference)

    def test_ambiguous_current_surfaces_revoke_reference(self):
        self.n.process_observation()
        points=np.array([[x*z/100,y*z/100,z] for z in (8.,20.)
                         for x in range(-10,0,2) for y in range(-10,0,2)])
        points[25:,:2]*=-1
        self.set_input(99_500_000_000,points)
        self.n.process_observation()
        self.assertIsNone(self.n.depth_reference)
        self.n.publish_localization_rejection.assert_called_once_with('ambiguous_depth_surfaces')

    def test_world_reference_uses_camera_acquisition_transform(self):
        rotation=Rotation.from_euler('z',90,degrees=True).as_matrix()
        origin=np.array([4.,-1.,0.]);anchor=rotation@self.points[8]+origin
        self.cam['R_wc']=rotation;self.cam['t_wc']=origin.reshape(3,1)
        self.n.depth_reference=dict(position=anchor,stamp_ns=98*10**9)
        self.set_input(99*10**9,self.points@rotation.T+origin)
        self.n.process_observation()
        np.testing.assert_allclose(self.n.triangulated_position,anchor,atol=1e-10)

if __name__=='__main__':unittest.main()
