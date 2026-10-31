"""Fresh observations must be evaluated without pretending to add a baseline."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import triangulate_bearings


class FreshBearingTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1]/'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ObjectMaskTriangulator')
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                    ('process_observation', 'prune_views', 'stamp_ns', 'bearing_observation')]
        env = dict(np=np, triangulate_bearings=triangulate_bearings)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), 'exec'), env)
        self.n = n = env['ObjectMaskTriangulator']()
        n.active_query = 'green residential house'
        n.query_started_ns = 90*10**9
        n.last_observation_ns = 99_000_000_000
        n.observation_timeout = 5.
        n.min_view_distance = 1.
        n.max_views = 12
        n.views = ['anchor', 'last']
        n.bearing_views = [self.bearing([-2., 0., 0.], 98_000_000_000),
                           self.bearing([0., 0., 0.], 99_000_000_000)]
        n.prev_view_pos = np.zeros(3)
        n.get_clock = lambda: S(now=lambda: S(nanoseconds=100*10**9))
        n.get_logger = Mock()
        n.check_lidar_in_mask = Mock()
        n.publish_localization_rejection = Mock()
        n.publish_navigation_goal_and_marker = Mock()
        n.publish_goal_hypotheses = Mock()
        n.add_views = Mock(side_effect=self.add_view)
        self.mask = np.zeros((101, 101), bool)
        self.mask[50, 48] = True
        self.camera = dict(object_mask=self.mask, K=np.array([[100.,0,50],[0,100.,50],[0,0,1]]),
                           R_wc=np.eye(3), t_wc=np.array([[.2],[0.],[0.]]))
        n.extract_data_from_obj_mask_msg = Mock(side_effect=lambda _: [self.camera])
        self.message = S(query=n.active_query,
                         header=S(stamp=S(sec=99, nanosec=300_000_000)),
                         odom=S(pose=S(pose=S(position=S(x=.2,y=0.,z=0.)))))
        n.msg_buffer = S(buffer=[1], pop_oldest_msg=lambda: (dict(obj_mask=self.message, lidar=object()),None,None))

    def bearing(self, origin, stamp):
        origin = np.array(origin)
        return dict(origin=origin, ray=np.array([0.,0.,10.])-origin,
                    stamp_ns=stamp, angular_tolerance=.04)

    def add_view(self, cameras):
        camera=cameras[0]
        if not camera['object_mask'].any():return
        v,u=np.median(np.argwhere(camera['object_mask']),axis=0)
        b=dict(origin=camera['t_wc'].ravel(),
               ray=camera['R_wc']@np.linalg.solve(camera['K'],[u,v,1.]),
               stamp_ns=self.n.stamp_ns(self.n.current_observation.header.stamp),angular_tolerance=.04)
        self.n.views.append('new')
        self.n.bearing_views.append(b)

    def test_close_fresh_frame_rechecks_against_independent_anchor(self):
        n=self.n; before=deepcopy(n.bearing_views)
        n.process_observation()
        self.assertIsNotNone(n.triangulated_position)
        np.testing.assert_allclose(n.triangulated_position, [0.,0.,10.], atol=1e-8)
        n.add_views.assert_not_called()
        self.assertEqual(len(n.bearing_views),len(before))
        for a,b in zip(n.bearing_views,before):
            np.testing.assert_array_equal(a['origin'],b['origin'])
            self.assertEqual(a['stamp_ns'],b['stamp_ns'])
        np.testing.assert_array_equal(n.prev_view_pos,np.zeros(3))

    def test_four_fresh_frames_can_produce_four_current_positions(self):
        stamps=[]
        for i in range(4):
            self.message.header.stamp.nanosec=300_000_000+i*100_000_000
            self.n.process_observation()
            self.assertIsNotNone(self.n.triangulated_position)
            stamps.append(self.n.stamp_ns(self.n.current_observation.header.stamp))
        self.assertEqual(len(set(stamps)),4)
        self.assertEqual(len(self.n.bearing_views),2)

    def test_close_frames_cannot_create_baseline(self):
        self.n.views=['last']; self.n.bearing_views=self.n.bearing_views[-1:]
        self.n.process_observation()
        self.assertIsNone(self.n.triangulated_position)

    def test_new_inconsistent_ray_cannot_republish_old_solution(self):
        self.n.bearing_views.insert(1,self.bearing([-1.,0.,0.],98_500_000_000))
        self.n.views.insert(1,'middle')
        self.mask[:]=False; self.mask[50,95]=True
        self.n.process_observation()
        self.assertIsNone(self.n.triangulated_position)

    def test_expired_anchor_cannot_be_reused(self):
        self.n.bearing_views[0]['stamp_ns']=90*10**9
        self.n.process_observation()
        self.assertIsNone(self.n.triangulated_position)

    def test_empty_mask_cannot_refresh_history(self):
        self.mask[:]=False
        self.n.process_observation()
        self.assertIsNone(self.n.triangulated_position)

    def test_duplicate_observation_is_not_counted_twice(self):
        self.n.last_observation_ns=99_300_000_000
        self.n.process_observation()
        self.n.check_lidar_in_mask.assert_not_called()
        self.n.publish_navigation_goal_and_marker.assert_not_called()

    def test_old_query_cannot_refresh_position(self):
        self.message.query='gas station'
        self.n.process_observation()
        self.n.check_lidar_in_mask.assert_not_called()

    def test_independent_view_still_updates_keyframe(self):
        self.message.odom.pose.pose.position.x=2.
        self.camera['t_wc']=np.array([[2.],[0.],[0.]])
        self.mask[:]=False;self.mask[50,30]=True
        self.n.process_observation()
        self.n.add_views.assert_called_once()
        np.testing.assert_array_equal(self.n.prev_view_pos,[2.,0.,0.])
        np.testing.assert_allclose(self.n.triangulated_position,[0.,0.,10.],atol=1e-8)

    def test_ambiguous_depth_still_clears_history_and_blocks_fallback(self):
        def ambiguity(*_):self.n.localization_quality={'association_validated':False}
        self.n.check_lidar_in_mask.side_effect=ambiguity
        self.n.process_observation()
        self.assertEqual(self.n.bearing_views,[])
        self.n.publish_localization_rejection.assert_called_once_with('ambiguous_depth_surfaces')
        self.n.publish_navigation_goal_and_marker.assert_not_called()


if __name__ == '__main__':unittest.main()
