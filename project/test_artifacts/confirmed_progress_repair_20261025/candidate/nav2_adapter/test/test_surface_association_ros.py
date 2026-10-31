"""Recorded surface ambiguity and actual downstream state callbacks, without DDS."""
import ast
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as S
import time
import unittest
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth

ROOT = Path(__file__).resolve().parents[2]
VIS = ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation'


class SurfaceAssociationTests(unittest.TestCase):
    def setUp(self):
        self.now = 100*10**9
        self.clock = S(now=lambda: S(nanoseconds=self.now,
            to_msg=lambda: Time(sec=self.now//10**9, nanosec=self.now%10**9)))
        self.goal, self.bridge = GoalStandoff(), GraphPathBridge()
        for node in (self.goal, self.bridge):
            node.get_clock = lambda: self.clock
        self.goal.publisher = Mock()
        self.goal.confirmation_pub = Mock()
        self.bridge.status_pub = Mock()
        self.bridge.query = 'house'
        self.bridge.query_started_ns = self.now
        self.goal.on_query(String(data='house'))
        self.bridge.query_reset_pending = False
        self.bridge.odom = Odometry()
        self.bridge.odom.pose.pose.orientation.w = 1.
        self.bridge.odom.header.frame_id = 'spot1/odom'
        self.bridge.odom_received_at = time.monotonic()
        self.advance()

    def tearDown(self):
        self.goal.destroy_node()
        self.bridge.destroy_node()

    def advance(self):
        self.now += 100_000_000
        odom = deepcopy(self.bridge.odom)
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(self.now, 10**9)
        self.goal.on_odom(odom)

    def localize(self, x=20., **changes):
        self.advance()
        data = dict(query='house', frame='spot1/odom', observation_stamp_ns=self.now,
            localization_valid=True, source='lidar', position=[x, 0., 2.],
            observer_position=[0., 0., 0.], observer_orientation=[0., 0., 0., 1.],
            quality={'confidence': .9, 'association_validated': True})
        data.update(changes)
        self.goal.on_localization(String(data=json.dumps(data)))
        return data

    def rejected(self, stamp=None, **changes):
        self.advance()
        data = dict(query='house', frame='spot1/odom',
            observation_stamp_ns=self.now if stamp is None else stamp,
            evidence_rejected=True, localization_valid=False,
            instance_id='', reason='ambiguous_depth_surfaces')
        data.update(changes)
        return String(data=json.dumps(data))

    def arrival(self, **changes):
        data = dict(query='house', frame='spot1/odom', observation_stamp_ns=self.now,
            localization_valid=True, search_confirmed=True,
            object_position=[2., 0., 2.], observer_position=[0., 0., 0.],
            radius=.3, inliers=4, observations=4, standoff=[0., 0.],
            quality={'confidence': .9, 'association_validated': True})
        data.update(changes)
        return String(data=json.dumps(data))

    def fixture(self):
        with np.load(VIS/'test/fixtures/trial14_ambiguous_surfaces.npz') as sample:
            return {key: sample[key].copy() for key in sample.files}

    def test_recorded_trial14_preserves_two_surfaces_without_validating_nearest(self):
        sample = self.fixture()
        result = foreground_depth(sample['points_cam'], sample['k'], sample['mask'])
        self.assertFalse(result['association_validated'])
        self.assertEqual(len(result['depth_candidates']), 2)
        np.testing.assert_allclose([c['position'][2] for c in result['depth_candidates']],
                                   [2.3360903896, 19.697658], atol=1e-5)
        # Higher geometric confidence must never override semantic ambiguity.
        self.assertGreater(result['confidence'], .8)

    def test_single_supported_surface_remains_usable(self):
        sample = self.fixture()
        points = sample['points_cam'][sample['points_cam'][:, 2] > 10.]
        result = foreground_depth(points, sample['k'], sample['mask'])
        self.assertTrue(result['association_validated'])
        self.assertEqual(len(result['depth_candidates']), 1)

    def test_ambiguity_clears_pending_evidence_but_preserves_instance_for_ack(self):
        for _ in range(5):
            self.localize()
        anchor = deepcopy(self.goal.confirmed_instance_anchor)
        self.assertIsNotNone(anchor)
        self.localize(localization_valid=False, reason='ambiguous_depth_surfaces')
        self.assertFalse(self.goal.observations)
        self.assertIsNone(self.goal.pending_confirmation)
        self.assertIsNone(self.goal.active_approach)
        self.assertEqual(self.goal.confirmed_instance_anchor, anchor)
        rejected = json.loads(self.goal.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(rejected['evidence_rejected'])
        self.assertEqual(rejected['instance_id'], anchor['instance_id'])

    def test_new_far_surface_invalidates_wrong_anchor_without_widening_gate(self):
        for _ in range(5):
            self.localize()
        self.localize(40.)
        self.assertEqual(self.goal.confirmed_instance_anchor['object_position'][0], 20.)
        result = json.loads(self.goal.confirmation_pub.publish.call_args.args[0].data)
        self.assertEqual(result['reason'], 'instance_mismatch')
        self.assertFalse(self.goal.observations)

    def test_old_larger_cluster_cannot_refresh_on_new_disjoint_observation(self):
        data = self.localize()
        self.goal.observations = [dict(data, observation_stamp_ns=self.now-i)
                                  for i in (3, 2, 1, 0)]
        self.localize(40.)
        self.assertIsNone(self.goal.pending_confirmation)
        result = json.loads(self.goal.confirmation_pub.publish.call_args.args[0].data)
        self.assertEqual(result['reason'], 'current_observation_outside_cluster')

    def test_after_ambiguity_fresh_supported_frames_can_confirm_again(self):
        self.localize(localization_valid=False, reason='ambiguous_depth_surfaces')
        self.goal.confirmation_pub.reset_mock()
        for _ in range(4):
            self.localize()
        result = json.loads(self.goal.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(result['localization_valid'])
        self.assertTrue(result['search_confirmed'])

    def test_delayed_positive_and_refresh_cannot_cross_rejection_fence(self):
        old = self.arrival()
        self.bridge.on_object_confirmation(self.rejected())
        self.bridge.on_object_confirmation(old)
        self.bridge.on_object_confirmation(String(data=json.dumps(dict(json.loads(old.data),
            approach_refresh=True, base_observation_stamp_ns=self.now-100_000_000,
            approach_update_stamp_ns=self.now))))
        self.assertFalse(self.bridge.object_found)

    def test_rejection_does_not_cancel_path_or_reset_observation_timer(self):
        self.bridge.reset = Mock()
        self.bridge.following = True
        self.bridge.live_observation_deadline = 999.
        self.bridge.deferred_confirmed_approach = {'observation_stamp_ns': self.now}
        self.bridge.on_object_confirmation(self.rejected())
        self.bridge.reset.assert_not_called()
        self.assertTrue(self.bridge.following)
        self.assertEqual(self.bridge.live_observation_deadline, 999.)
        self.assertIsNone(self.bridge.deferred_confirmed_approach)

    def test_candidate_comparison_also_records_negative_evidence(self):
        self.bridge.memory_selecting = True
        self.bridge.on_object_confirmation(self.rejected())
        self.assertEqual(self.bridge.confirmation_evidence_not_before_ns, self.now+1)
        self.bridge.memory_selecting = False
        self.bridge.on_object_confirmation(self.arrival(observation_stamp_ns=self.now-1))
        self.assertFalse(self.bridge.object_found)

    def test_repeated_old_other_query_and_other_instance_rejections_are_ignored(self):
        self.bridge.active_object_confirmation = dict(query='house',
            observation_stamp_ns=self.now, instance_id='new')
        for changes in (dict(query='station'), dict(instance_id='old'),
                        dict(observation_stamp_ns=self.now-10_000_000_000)):
            self.bridge.on_object_confirmation(self.rejected(**changes))
        self.assertEqual(self.bridge.confirmation_evidence_not_before_ns, 0)
        message = self.rejected(instance_id='new')
        self.bridge.on_object_confirmation(message)
        fence = self.bridge.confirmation_evidence_not_before_ns
        self.bridge.on_object_confirmation(message)
        self.assertEqual(fence, self.bridge.confirmation_evidence_not_before_ns)

    def test_same_frame_ambiguous_depth_preserves_bearing_guidance(self):
        self.bridge.on_object_confirmation(self.rejected())
        self.bridge.activate_visible_approach = Mock()
        hint = dict(query='house', frame='spot1/odom', observation_stamp_ns=self.now,
            point=[5., 0.], bearing=0., evidence='bearing_only',
            position_confirmed=False, search_confirmed=False)
        self.bridge.on_visible_approach(String(data=json.dumps(hint)))
        self.assertEqual(self.bridge.visible_hint, hint)
        self.bridge.activate_visible_approach.assert_called_once()
        self.assertFalse(self.bridge.object_found)

    def test_evidence_fence_survives_reselection_and_resets_on_new_query(self):
        self.bridge.on_object_confirmation(self.rejected())
        fence = self.bridge.confirmation_evidence_not_before_ns
        self.bridge.init_observation(preserve_instance=True)
        self.assertEqual(self.bridge.confirmation_evidence_not_before_ns, fence)
        self.advance()
        self.bridge.on_query(String(data='gas station'))
        self.assertEqual(self.bridge.confirmation_evidence_not_before_ns, 0)
        self.bridge.on_object_confirmation(self.rejected(query='house'))
        self.assertEqual(self.bridge.confirmation_evidence_not_before_ns, 0)

    def test_missing_or_ambiguous_association_cannot_arrive(self):
        for changes in (dict(localization_valid=False), dict(localization_valid=None),
                        dict(quality=None), dict(quality={'association_validated': False}),
                        dict(quality={'association_validated': True, 'confidence': .1}),
                        dict(quality={'association_validated': True, 'confidence': float('nan')})):
            self.bridge.on_object_confirmation(self.arrival(**changes))
        self.assertFalse(self.bridge.object_found)
        self.bridge.on_object_confirmation(self.arrival())
        self.assertTrue(self.bridge.object_found)

    def test_duplicate_or_late_rejection_does_not_clear_new_goal_evidence(self):
        self.localize(localization_valid=False, reason='ambiguous_depth_surfaces')
        rejected_stamp = self.now
        for _ in range(4):
            self.localize()
        anchor = deepcopy(self.goal.confirmed_instance_anchor)
        self.localize(observation_stamp_ns=rejected_stamp, localization_valid=False,
                      reason='ambiguous_depth_surfaces')
        self.assertEqual(self.goal.confirmed_instance_anchor, anchor)
        self.assertEqual(len(self.goal.observations), 4)

    def test_actual_triangulation_callback_cannot_fall_back_from_ambiguous_scan(self):
        source = VIS/'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'ObjectMaskTriangulator')
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                    ('process_observation', 'check_lidar_in_mask', 'publish_localization_rejection',
                     'stamp_ns', 'prune_views', 'publish_navigation_goal_and_marker')]
        sample = self.fixture()
        namespace = dict(np=np, R=R, json=json, String=String,
            point_cloud2=S(read_points=lambda *a, **kw: sample['points_cam']),
            rclpy=S(time=S(Time=S(from_msg=lambda s: s))), Duration=lambda **kw: None,
            foreground_depth=foreground_depth, ground_like_surface=Mock(),
            triangulate_bearings=Mock(return_value={'position': np.zeros(3)}))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                     str(source), 'exec'), namespace)
        node = namespace['ObjectMaskTriangulator']()
        obs = S(query='house', header=S(stamp=S(sec=100, nanosec=0)),
                odom=S(pose=S(pose=S(position=S(x=0., y=0., z=0.)))))
        node.active_query = 'house'
        node.query_started_ns = node.last_observation_ns = 0
        node.observation_timeout = 5.
        node.global_frame = 'spot1/odom'
        node.lidar_tf_frame = 'lidar'
        node.get_clock = lambda: self.clock
        node.get_logger = Mock()
        node.views = ['old', 'old2']
        node.bearing_views = [{'stamp_ns': 99*10**9}, {'stamp_ns': 99*10**9}]
        node.prev_view_pos = np.array([3., 0., 0.])
        node.min_lidar_points, node.sparse_lidar_points = 20, 8
        node.localization_pub, node.nav_goal_publisher = Mock(), Mock()
        lidar = S(header=obs.header)
        node.msg_buffer = S(buffer=[1], pop_oldest_msg=lambda: ({'obj_mask': obs, 'lidar': lidar}, None, None))
        cam = dict(object_mask=sample['mask'], K=sample['k'], R_wc=np.eye(3),
                   t_wc=np.zeros((3, 1)))
        node.extract_data_from_obj_mask_msg = Mock(return_value=[cam])
        transform = S(rotation=S(x=0., y=0., z=0., w=1.), translation=S(x=0., y=0., z=0.))
        node.tf_buffer = S(lookup_transform=lambda *a, **kw: S(transform=transform))
        node.process_observation()
        namespace['triangulate_bearings'].assert_not_called()
        node.nav_goal_publisher.publish.assert_not_called()
        self.assertEqual(node.bearing_views, [])
        result = json.loads(node.localization_pub.publish.call_args.args[0].data)
        self.assertFalse(result['localization_valid'])
        self.assertEqual(len(result['depth_candidates']), 2)
        node.triangulated_position = np.zeros(3)
        node.publish_navigation_goal_and_marker()
        node.nav_goal_publisher.publish.assert_not_called()
