"""Recorded calibration and production-callback regressions; no live ROS/BeamNG."""
import ast
import json
import math
import time
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
from concurrent.futures import Future
import unittest
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from beamng_wildos_adapter.geometry import (
    body_rotation, shifted_body_position, optical_rotation, matrix_quaternion)
from wildos_nav2_adapter.sensor_geometry import SBR_CAMERA_OFFSET, observation_clearance
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
import test_execution_mechanism_ros as _execution_fixture

ROOT = Path(__file__).resolve().parents[2]


class SensorCalibrationTests(unittest.TestCase):
    def test_production_image_pose_matches_all_three_measured_samples(self):
        source = ROOT/'official_adapter/beamng_wildos_adapter/node.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Adapter')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'process_image')
        namespace = dict(body_rotation=body_rotation, shifted_body_position=shifted_body_position,
            optical_rotation=optical_rotation, matrix_quaternion=matrix_quaternion,
            xyz=lambda p: np.array([p.x, p.y, p.z]),
            Image=lambda: S(header=S()), CameraInfo=S, math=math)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        samples = json.loads((ROOT/'captures/20261008_B_green_house_station_trial12_repeat_geometry/'
                            'stationary_sensor_calibration.json').read_text())['samples']
        adapter = S(base_offset=np.array(SBR_BASE_OFFSET), camera_offset=np.array(SBR_CAMERA_OFFSET),
                    publish_tf=Mock(), image_pub=Mock(), info_pub=Mock(), publish_measurement_timing=Mock(),
                    measurement_output_clock=lambda: 'ros')
        def vector(value): return S(x=value[0], y=value[1], z=value[2])
        for sample in samples:
            state = S(position=vector(sample['native_position']),
                      dir=vector(sample['native_direction']), up=vector([0., 0., 1.]))
            image = S(header=S(stamp=S(sec=100, nanosec=0)), encoding='rgb8', width=960,
                      height=640, is_bigendian=False, step=2880, data=b'')
            namespace['process_image'](adapter, image, state, image.header.stamp)
            position = adapter.publish_tf.call_args.args[2]
            np.testing.assert_allclose(position, sample['camera_actual'], atol=2e-5)
            nav = shifted_body_position(np.array(sample['native_position']),
                body_rotation(state.dir, state.up), SBR_BASE_OFFSET)
            np.testing.assert_allclose(nav, sample['nav_position'], atol=2e-5)

    def test_clearance_includes_camera_lever_arm_and_endpoint_tolerance(self):
        radius = observation_clearance(SBR_CAMERA_OFFSET, 1., .6)
        # Even a 0.6 m endpoint overshoot leaves at least 1 m at the sensor.
        self.assertGreaterEqual(radius-.6-math.hypot(*SBR_CAMERA_OFFSET[:2]), 1.-1e-9)
        for invalid in ([math.nan, 0., 0.], [1., 2.]):
            with self.assertRaises(ValueError): observation_clearance(invalid, 1., .6)

    def test_too_close_pose_moves_back_to_sensor_clearance_ring(self):
        from beamng_wildos_adapter.geometry import standoff_xy
        from beamng_wildos_adapter.mapping import reachable_ring_candidate
        radius = observation_clearance(SBR_CAMERA_OFFSET, 1., .6)
        point = standoff_xy((17., 0.), (20., 0.), radius, maintain_distance=True)
        self.assertAlmostEqual(20.-point[0], radius)
        self.assertLess(point[0], 17.)
        grid = np.zeros((60, 60), dtype=np.int16)
        local = reachable_ring_candidate(grid, .5, (-15., -15.), (3., 0.), radius,
                                         maintain_distance=True)
        self.assertAlmostEqual(math.hypot(local[0]-3., local[1]), radius)
        self.assertLess(local[0], 0.)


class ObservationLifecycleTests(unittest.TestCase):
    confirmation = _execution_fixture.ExecutionMechanismTests.confirmation

    def setUp(self):
        _execution_fixture.ExecutionMechanismTests.setUp(self)
        self.node.carrier_model = 'ideal_point'
        self.goal = GoalStandoff()
        self.goal.get_clock = self.node.get_clock
        self.goal.on_odom(deepcopy(self.node.odom))
        self.goal.on_query(String(data=self.node.query))
        self.anchor = dict(query=self.node.query, object_position=[3.2, 0., 2.],
            observation_stamp_ns=self.now, instance_id='instance-a', radius=.5)
        self.goal.confirmed_instance_anchor = dict(self.anchor)
        self.node.active_object_confirmation = dict(self.anchor)
        self.node.confirmed_route_object = (3.2, 0.)
        self.node.instance_release_pub = Mock()
        self.goal.instance_ack_pub = Mock()

    def tearDown(self):
        self.goal.destroy_node()
        _execution_fixture.ExecutionMechanismTests.tearDown(self)

    def test_no_new_geometry_requests_new_view_and_not_success(self):
        self.node.observation_wait_until = time.monotonic()-1
        self.node.monitor_observation()
        self.assertEqual(self.node.last_status['status'], 'target_observation_approach')
        self.assertFalse(self.node.object_found)
        self.assertGreaterEqual(math.hypot(*self.node.expected), 1.1)
        self.assertGreaterEqual(math.hypot(self.node.expected[0]-3.2, self.node.expected[1]), 4.3519)
        expected_heading = math.atan2(-self.node.expected[1], 3.2-self.node.expected[0])
        self.assertAlmostEqual(self.node.exploration_observation_heading(), expected_heading)
        self.assertIsNone(self.node.instance_release_pending)

    def exhaust(self):
        self.node.observation_attempts = {(i, i) for i in range(4)}
        self.node.observation_wait_until = time.monotonic()-1
        self.node.monitor_observation()

    def test_exhaustion_waits_for_release_ack_before_graph_reset(self):
        self.exhaust()
        self.node.graph_reset_client.service_is_ready.return_value = True
        self.node.retry_graph_reset()
        self.node.graph_reset_client.call_async.assert_not_called()
        self.goal.on_instance_release(self.node.instance_release_pub.publish.call_args.args[0])
        self.assertIsNone(self.goal.confirmed_instance_anchor)
        self.node.on_instance_release_ack(self.goal.instance_ack_pub.publish.call_args.args[0])
        self.assertIsNone(self.node.instance_release_pending)
        self.node.graph_reset_client.call_async.assert_called_once()
        self.assertFalse(self.node.object_found)

    def test_duplicate_release_does_not_clear_a_newly_selected_instance(self):
        self.exhaust()
        request = self.node.instance_release_pub.publish.call_args.args[0]
        self.goal.on_instance_release(request)
        self.goal.confirmed_instance_anchor = dict(self.anchor, instance_id='instance-b')
        self.goal.on_instance_release(request)
        self.assertEqual(self.goal.confirmed_instance_anchor['instance_id'], 'instance-b')
        self.assertEqual(self.goal.instance_ack_pub.publish.call_count, 2)

    def test_wrong_instance_or_old_query_release_cannot_clear_current_anchor(self):
        self.exhaust()
        request = json.loads(self.node.instance_release_pub.publish.call_args.args[0].data)
        for change in (dict(instance_id='wrong'), dict(query='house'),
                       dict(request_stamp_ns=self.now-1)):
            self.goal.on_instance_release(String(data=json.dumps(dict(request, **change))))
        self.assertIsNotNone(self.goal.confirmed_instance_anchor)
        # An explicitly different instance is already absent; acknowledge that
        # fact without releasing the current one. Stale/query-mismatched
        # requests remain rejected.
        self.assertEqual(self.goal.instance_ack_pub.publish.call_count, 1)

    def test_missing_consumer_identity_releases_only_pre_fence_producer_state(self):
        self.node.active_object_confirmation = None
        self.node.upstream_instance_id = ''
        self.exhaust()
        self.goal.on_instance_release(self.node.instance_release_pub.publish.call_args.args[0])
        self.assertIsNone(self.goal.confirmed_instance_anchor)
        self.node.on_instance_release_ack(self.goal.instance_ack_pub.publish.call_args.args[0])
        self.assertIsNone(self.node.instance_release_pending)

    def test_unknown_identity_cannot_clear_a_post_fence_instance(self):
        self.node.active_object_confirmation = None
        self.node.upstream_instance_id = ''
        self.exhaust()
        fence = self.node.instance_release_pending['observation_fence_ns']
        self.now += 2
        self.goal.confirmed_instance_anchor = dict(self.anchor,
            instance_id='post-fence-instance', observation_stamp_ns=fence+1)
        self.node.retry_instance_release()
        request = self.node.instance_release_pub.publish.call_args.args[0]
        self.goal.on_instance_release(request)
        self.assertEqual(self.goal.confirmed_instance_anchor['instance_id'], 'post-fence-instance')
        self.node.on_instance_release_ack(self.goal.instance_ack_pub.publish.call_args.args[0])
        self.assertIsNone(self.node.instance_release_pending)
        self.goal.on_instance_release(request)
        self.assertEqual(self.goal.confirmed_instance_anchor['instance_id'], 'post-fence-instance')

    def test_confirmed_route_handover_clears_old_observation_timeout(self):
        self.node.active_object_confirmation['object_position'] = [20., 0.]
        self.node.confirmed_route_object = (20., 0.)
        self.assertTrue(self.node.start_live_observation())
        old_goal = self.node.expected
        self.node.odom.pose.pose.position.x, self.node.odom.pose.pose.position.y = old_goal
        self.node.route = None
        self.node.pending = self.node.following = False
        self.node.confirmed_route_started_at = time.monotonic()-11
        data = dict(query=self.node.query, object_position=[20., 0., 2.], radius=.5,
                    standoff=[15.648004908, 0.])
        self.node.route_confirmed_target(data, old_goal)
        self.assertEqual(self.node.expected, tuple(data['standoff']))
        self.assertIsNone(self.node.live_observation_deadline)
        self.assertIsNone(self.node.live_observation_target)
        self.assertFalse(self.node.observation_active)

    def test_malformed_and_delayed_ack_cannot_resume_a_new_query(self):
        self.exhaust()
        request = dict(self.node.instance_release_pending)
        bad = dict(request, released=True)
        bad.pop('request_stamp_ns')
        self.node.on_instance_release_ack(String(data=json.dumps(bad)))
        self.assertIsNotNone(self.node.instance_release_pending)
        self.node.on_query(String(data='house'))
        self.node.on_instance_release_ack(String(data=json.dumps(dict(request, released=True))))
        self.assertEqual(self.node.query, 'house')
        self.assertIsNone(self.node.instance_release_pending)

    def test_old_confirmation_and_waypoint_do_not_relock_after_release(self):
        self.exhaust()
        request = self.node.instance_release_pub.publish.call_args.args[0]
        self.goal.on_instance_release(request)
        self.node.on_instance_release_ack(self.goal.instance_ack_pub.publish.call_args.args[0])
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'spot1/odom'
        waypoint.header.stamp.sec = 99
        waypoint.pose.position.x = 40.
        self.node.on_waypoint(waypoint)
        self.assertIsNone(self.node.expected)
        stale = self.confirmation(self.node.query, [3.2, 0., 2.], [0., 0., 0.], [0., 0.])
        data = json.loads(stale.data); data['observation_stamp_ns'] = self.now-1
        self.node.on_object_confirmation(String(data=json.dumps(data)))
        self.assertIsNone(self.node.active_object_confirmation)

    def test_blocked_observation_is_bounded_even_without_graph_route(self):
        self.assertTrue(self.node.start_live_observation())
        self.node.live_observation_deadline = time.monotonic()-1
        self.node.observation_attempts = {(i, i) for i in range(4)}
        self.node.monitor_observation()
        self.assertIsNotNone(self.node.instance_release_pending)
        self.assertIsNone(self.node.expected)

    def test_old_graph_reset_callback_cannot_bypass_release_barrier(self):
        old_fence = self.node.confirmation_not_before_ns
        self.exhaust()
        future = Future(); future.set_result(S(success=True))
        self.node.on_graph_reset_done(future, self.node.query_generation, old_fence)
        self.assertTrue(self.node.query_reset_pending)
        self.goal.on_instance_release(self.node.instance_release_pub.publish.call_args.args[0])
        self.node.on_instance_release_ack(self.goal.instance_ack_pub.publish.call_args.args[0])
        self.node.on_graph_reset_done(future, self.node.query_generation, old_fence)
        self.assertTrue(self.node.query_reset_pending)

    def test_geometry_change_rejects_whole_follow_parameter_batch(self):
        before = self.node.follow_enabled
        result = self.node.on_parameters([
            S(name='follow_path', value=not before),
            S(name='camera_offset', value=[0., 0., 0.])])
        self.assertFalse(result.successful)
        self.assertEqual(self.node.follow_enabled, before)

    def test_memory_reselection_preserves_release_transaction_and_memory_inputs(self):
        self.node.upstream_instance_id = 'instance-a'
        view = dict(id='next-region', frame='spot1/odom', position=[15., 0., 2.],
                    heading=0., object_position=[40., 0., 2.])
        self.node.begin_memory_selection([view])
        self.assertTrue(self.node.memory_selecting)
        self.assertEqual(self.node.instance_release_pending['instance_id'], 'instance-a')
        self.assertEqual(self.node.confirmation_not_before_ns, self.now)
        self.assertEqual(self.node.memory_source_views, [view])
        self.goal.on_instance_release(self.node.instance_release_pub.publish.call_args.args[0])
        self.node.on_instance_release_ack(self.goal.instance_ack_pub.publish.call_args.args[0])
        self.assertTrue(self.node.memory_selecting)
        self.assertIsNone(self.node.instance_release_pending)
        self.assertEqual(self.node.memory_source_views, [view])

    def test_released_instance_accepts_a_different_supported_cluster(self):
        self.exhaust()
        self.goal.on_instance_release(self.node.instance_release_pub.publish.call_args.args[0])
        self.goal.publisher = Mock(); self.goal.confirmation_pub = Mock()
        for i in range(5):
            self.now += 100_000_000
            odom = deepcopy(self.node.odom)
            odom.header.stamp = self.node.get_clock().now().to_msg()
            self.goal.on_odom(odom)
            data = dict(query=self.node.query, frame='spot1/odom', observation_stamp_ns=self.now,
                position=[40., 0., 2.], observer_position=[0., 0., 0.],
                observer_orientation=[0., 0., 0., 1.], source='lidar', localization_valid=True,
                quality={'confidence': .9, 'association_validated': True})
            self.goal.on_localization(String(data=json.dumps(data)))
        self.assertIsNotNone(self.goal.confirmed_instance_anchor)
        self.assertAlmostEqual(self.goal.confirmed_instance_anchor['object_position'][0], 40.)
        self.assertNotEqual(self.goal.confirmed_instance_anchor['instance_id'], 'instance-a')

