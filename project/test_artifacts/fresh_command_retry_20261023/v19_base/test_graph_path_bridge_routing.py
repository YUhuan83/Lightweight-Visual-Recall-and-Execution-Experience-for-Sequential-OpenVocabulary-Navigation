import unittest
import time
import json
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch
from concurrent.futures import Future

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String

from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter.motion_path import motion_sections


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = "spot1/odom"
    p.header.stamp = rclpy.clock.Clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


def route(*points):
    path = Path()
    path.header.frame_id = "spot1/odom"
    path.header.stamp = rclpy.clock.Clock().now().to_msg()
    path.poses = [pose(*point) for point in points]
    return path


def drivable_route(*points):
    """Sample straight fixtures as a planner would, before RPP pose pruning.

    Long two-pose graph routes are legitimate graph input, but they are not
    executable controller paths: the first pose can become the rear carrot.
    """
    sampled=[points[0]]
    for a,b in zip(points,points[1:]):
        steps=max(1,math.ceil(math.dist(a,b)/.25))
        sampled.extend((a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps)
                       for i in range(1,steps+1))
    return route(*sampled)


def configure_ackermann(node):
    """Keep legacy car-specific callback tests explicit as bridge default becomes ideal."""
    node.carrier_model = "ackermann"
    node.planner_id = "CarLike"
    node.minimum_turning_radius = 8.0
    # Car callback fixtures model a ready actuator; missing/failed/stale
    # transport is covered separately in test_execution_transport_health_ros.
    node.native_status = dict(wall_time=0., speed=0., telemetry_ok=True,
        actuator_ok=True, control_acknowledged=True, telemetry_age_sec=0.)
    node.native_status_at = time.monotonic()
    return node


class GraphBridgeRoutingTests(unittest.TestCase):
    def test_default_runtime_selects_ideal_point_and_grid_planner(self):
        node = GraphPathBridge()
        try:
            self.assertEqual(node.carrier_model, 'ideal_point')
            self.assertEqual(node.planner_id, 'GridBased')
            self.assertEqual(node.minimum_turning_radius, 0.0)
            points = [(0., 0.), (1., 0.), (1., 1.), (2., 1.)]
            headings = [0., 0., math.pi/2, math.pi]
            self.assertIsNone(node.validate_path(
                points, headings, [(0., 0.), (2., 1.)], self.make_grid(), points[-1],
                start=points[0], target_heading=math.pi))
            node.status_pub = Mock()
            node.status('profile_probe')
            status = json.loads(node.status_pub.publish.call_args.args[0].data)
            self.assertEqual(status['schema_version'], 2)
            self.assertGreater(status['stamp_ns'], 0)
            self.assertEqual(status['carrier_model'], 'ideal_point')
            self.assertEqual(status['minimum_turning_radius_m'], 0.0)
            self.assertEqual(status['planner_id'], 'GridBased')
        finally:
            node.destroy_node()

    def test_ideal_rotation_counts_heading_progress_and_reports_a_kinematic_stall(self):
        self.node.carrier_model = 'ideal_point'
        self.node.following = True
        self.node.odom.pose.pose.orientation.w = 1.
        path = route((0., 0.), (0., 0.))
        path.poses[-1].pose.orientation.z = math.sin(math.pi/4)
        path.poses[-1].pose.orientation.w = math.cos(math.pi/4)
        self.node.motion_queue = [path]
        self.node.control_status = {'actuating': True, 'target_speed': 0., 'target_angular': .5}
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1000.):
            self.node.control_status_at = 1000.
            self.assertIsNone(self.node.execution_stall())
        self.node.odom.pose.pose.orientation.z = math.sin(.4/2)
        self.node.odom.pose.pose.orientation.w = math.cos(.4/2)
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1001.):
            self.node.control_status_at = 1001.
            self.assertIsNone(self.node.execution_stall())
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1006.1):
            self.node.control_status_at = 1006.1
            stalled = self.node.execution_stall()
        self.assertEqual(stalled['stall_class'], 'kinematic_pose_not_advancing')

    def stall_sample(self, at, *, heading=0., speed=0., angular=.5, x=0.):
        self.node.carrier_model = 'ideal_point'
        self.node.following = True
        self.node.odom.pose.pose.position.x = x
        self.node.odom.pose.pose.orientation.z = math.sin(heading/2)
        self.node.odom.pose.pose.orientation.w = math.cos(heading/2)
        self.node.control_status = dict(actuating=True, target_speed=speed, target_angular=angular)
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=at):
            self.node.control_status_at = at
            return self.node.execution_stall()

    def test_start_rotation_can_move_away_from_final_path_heading(self):
        # RPP initially aligns to its carrot, not to the final goal orientation.
        self.node.motion_queue = [route((0., 0.), (0., 3.), (3., 3.))]
        for index in range(8):
            self.assertIsNone(self.stall_sample(1000.+index, heading=.25*index))
        stalled = self.stall_sample(1012.1, heading=1.75)
        self.assertIsNotNone(stalled)  # Actual yaw stops despite a live command.

    def test_rotation_to_translation_starts_a_distance_progress_window(self):
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        self.assertIsNone(self.stall_sample(1000.))
        self.assertIsNone(self.stall_sample(1004.9, speed=1., angular=0.))
        self.assertIsNone(self.stall_sample(1005.1, speed=1., angular=0.))
        self.assertIsNotNone(self.stall_sample(1010., speed=1., angular=0.))

    def test_translation_to_rotation_does_not_inherit_distance_baseline(self):
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        self.assertIsNone(self.stall_sample(1000., speed=1., angular=0., x=5.))
        self.assertIsNone(self.stall_sample(1004.9, heading=.4, x=5.))
        self.assertIsNone(self.stall_sample(1005.1, heading=.7, x=5.))

    def test_same_mode_replan_does_not_clear_no_progress_timeout(self):
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        self.assertIsNone(self.stall_sample(1000., speed=1., angular=0.))
        self.node.motion_queue = [route((0., 0.), (20., 0.))]
        self.assertIsNotNone(self.stall_sample(1005.1, speed=1., angular=0.))

    def test_rotation_opposite_command_does_not_count_as_progress(self):
        self.node.motion_queue = [route((0., 0.), (3., 0.))]
        self.assertIsNone(self.stall_sample(1000., heading=1.))
        self.assertIsNotNone(self.stall_sample(1005.1, heading=.5))

    def test_rotation_progress_wraps_across_pi(self):
        self.node.motion_queue = [route((0., 0.), (3., 0.))]
        self.assertIsNone(self.stall_sample(1000., heading=3.))
        self.assertIsNone(self.stall_sample(1004.9, heading=-3.))
        self.assertIsNone(self.stall_sample(1005.1, heading=-3.))

    def test_rotation_threshold_is_independent_of_distance_threshold(self):
        self.node.motion_queue = [route((0., 0.), (3., 0.))]
        get_parameter = self.node.get_parameter
        with patch.object(self.node, 'get_parameter', side_effect=lambda name:
                SimpleNamespace(value=10.) if name == 'stall_progress_m' else get_parameter(name)):
            self.assertIsNone(self.stall_sample(1000.))
            self.assertIsNone(self.stall_sample(1004.9, heading=.3))
            self.assertIsNone(self.stall_sample(1005.1, heading=.3))

    def test_rotation_command_reversal_keeps_no_progress_timeout(self):
        self.node.motion_queue = [route((0., 0.), (3., 0.))]
        self.assertIsNone(self.stall_sample(1000.))
        stalled = self.stall_sample(1005.1, angular=-.5)
        self.assertIsNotNone(stalled)
        self.assertEqual(stalled['progress_kind'], 'heading_rad')
        self.assertEqual(stalled['target_angular'], -.5)

    def test_invalid_control_numbers_do_not_replace_live_execution_status(self):
        stamp = self.node.get_clock().now().nanoseconds
        for extra in ({'target_speed': 'invalid'}, {'target_angular': float('nan')},
                      {'target_angular': float('inf')}, {'target_speed': True},
                      {'stamp_ns': float('inf')}, {'stamp_ns': True}):
            with self.subTest(extra=extra):
                self.node.control_status = None
                data = dict(stamp_ns=stamp, actuating=True)
                data.update(extra)
                self.node.on_control_status(String(data=json.dumps(data)))
                self.assertIsNone(self.node.control_status)

    def test_control_numeric_strings_are_normalized_before_stall_calculation(self):
        self.node.on_control_status(String(data=json.dumps(dict(
            stamp_ns=self.node.get_clock().now().nanoseconds, actuating=True,
            target_speed='0.0', target_angular='0.5'))))
        self.assertIsInstance(self.node.control_status['target_speed'], float)
        self.assertIsInstance(self.node.control_status['target_angular'], float)

    def test_native_numeric_speed_is_normalized_before_slip_classification(self):
        self.node.on_native_status(String(data=json.dumps(dict(
            wall_time=time.time(), speed='0.0', actual={'wheelspeed': 1.}, wheel_contact=None))))
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        self.assertIsNone(self.stall_sample(1000., speed=1., angular=0.))
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1005.1):
            self.node.native_status_at = 1005.1
            stalled = self.stall_sample(1005.1, speed=1., angular=0.)
        self.assertEqual(stalled['stall_class'], 'wheel_slip_or_contact_loss')

    @staticmethod
    def make_grid():
        grid = OccupancyGrid()
        grid.info.resolution = .25
        grid.info.width = grid.info.height = 80
        grid.info.origin.position.x = grid.info.origin.position.y = -10.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 6400
        return grid

    def prepare_escape(self):
        self.node.follow_enabled = True
        self.node.odom.pose.pose = pose(6., 0.).pose
        self.node.route = route((6., 0.), (9., 0.))
        self.node.route_endpoint = (9., 0.)
        now = time.monotonic()
        self.node.escape_trace.extend((i*.3, 0., 0., now-3.+i*.1) for i in range(21))
        self.node.reject_exploration_route = Mock(return_value=True)

    def test_stall_location_survives_changed_remote_frontier(self):
        self.prepare_escape()
        self.node.send_motion_segment = Mock()
        self.node.follow_failed('execution_stalled', stall_class='actuated_no_path_progress')
        self.node.route_endpoint = (15., 5.)
        self.node.recovery_attempts = 0
        self.node.follow_failed('execution_stalled', stall_class='actuated_no_path_progress')
        self.assertTrue(self.node.recovering)
        self.node.send_motion_segment.assert_called_once()
        self.assertLess(self.node.escape_path.poses[-1].pose.position.x, 4.1)

    def test_escape_respects_current_map_and_does_not_erase_memory(self):
        self.prepare_escape()
        self.node.follow_client = Mock()
        self.node.grid.data[10*40+13] = 100
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertFalse(self.node.recovering)
        self.assertEqual(self.node.last_status['status'], 'follow_failed')
        self.assertEqual(self.node.last_status['recovery_reason'], 'motion_section_invalid')
        self.node.expected = self.node.memory_goal = (9., 0.)
        self.node.send_motion_segment = Mock()
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.send_motion_segment.assert_not_called()
        self.assertEqual(self.node.memory_goal, (9., 0.))

    def test_failed_escape_retries_latest_graph_route(self):
        self.prepare_escape()
        self.node.recovering = True
        self.node.escape_path = route((6., 0.), (3.9, 0.))
        self.node.latest_route = route((6., 0.), (15., 5.))
        self.node.latest_route_complete = False
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.cancel_goal = Mock()
        self.node.start_segment = Mock()

        self.node.finish_execution_escape(False, 'escape_action_failed', action_status=6)

        self.assertFalse(self.node.recovering)
        self.assertEqual(self.node.route_endpoint, (15., 5.))
        self.assertEqual(self.node.last_status['status'], 'follow_failed')
        self.assertEqual(self.node.last_status['recovery_reason'], 'escape_action_failed')
        self.node.blocked_until = time.monotonic() - 1.
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once()

    def test_escape_acceptance_does_not_learn_rejected_forward_route(self):
        self.prepare_escape()
        self.node.send_motion_segment = Mock()
        for _ in range(2):
            self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.checkpoint_execution_feedback = Mock()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = Future()
        future = Future(); future.set_result(handle)
        self.node.on_follow_accepted(future, self.node.epoch, (3.9, 0.))
        self.node.checkpoint_execution_feedback.assert_not_called()

    def test_stalled_explicit_target_can_escape_and_resume_same_goal(self):
        self.prepare_escape()
        target = (12., 3.)
        self.node.expected = target
        self.node.send_motion_segment = Mock()
        self.node.reject_exploration_route = Mock(return_value=False)

        for _ in range(2):
            self.node.follow_failed('execution_stalled',
                                    stall_class='drive_limited_or_physical_contact')

        self.assertTrue(self.node.recovering)
        self.node.send_motion_segment.assert_called_once()
        self.assertEqual(self.node.expected, target)
        self.assertIsNone(self.node.memory_goal)

    def test_airborne_wheel_spin_uses_measured_reverse_corridor(self):
        self.prepare_escape()
        self.node.send_motion_segment = Mock()

        self.node.follow_failed('execution_stalled', stall_class='wheel_airborne_spin')
        self.node.follow_failed('execution_stalled', stall_class='wheel_airborne_spin')

        self.assertTrue(self.node.recovering)
        self.assertIsNotNone(self.node.escape_path)
        self.assertEqual(self.node.last_status['status'], 'recovery_started')
        self.node.send_motion_segment.assert_called_once()

    def test_frontier_observation_heading_is_prioritized(self):
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        from wildos_nav2_adapter.input_quality import stamp_ns
        self.node.route_metadata[stamp_ns(self.node.route)] = dict(complete=True, observation_heading=1.2)
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertEqual(self.node.candidates[0][:3], (6., 0., 1.2))

    def test_empty_path_budget_covers_near_and_far_route_samples(self):
        # v7 223039: the first four failures only tried the far end of a
        # shared prefix; three different frontiers repeated that prefix.
        self.node.route = route((0., 0.), (24., 0.))
        self.node.route_endpoint = (24., 0.)
        self.node.follow_enabled = True
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertEqual([c[0] for c in self.node.candidates[:3]], [24., 22., 20.])
        for _ in range(self.node.max_consecutive_empty_paths-1):
            self.node.candidate_failed('empty_path')
        # The existing ten-sample cap retains 24..6 m on this straight route.
        self.assertEqual(self.node.candidates[self.node.candidate_index][:2], (6., 0.))
        self.node.block_current_route = Mock()
        self.node.reject_exploration_route = Mock()
        self.node.candidate_failed('empty_path')
        self.node.reject_exploration_route.assert_called_once()

    def test_reverse_prefix_candidates_use_the_same_cache_and_pose_filter(self):
        self.node.route = route((0., 0.), (-6., 0.))
        self.node.route_endpoint = (-6., 0.)
        self.node.follow_enabled = True
        self.node.try_candidate = Mock()
        self.node.goal_recently_failed = Mock(side_effect=lambda start, point, heading:
            point == (-6., 0.) and abs(math.remainder(heading, 2*math.pi)) < .1)
        self.node.vehicle_pose_feasible = Mock(side_effect=lambda point, heading:
            not (point == (-4., 0.) and abs(math.remainder(heading, 2*math.pi)) < .1))
        self.node.start_segment()
        self.node.candidate_failed('empty_path')
        self.node.candidate_failed('empty_path')
        self.assertTrue(self.node.candidates)  # Original-direction alternatives remain.
        self.assertFalse(any(abs(math.remainder(c[2], 2*math.pi)) < .1
                             for c in self.node.candidates))

    def test_ideal_carrier_keeps_original_prefix_order(self):
        self.node.carrier_model = 'ideal_point'
        self.node.route = route((0., 0.), (24., 0.))
        self.node.route_endpoint = (24., 0.)
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertEqual([c[0] for c in self.node.candidates[:4]], [24., 22., 20., 18.])

    def test_backward_graph_prefix_can_keep_vehicle_heading(self):
        # A graph tangent points along travel; it is not the car's required
        # body heading when the same intermediate point is reached in reverse.
        self.node.route = route((0., 0.), (-6., 0.))
        self.node.route_endpoint = (-6., 0.)
        self.node.follow_enabled = True
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertAlmostEqual(abs(self.node.candidates[0][2]), math.pi)
        self.node.candidate_failed('empty_path')
        self.node.candidate_failed('empty_path')
        probe = self.node.candidates[self.node.candidate_index]
        self.assertEqual(probe[:2], (-4., 0.))
        self.assertAlmostEqual(math.remainder(probe[2], 2*math.pi), 0.)
        self.assertTrue(any(c[:2] == (-6., 0.) and
            abs(math.remainder(c[2]-math.pi, 2*math.pi)) < 1e-6
            for c in self.node.candidates))  # The original heading is still an option.

    def test_visible_observation_heading_is_not_reversed(self):
        self.node.route = route((0., 0.), (-6., 0.))
        self.node.route_endpoint = (-6., 0.)
        self.node.expected = (-6., 0.)
        self.node.visible_approach_active = True
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertTrue(all(abs(math.remainder(c[2]-math.pi, 2*math.pi)) < 1e-6
                            for c in self.node.candidates))

    def test_ordinary_first_failure_keeps_original_next_prefix(self):
        self.node.route = route((0., 0.), (24., 0.))
        self.node.route_endpoint = (24., 0.)
        self.node.follow_enabled = True
        self.node.try_candidate = Mock()
        self.node.start_segment()
        original = list(self.node.candidates)
        self.node.candidate_failed('empty_path')
        self.assertEqual(self.node.candidates, original)
        self.assertEqual(self.node.candidate_index, 1)

    def test_short_probe_preserves_required_heading_beyond_graph_endpoint(self):
        from wildos_nav2_adapter.input_quality import stamp_ns
        self.node.follow_enabled = True
        self.node.route = route((0.,0.),(-2.,0.))
        self.node.route_endpoint = (-2.,0.)
        self.node.route_metadata[stamp_ns(self.node.route)] = dict(
            exploration_id='required:1', complete=True, goal=[-4.,0.],
            observation_heading=math.pi)
        self.node.candidates = [(-4.,0.,math.pi,4.)]
        self.node.candidate_index = 0
        self.assertFalse(self.node.reserve_short_route_probe())
        self.assertEqual(len(self.node.candidates), 1)

    def test_route_candidates_that_collide_with_vehicle_body_are_skipped(self):
        # 093254 epoch 6: seven route samples passed the centre-cell check but
        # the body footprint overlapped lethal cells, so every ComputePath
        # failed serially. The endpoint filter must leave only free samples.
        self.node.follow_enabled = False
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        # Thick wall across the route end; x=8/10 have no clear body pose
        # within goal tolerance while x=4 remains free.
        for ix in range(18, 23):
            for iy in range(9, 11):
                self.node.grid.data[iy*40+ix] = 100
        self.node.try_candidate = Mock()
        self.node.start_segment()
        xs = [candidate[0] for candidate in self.node.candidates]
        self.assertTrue(xs)
        self.assertTrue(all(x < 8. for x in xs), xs)

    def test_clear_pose_within_goal_tolerance_keeps_colliding_endpoint(self):
        # 093254 candidate 5: the exact graph pose overlapped a lethal cell but
        # the planner stopped just short and produced a valid 62-pose path.
        # The endpoint filter must not discard that usable sample.
        self.node.follow_enabled = False
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        for iy in range(8, 12):
            self.node.grid.data[iy*40+23] = 100  # x=13, exact 10 m body only
        self.node.try_candidate = Mock()
        self.node.start_segment()
        xs = [candidate[0] for candidate in self.node.candidates]
        self.assertIn(10., xs)

    def test_failed_goal_cache_is_scoped_to_exploration_and_start_region(self):
        self.node.sync_failed_goal_scope('a:1')
        self.node.remember_failed_goal((0., 0.), (10., 0.), 0.)
        self.assertTrue(self.node.goal_recently_failed((0., 0.), (10., 0.), 0.))
        self.assertFalse(self.node.goal_recently_failed((3., 0.), (10., 0.), 0.))
        self.assertFalse(self.node.goal_recently_failed((0., 0.), (10., 0.), 1.))
        self.node.sync_failed_goal_scope('b:2')
        self.assertFalse(self.node.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_query_change_clears_cached_planner_failures(self):
        self.node.sync_failed_goal_scope('a:1')
        self.node.remember_failed_goal((0., 0.), (10., 0.), 0.)
        self.assertTrue(self.node.goal_recently_failed((0., 0.), (10., 0.), 0.))
        self.node.on_query(String(data='bus stop'))
        self.assertEqual(self.node.failed_goals, {})
        self.assertIsNone(self.node.failed_goal_scope)
        self.assertFalse(self.node.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_side_lookback_turns_back_once_near_large_structure(self):
        self.node.query = 'house'
        self.node.query_origin = (-30., 0.)
        self.node.query_started_ns = self.node.get_clock().now().nanoseconds - 21_000_000_000
        self.node.odom_received_at = time.monotonic()
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.local_freshness_issue = Mock(return_value=None)
        self.node.pose_blockers = Mock(return_value=[])
        self.node.relay_destination = Mock()
        # A large occupied cluster 1-3 m around the vehicle, excluding its cell.
        for ix in range(7, 14):
            for iy in range(7, 14):
                if (ix, iy) != (10, 10):
                    self.node.grid.data[iy*40+ix] = 100
        self.assertTrue(self.node.maybe_side_lookback((0., 0.), 0.))
        self.assertTrue(self.node.lookback_active)
        self.assertIsNotNone(self.node.expected)
        self.assertLess(self.node.expected[0], 0.)
        self.node.relay_destination.assert_called_once()

    def test_route_filter_falls_back_when_all_candidates_rejected(self):
        self.node.follow_enabled = False
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        for ix in range(15, 20):
            for iy in range(8, 13):
                self.node.grid.data[iy*40+ix] = 100
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertTrue(self.node.candidates)
        self.assertEqual(self.node.last_status['status'], 'route_filter_fallback')

    def test_route_filter_fallback_does_not_readd_cached_planner_failures(self):
        # Cached empty_path goals are exactly the searches this cache avoids.
        # When every sample is cached, retire the route instead of re-planning.
        self.node.follow_enabled = False
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.plan_client = Mock()
        self.node.try_candidate = Mock()
        candidates = [(6., 0., 0., 6.), (8., 0., 0., 8.)]
        with patch('wildos_nav2_adapter.graph_path_bridge.route_candidates',
                   return_value=candidates):
            for x, y, heading, _ in candidates:
                self.node.remember_failed_goal((0., 0.), (x, y), heading)
            self.node.start_segment()
        self.assertEqual(self.node.candidates, [])
        self.assertEqual(self.node.last_status['status'], 'no_free_graph_candidate')

    def test_visible_approach_uses_local_fallback_when_graph_has_no_candidate(self):
        self.node.visible_approach_active = True
        self.node.expected = (10., 0.)
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.try_candidate = Mock()
        with patch('wildos_nav2_adapter.graph_path_bridge.route_candidates',
                   return_value=[]):
            self.node.start_segment()
        self.assertEqual(self.node.candidates[0][:2], (10., 0.))
        self.assertEqual(self.node.last_status['status'],
                         'visible_target_local_fallback')

    def test_route_candidate_with_cached_empty_path_is_skipped(self):
        from wildos_nav2_adapter.input_quality import stamp_ns
        self.node.follow_enabled = False
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.route_metadata[stamp_ns(self.node.route)] = dict(
            exploration_id='a:1', complete=True, goal=[10., 0.])
        self.node.sync_failed_goal_scope('a:1')
        self.node.remember_failed_goal((0., 0.), (10., 0.), 0.)
        self.node.try_candidate = Mock()
        self.node.start_segment()
        xs = [candidate[0] for candidate in self.node.candidates]
        self.assertTrue(xs)
        self.assertNotIn(10., xs)
        self.assertIn(8., xs)

    def test_repeated_stall_postpones_exploration_after_two_retries(self):
        self.node.reject_exploration_route = Mock(return_value=True)
        self.node.recovery_attempts = 0
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.reject_exploration_route.assert_not_called()
        self.node.recovery_attempts = 2
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.reject_exploration_route.assert_called_once()

    def test_repeated_memory_route_stall_rechecks_alternate_views(self):
        views = [{'id': 'remembered-a'}, {'id': 'remembered-b'}]
        self.node.memory_source_views = views
        self.node.memory_active = views[0]
        self.node.memory_goal = self.node.expected = (8., 0.)
        self.node.recovery_attempts = 2
        self.node.begin_memory_selection = Mock()

        self.node.follow_failed('execution_stalled',
                                stall_class='drive_limited_or_physical_contact')

        self.node.begin_memory_selection.assert_called_once_with(views, {'remembered-a'})
        self.assertEqual(self.node.memory_goal, (8., 0.))
        self.node.memory_feedback_pub.publish.assert_not_called()

    def test_repeated_local_collision_postpones_frontier_before_budget_exhaustion(self):
        self.node.follow_enabled = True
        self.node.route = route((684.95, -31.31), (684.5423, -17.3379))
        self.node.route_endpoint = (684.5423, -17.3379)
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.route_rejection_pub = Mock()
        for attempt in (0, 1):
            self.node.recovery_attempts = attempt
            self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')
            self.node.route_rejection_pub.publish.assert_not_called()
        self.node.recovery_attempts = 2
        self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')
        message = self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(message.header.frame_id, self.node.route.header.frame_id)
        self.assertEqual(message.header.stamp.sec, self.node.route.header.stamp.sec)
        self.assertEqual(message.header.stamp.nanosec, self.node.route.header.stamp.nanosec)
        self.assertEqual(message.pose.position.x, self.node.route_endpoint[0])
        self.assertEqual(message.pose.position.y, self.node.route_endpoint[1])
        self.assertEqual(self.node.last_status['retry_after_sec'], 8.)
        self.assertFalse(self.node.following)

    def test_repeated_visible_target_collision_releases_blocked_hint_after_map_retry(self):
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (8., 0.))
        self.node.route_endpoint = (8., 0.)
        self.node.expected = (8., 0.)
        self.node.visible_approach_active = True
        self.node.release_visible_approach = Mock()
        self.node.block_current_route = Mock()
        self.node.recovery_attempts = 0
        self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')
        self.node.release_visible_approach.assert_not_called()
        self.assertEqual(self.node.last_status['retry_after_sec'], 2.)
        self.node.recovery_attempts = 1
        self.node.block_current_route.reset_mock()
        self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')
        self.node.release_visible_approach.assert_called_once_with('visible_target_path_blocked')
        self.node.block_current_route.assert_not_called()

    def test_repeated_collision_does_not_release_memory_or_side_lookback_goal(self):
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (8., 0.))
        self.node.route_endpoint = (8., 0.)
        self.node.expected = (8., 0.)
        self.node.visible_approach_active = True
        self.node.recovery_attempts = 1
        self.node.release_visible_approach = Mock()
        for guard in ({'memory_goal': (8., 0.)},
                      {'memory_active': {'position': (8., 0.)}},
                      {'lookback_active': True}):
            with self.subTest(guard=guard):
                with patch.multiple(self.node, **guard):
                    self.node.follow_failed('motion_section_invalid',
                                            detail='local_footprint_collision')
                self.node.release_visible_approach.assert_not_called()
    def test_local_collision_exhaustion_preserves_destination_and_query_guards(self):
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.route_rejection_pub = Mock()
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.recovery_attempts = 3
        cases = ({'expected': (10., 0.)}, {'memory_goal': (10., 0.)},
                 {'query_reset_pending': True}, {'query_started_ns': 10**30},
                 {'follow_enabled': False})
        for case in cases:
            with self.subTest(case=case):
                with patch.multiple(self.node, **case):
                    self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')
                self.node.route_rejection_pub.publish.assert_not_called()
        self.node.input_freshness_issue.return_value = 'stale_odom'
        self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')
        self.node.route_rejection_pub.publish.assert_not_called()

    def test_other_section_refusals_do_not_claim_repeated_local_collision(self):
        self.node.recovery_attempts = 3
        self.node.reject_exploration_route = Mock()
        for detail in ('excess_curvature', 'stale_local_costmap', None):
            self.node.follow_failed('motion_section_invalid', detail=detail)
        self.node.reject_exploration_route.assert_not_called()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = configure_ackermann(GraphPathBridge())
        self.node.require_route_status = False  # Existing legacy route fixtures.
        self.node.waypoint_pub = Mock()
        self.node.path_pub = Mock()
        self.node.status_pub = Mock()
        self.node.memory_feedback_pub = Mock()
        self.node.memory_selection_pub = Mock()
        self.node.graph_reset_client = Mock()
        self.node.graph_search_reset_client = self.node.graph_reset_client
        self.node.graph_reset_client.service_is_ready.return_value = True
        def cleared(_request):
            future = Future()
            future.set_result(Mock(success=True))
            return future
        self.node.graph_reset_client.call_async.side_effect = cleared
        odom = Odometry()
        odom.header.frame_id = "spot1/odom"
        odom.pose.pose.position.x = 0.
        self.node.odom = odom
        self.node.odom_received_at = time.monotonic()
        grid = OccupancyGrid()
        grid.header.frame_id = "spot1/odom"
        grid.info.resolution = 1.
        grid.info.width = 40
        grid.info.height = 40
        grid.info.origin.position.x = -10.
        grid.info.origin.position.y = -10.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 1600
        self.node.grid = grid
        self.node.grid_received_at = time.monotonic()
        self.node.grid_arrived_at = time.monotonic()
        from copy import deepcopy
        self.node.local_grid = deepcopy(grid)
        self.node.local_grid_received_at = time.monotonic()

    def prepare_synchronous_dispatch_result(self, path=None):
        node=self.node
        node.follow_enabled=True
        node.route=node.latest_route=route((0.,0.),(10.,0.))
        node.route_endpoint=(10.,0.)
        node.odom.pose.pose.orientation.w=1.
        node.candidates=[(10.,0.,0.,10.)]
        node.candidate_index=0
        node.pending=True
        node.active_plan_serial=42
        node.send_motion_segment=Mock()
        node.full_path_pub=Mock()
        node.validate_path=Mock(wraps=node.validate_path)
        node.initial_tracking_issue=Mock(wraps=node.initial_tracking_issue)
        path=path or drivable_route((0.,0.),(10.,0.))
        future=Future();future.set_result(SimpleNamespace(status=4,result=SimpleNamespace(path=path)))
        return future

    def test_synchronous_result_reuses_same_input_full_path_validation(self):
        future=self.prepare_synchronous_dispatch_result()
        self.node.on_plan_result(future,self.node.epoch,0,42)
        self.node.validate_path.assert_called_once()
        self.node.initial_tracking_issue.assert_called_once()
        self.node.send_motion_segment.assert_called_once()

    def test_ideal_result_keeps_synchronous_validation_reuse(self):
        future=self.prepare_synchronous_dispatch_result()
        self.node.carrier_model='ideal_point'
        self.node.on_plan_result(future,self.node.epoch,0,42)
        self.node.validate_path.assert_called_once()
        self.node.send_motion_segment.assert_called_once()

    def test_earlier_heading_winner_is_revalidated_on_current_map(self):
        future=self.prepare_synchronous_dispatch_result(drivable_route((0.,0.),(4.,0.),(2.,0.),(10.,0.)))
        earlier=drivable_route((0.,0.),(10.,0.))
        self.node.normal_best=(earlier,(10.,0.),1.)
        self.node.validate_path.side_effect=[None,'footprint_collision']
        self.node.on_plan_result(future,self.node.epoch,0,42)
        self.assertEqual(self.node.validate_path.call_count,2)
        self.node.send_motion_segment.assert_not_called()
        self.assertEqual(self.node.last_status['status'],'path_changed_before_execution')

    def test_input_expiring_during_validation_still_prevents_dispatch(self):
        future=self.prepare_synchronous_dispatch_result()
        def expire(*args):
            self.node.odom_received_at=time.monotonic()-2.
            return None
        self.node.initial_tracking_issue.side_effect=expire
        self.node.on_plan_result(future,self.node.epoch,0,42)
        self.node.send_motion_segment.assert_not_called()
        self.assertEqual(self.node.last_status['status'],'stale_odom')

    def test_visible_fallback_keeps_distinct_execution_corridor_check(self):
        future=self.prepare_synchronous_dispatch_result()
        self.node.visible_approach_active=True
        self.node.expected=(10.,0.)
        self.node.route=route((0.,0.),(5.,2.),(10.,0.))
        self.node.validate_path.side_effect=[None,'route_corridor_violation']
        self.node.on_plan_result(future,self.node.epoch,0,42)
        self.assertEqual(self.node.validate_path.call_count,2)
        self.node.send_motion_segment.assert_not_called()
        self.assertEqual(self.node.last_status['status'],'path_changed_before_execution')

    def test_local_collision_prevents_follow_despite_free_global_map(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.motion_queue = [self.node.route]
        # Local-only obstacle will be swept by the long front of the body.
        self.node.local_grid.data[10*40+14] = 100
        self.node.send_motion_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status['detail'], 'local_footprint_collision')

    def test_stale_local_map_prevents_follow_and_keeps_memory_destination(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.expected = self.node.memory_goal = (10., 0.)
        self.node.motion_queue = [self.node.route]
        self.node.local_grid_received_at = time.monotonic()-2.
        self.node.send_motion_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'stale_local_costmap')
        self.assertEqual(self.node.memory_goal, (10., 0.))

    def test_follow_failure_recovery_backoff_is_bounded_and_short(self):
        for attempt, expected in enumerate((2., 4., 8., 8.)):
            self.node.recovery_attempts = attempt
            started = time.monotonic()
            self.node.follow_failed('follow_aborted')
            self.assertAlmostEqual(self.node.blocked_until-started, expected, delta=.1)

    def tearDown(self):
        self.node.destroy_node()

    def test_new_costmap_wakes_candidate_even_when_header_is_one_period_old(self):
        # The live global costmap can arrive roughly once per second with a
        # header close to one period old. That is fresh under the 3 s input
        # timeout and must not cause the bridge to wait forever on every map.
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.candidate_index = 0
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.plan_client.send_goal_async.return_value = Future()
        self.node.grid_received_at = time.monotonic() - .95
        self.node.grid_arrived_at = time.monotonic() - .95

        self.node.try_candidate()
        self.assertTrue(self.node.candidate_waiting_for_map)
        self.assertTrue(self.node.pending)

        from copy import deepcopy
        update = deepcopy(self.node.grid)
        stamp = self.node.get_clock().now().nanoseconds - 950_000_000
        update.header.stamp.sec = stamp // 10**9
        update.header.stamp.nanosec = stamp % 10**9
        self.node.on_map(update)

        self.node.plan_client.send_goal_async.assert_called_once()
        self.assertFalse(self.node.candidate_waiting_for_map)
        self.assertTrue(self.node.pending)

    def prepare_blocked_start_retry(self):
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.blocked_endpoint = (10., 0.)
        self.node.blocked_route = [(0., 0.), (10., 0.)]
        self.node.blocked_grid_hash = hash(tuple(self.node.grid.data))
        self.node.blocked_until = time.monotonic() + 20.
        self.node.status('start_footprint_blocked', goal=[10., 0.],
                         navigation_epoch=self.node.epoch)
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.local_freshness_issue = Mock(return_value=None)
        self.node.execution_transport_issue = Mock(return_value=None)
        self.node.pose_blockers = Mock(return_value=[])
        self.node.start_segment = Mock()

    def deliver_fresh_global_map(self):
        from copy import deepcopy
        update = deepcopy(self.node.grid)
        stamp = self.node.get_clock().now().nanoseconds - 100_000_000
        update.header.stamp.sec = stamp // 10**9
        update.header.stamp.nanosec = stamp % 10**9
        self.node.on_map(update)

    def test_fresh_clear_costmap_retries_same_blocked_start_without_backoff(self):
        self.prepare_blocked_start_retry()

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_called_once_with()
        self.assertIsNone(self.node.blocked_endpoint)
        self.assertEqual(self.node.last_status['status'], 'blocked_start_cleared')
        self.assertEqual(self.node.last_status['trigger'], 'fresh_costmap_footprint_clear')

    def test_costmap_update_does_not_retry_while_either_footprint_is_blocked(self):
        self.prepare_blocked_start_retry()
        self.node.pose_blockers.return_value = [(3.38, 1.09)]

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'start_footprint_blocked')
        self.assertEqual(self.node.blocked_endpoint, (10., 0.))

    def test_costmap_update_does_not_retry_with_stale_local_map(self):
        self.prepare_blocked_start_retry()
        self.node.local_freshness_issue.return_value = ('stale_local_costmap', 2.0)

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_not_called()
        self.assertEqual(self.node.blocked_endpoint, (10., 0.))

    def test_costmap_update_does_not_resume_retired_navigation_epoch(self):
        self.prepare_blocked_start_retry()
        self.node.last_status['navigation_epoch'] -= 1

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_not_called()
        self.assertEqual(self.node.blocked_endpoint, (10., 0.))

    def test_remembered_goal_routes_through_graph_and_ignores_other_waypoints(self):
        self.node.on_query(String(data="gas station"))
        self.node.on_memory_goal(pose(15., 0.))
        self.node.on_waypoint(pose(30., 0.))
        self.assertEqual(self.node.waypoint_pub.publish.call_count, 1)
        self.assertEqual(self.node.expected, (15., 0.))
        self.assertEqual(self.node.last_status["source"], "memory_via_graphnav")

        self.node.start_segment = Mock()
        self.node.on_route(route((0., 0.), (30., 0.)))
        self.node.start_segment.assert_not_called()
        self.node.on_route(route((0., 0.), (15., 0.)))
        self.node.start_segment.assert_called_once()

        self.node.on_query(String(data="trash can"))
        self.node.on_waypoint(pose(25., 0.))
        self.assertIsNone(self.node.memory_goal)
        self.assertEqual(self.node.waypoint_pub.publish.call_count, 2)

    def test_same_route_is_not_replanned_after_failure(self):
        path = route((0., 0.), (10., 0.))
        self.node.route = path
        self.node.route_endpoint = (10., 0.)
        self.node.block_current_route()
        self.node.start_segment = Mock()
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()

    def test_exhausted_car_plans_reject_exact_exploration_route(self):
        self.node.follow_enabled = True
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.route_rejection_pub = Mock()
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.candidate_index = len(self.node.candidates)
        self.node.try_candidate()
        self.node.route_rejection_pub.publish.assert_called_once()
        message = self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(message.header.frame_id, self.node.route.header.frame_id)
        self.assertEqual(message.header.stamp.sec, self.node.route.header.stamp.sec)
        self.assertEqual(message.header.stamp.nanosec, self.node.route.header.stamp.nanosec)
        self.assertEqual(message.pose.position.x, 10.)

    def test_local_exploration_rejection_preserves_object_goals_and_freshness(self):
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_rejection_pub = Mock()
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.expected = (10., 0.)
        self.assertFalse(self.node.reject_exploration_route())
        self.node.expected = None
        self.node.input_freshness_issue.return_value = 'stale_odom'
        self.assertFalse(self.node.reject_exploration_route())
        self.node.input_freshness_issue.return_value = None
        self.node.query_started_ns = 10**30
        self.assertFalse(self.node.reject_exploration_route())
        self.node.route_rejection_pub.publish.assert_not_called()

    def test_failed_follow_retries_without_new_graph_path_and_stops_after_three(self):
        self.node.memory_goal = self.node.expected = (15., 0.)
        self.node.route = route((0., 0.), (15., 0.))
        self.node.route_endpoint = (15., 0.)
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.follow_client = Mock()
        self.node.follow_failed('follow_aborted')
        self.node.start_segment = Mock()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()  # backoff still applies
        self.node.blocked_until = time.monotonic()-1
        for _ in range(8):
            self.node.retry_when_planner_starts()
        self.assertEqual(self.node.start_segment.call_count, 3)
        self.assertEqual(self.node.expected, (15., 0.))
        self.assertEqual(self.node.memory_goal, (15., 0.))
        # Map churn and repeated graph publication cannot reset the budget.
        self.node.grid.data[-1] = 100
        self.node.on_route(route((0., 0.), (15., 0.)))
        self.assertEqual(self.node.start_segment.call_count, 3)

    def test_recovery_requires_enabled_execution_fresh_inputs_and_correct_destination(self):
        self.node.expected = self.node.memory_goal = (15., 0.)
        self.node.route = route((0., 0.), (15., 0.))
        self.node.route_endpoint = (15., 0.)
        self.node.plan_client = Mock()
        self.node.follow_client = Mock()
        self.node.follow_failed('follow_aborted')
        self.node.blocked_until = time.monotonic()-1
        self.node.start_segment = Mock()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()
        self.node.follow_enabled = True
        self.node.odom_received_at = time.monotonic()-10
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()
        self.node.odom_received_at = time.monotonic()
        self.node.on_route(route((0., 0.), (40., 0.)))
        self.node.start_segment.assert_not_called()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.route_endpoint, (15., 0.))

    def test_follow_failure_preserves_memory_and_rechecks_a_changed_map(self):
        self.node.on_query(String(data='gas station'))
        self.node.on_memory_goal(pose(15., 0.))
        view = {'id':'1', 'frame':'spot1/odom', 'position':[15.,0.,0.], 'heading':0.}
        self.node.memory_views = [view]
        self.node.memory_active = view
        path = route((0., 0.), (15., 0.))
        self.node.route = path
        self.node.route_endpoint = (15., 0.)
        self.node.following = True
        epoch = self.node.epoch

        self.node.follow_failed('follow_aborted', action_status=6)

        self.assertEqual(self.node.memory_goal, (15., 0.))
        self.assertEqual(self.node.memory_active, view)
        self.assertEqual(self.node.memory_rejected, set())
        self.assertEqual(self.node.last_status['source'], 'memory_via_graphnav')
        self.assertEqual(self.node.epoch, epoch+1)
        self.node.memory_feedback_pub.publish.assert_not_called()
        self.node.memory_selection_pub.publish.assert_not_called()
        self.node.start_segment = Mock()
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()
        self.node.blocked_until = time.monotonic()-1
        self.node.grid.data[-1] = 100
        self.node.on_route(route((0., 0.), (15., 0.)))
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.expected, (15., 0.))

    def test_tracking_deviation_cancels_current_section_and_keeps_memory(self):
        self.node.memory_goal = self.node.expected = (15., 0.)
        self.node.route = route((0., 0.), (15., 0.))
        self.node.route_endpoint = (15., 0.)
        self.node.motion_queue = [route((0., 0.), (6., 0.))]
        self.node.following = True
        handle = self.node.follow_handle = Mock()
        self.node.odom.pose.pose.position.y = 1.2
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once_with()
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.motion_queue, [])
        self.assertEqual(self.node.last_status['reason'], 'tracking_deviation')
        self.assertEqual(self.node.memory_goal, (15., 0.))
        self.node.memory_feedback_pub.publish.assert_not_called()

    def test_new_query_cancels_in_flight_planning_goal(self):
        planner_goal = Mock()
        self.node.plan_handle = planner_goal
        self.node.pending = True

        self.node.on_query(String(data="tree"))

        planner_goal.cancel_goal_async.assert_called_once_with()
        self.assertIsNone(self.node.plan_handle)
        self.assertFalse(self.node.pending)

    def test_late_planner_acceptance_after_reset_is_cancelled(self):
        self.node.reset()
        stale_epoch = self.node.epoch - 1
        planner_goal = Mock()
        planner_goal.accepted = True
        future = Mock()
        future.result.return_value = planner_goal

        self.node.on_plan_accepted(future, stale_epoch, 0)

        planner_goal.cancel_goal_async.assert_called_once_with()
        planner_goal.get_result_async.assert_not_called()

    def test_late_planner_acceptance_for_old_candidate_is_cancelled(self):
        self.node.epoch = 2
        self.node.candidate_index = 1
        planner_goal = Mock()
        planner_goal.accepted = True
        future = Mock()
        future.result.return_value = planner_goal

        self.node.on_plan_accepted(future, self.node.epoch, 0)

        planner_goal.cancel_goal_async.assert_called_once_with()
        planner_goal.get_result_async.assert_not_called()

    def test_stalled_compute_path_adopts_a_newer_graph_route(self):
        old_route = route((0., 0.), (10., 0.))
        newest_route = route((0., 0.), (20., 0.))
        self.node.route = old_route
        self.node.latest_route = newest_route
        self.node.route_endpoint = (10., 0.)
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.pending = True
        self.node.plan_handle = Mock()
        old_plan_handle = self.node.plan_handle
        self.node.plan_request_started_at = time.monotonic() - self.node.compute_path_timeout - .1
        self.node.start_segment = Mock()
        old_epoch = self.node.epoch

        self.node.monitor_inputs()

        old_plan_handle.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.node.epoch, old_epoch)
        self.assertIs(self.node.route, newest_route)
        self.assertEqual(self.node.route_endpoint, (20., 0.))
        self.assertEqual(self.node.last_status['status'], 'planner_timeout_using_latest_route')
        self.node.start_segment.assert_called_once_with()

    def test_stalled_compute_path_advances_candidate_when_route_is_unchanged(self):
        path = route((0., 0.), (10., 0.))
        self.node.route = self.node.latest_route = path
        self.node.route_endpoint = (10., 0.)
        self.node.candidates = [(6., 0., 0., 6.), (8., 0., 0., 8.)]
        self.node.candidate_index = 0
        self.node.pending = True
        self.node.plan_handle = Mock()
        old_plan_handle = self.node.plan_handle
        self.node.plan_request_started_at = time.monotonic() - self.node.compute_path_timeout - .1
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.plan_client.send_goal_async.return_value = Future()
        old_epoch = self.node.epoch

        self.node.monitor_inputs()

        old_plan_handle.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.node.epoch, old_epoch)
        self.assertEqual(self.node.candidate_index, 1)
        self.node.plan_client.send_goal_async.assert_called_once()
        self.assertTrue(self.node.pending)
        self.assertIsNotNone(self.node.plan_request_started_at)

    def test_stale_costmap_prevents_starting_a_plan(self):
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.grid_received_at = time.monotonic() - self.node.costmap_timeout - 0.1
        self.node.plan_client = Mock()

        self.node.start_segment()

        self.assertEqual(self.node.last_status["status"], "stale_costmap")
        self.node.plan_client.wait_for_server.assert_not_called()

    def test_stale_costmap_discards_a_completed_planner_result(self):
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.pending = True
        self.node.grid_received_at = time.monotonic() - self.node.costmap_timeout - 0.1
        future = Mock()

        self.node.on_plan_result(future, self.node.epoch, 0)

        self.assertEqual(self.node.last_status["status"], "stale_costmap")
        self.assertFalse(self.node.pending)
        future.result.assert_not_called()
        self.node.path_pub.publish.assert_not_called()

    def test_memory_goal_waits_for_old_graph_goal_to_be_cleared(self):
        future = Future()
        self.node.graph_reset_client.call_async.side_effect = None
        self.node.graph_reset_client.call_async.return_value = future
        self.node.on_query(String(data="gas station"))
        self.node.on_memory_goal(pose(20., 0.))
        self.node.waypoint_pub.publish.assert_not_called()
        self.node.start_segment = Mock()
        self.node.on_route(route((0., 0.), (20., 0.)))
        self.node.start_segment.assert_not_called()
        future.set_result(Mock(success=True))
        self.node.waypoint_pub.publish.assert_called_once()
        self.assertFalse(self.node.query_reset_pending)

    def test_memory_approach_extends_past_graph_goal_radius(self):
        self.node.on_memory_goal(pose(20., 0.))
        self.node.route = route((0., 0.), (14., 0.))
        self.assertEqual(self.node.current_route_points()[-1], (20., 0.))

    def test_coarse_graph_corner_allows_an_approach_heading(self):
        self.node.route = route((0., 0.), (6., 0.), (6., 6.))
        self.node.route_endpoint = (6., 6.)
        self.node.memory_goal = self.node.expected = (6., 6.)
        self.node.memory_heading = 1.2
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.try_candidate = Mock()

        self.node.start_segment()

        at_corner = [c for c in self.node.candidates if c[:2] == (6., 2.)]
        self.assertEqual(len(at_corner), 2)
        self.assertAlmostEqual(at_corner[0][2], 1.5707963267948966)
        self.assertAlmostEqual(at_corner[1][2], 0.3217505543966422)
        terminal = [c for c in self.node.candidates if c[:2] == (6., 6.)]
        self.assertTrue(any(abs(c[2] - 1.2) < 1e-6 for c in terminal))
        self.assertGreater(len(terminal), 1)

    def memory_views(self, *positions):
        return [{'id': str(i), 'position': [float(x), 0., 0.], 'heading': 0.,
                 'frame': 'spot1/odom'} for i, x in enumerate(positions)]

    def send_views(self, views):
        self.node.on_query(String(data='tree'))
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.on_memory_candidates(String(data=json.dumps({'query':'tree','views':views})))

    def valid_plan(self, endpoint):
        result = Mock()
        result.status = 4
        result.result.path = drivable_route((0., 0.), (endpoint, 0.))
        future = Mock()
        future.result.return_value = result
        self.node.on_plan_result(future, self.node.epoch, self.node.candidate_index)

    def test_memory_selection_compares_feasible_route_costs(self):
        self.send_views(self.memory_views(8., 4.))
        self.node.on_route(route((0.,0.),(8.,0.)))
        self.valid_plan(8.)
        self.assertTrue(self.node.memory_selecting)
        self.node.on_route(route((0.,0.),(4.,0.)))
        self.valid_plan(4.)
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.memory_active['id'], '1')
        self.assertEqual(self.node.memory_goal, (4.,0.))
        self.node.path_pub.publish.assert_called_once()
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses[-1].pose.position.x, 4.)

    def test_blocked_memory_view_switches_to_another_observation(self):
        self.node.grid.data[10*40+22] = 100  # remembered target x=12
        self.send_views(self.memory_views(12., 6.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.valid_plan(12.)
        self.assertEqual(self.node.memory_active['id'], '1')
        self.node.on_route(route((0.,0.),(6.,0.)))
        self.valid_plan(6.)
        self.assertEqual(self.node.memory_goal, (6.,0.))
        self.assertEqual(self.node.memory_rejected, {'0'})

    def test_local_memory_failure_retains_old_graph_goal(self):
        self.send_views(self.memory_views(12.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.node.candidate_failed('empty_path')
        self.assertEqual(self.node.memory_goal, (12., 0.))
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.graph_reset_client.call_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'memory_locally_blocked')
        feedback = json.loads(self.node.memory_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['outcome'], 'unavailable')

    def test_short_prefix_does_not_prove_memory_endpoint_reachable(self):
        self.send_views(self.memory_views(12.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.assertTrue(all(c[:2] == (12.,0.) for c in self.node.candidates))

    def test_blocked_current_pose_does_not_discard_the_memory_destination(self):
        self.send_views(self.memory_views(12.))
        self.node.route = route((0.,0.),(12.,0.))
        self.node.grid.data[10*40+12] = 100  # inside the current vehicle body
        self.node.start_segment()
        self.assertEqual(self.node.last_status['status'], 'start_footprint_blocked')
        self.node.memory_deadline = time.monotonic()-1.
        self.node.monitor_memory_selection()
        self.assertTrue(self.node.memory_selecting)
        self.assertEqual(self.node.memory_goal,(12.,0.))
        self.node.memory_feedback_pub.publish.assert_not_called()
        self.assertEqual(self.node.graph_reset_client.call_async.call_count,1)

    def test_memory_planning_resumes_when_current_pose_becomes_free(self):
        self.send_views(self.memory_views(12.))
        self.node.status('start_footprint_blocked')
        self.node.start_segment = Mock()
        self.node.monitor_memory_selection()
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.memory_goal,(12.,0.))

    def test_memory_uses_alternative_arrival_heading(self):
        views=self.memory_views(8.)
        views[0]['heading']=3.141592653589793
        self.send_views(views)
        self.node.on_route(route((0.,0.),(8.,0.)))
        self.node.candidate_failed('empty_path')
        self.valid_plan(8.)
        self.assertFalse(self.node.memory_selecting)
        self.assertAlmostEqual(self.node.memory_arrival_heading,0.)

    def test_memory_candidates_can_arrive_before_the_query(self):
        views=self.memory_views(8.)
        self.node.on_memory_candidates(String(data=json.dumps({'query':'tree','views':views})))
        self.node.on_query(String(data='tree'))
        self.assertEqual(self.node.memory_active['id'],'0')

    def test_candidate_set_ignores_legacy_single_pose_replay(self):
        self.send_views(self.memory_views(8.))
        self.node.on_memory_goal(pose(20.,0.))
        self.assertEqual(self.node.memory_goal,(8.,0.))

    def test_new_query_cancels_memory_selection(self):
        self.send_views(self.memory_views(8.))
        self.node.on_query(String(data='trash can'))
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.memory_views,[])

    def test_latest_graph_route_is_used_after_a_segment(self):
        self.node.following = True
        updated = route((0., 0.), (20., 0.))
        self.node.on_route(updated)
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.odom.pose.pose.position.x = 6.
        self.node.on_follow_result(future, self.node.epoch, (6., 0.))
        self.assertIs(self.node.route, updated)
        self.node.start_segment.assert_called_once()

    def prepare_reverse_forward_execution(self, dispatch=True):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        def accept(_goal, **_kwargs):
            future = Future()
            handle = Mock(accepted=True)
            handle.get_result_async.return_value = Future()
            future.set_result(handle)
            return future
        self.node.follow_client.send_goal_async.side_effect = accept
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.motion_queue = [route((0., 0.), (-2., 0.)),
                                  route((-2., 0.), (0., 0.), (6., 0.))]
        if dispatch:
            self.node.send_motion_segment()

    def test_follow_goal_ends_at_cusp_using_strict_goal_checker(self):
        self.prepare_reverse_forward_execution()
        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.goal_checker_id, 'cusp_goal_checker')
        self.assertEqual(goal.path.poses[-1].pose.position.x, -2.)
        self.assertEqual(len(goal.path.poses), 2)

    def test_short_maneuver_endpoint_inside_goal_disk_is_rejected(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.motion_queue = [route((0., 0.), (.15, 0.)),
                                  route((.15, 0.), (6., 0.))]
        self.node.candidate_failed = Mock()

        self.node.send_motion_segment()

        self.node.candidate_failed.assert_called_once_with(
            'section_goal_within_controller_tolerance')
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_explicit_target_endpoint_uses_strict_goal_checker(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.follow_client.send_goal_async.side_effect = lambda _goal, **_kwargs: Future()
        self.node.route = route((0., 0.), (8., 0.))
        self.node.route_endpoint = self.node.expected = (8., 0.)
        self.node.motion_queue = [drivable_route((0.,0.),(8.,0.))]

        self.node.send_motion_segment()

        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.goal_checker_id, 'maneuver_goal_checker')

    def test_real_short_cusp_outside_junction_disk_is_not_discarded(self):
        self.prepare_reverse_forward_execution(dispatch=False)
        self.node.following=False
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.motion_queue=[route((0.,0.),(-.35,0.)),route((-.35,0.),(6.,0.))]
        self.node.send_motion_segment()
        goal=self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.goal_checker_id,'cusp_goal_checker')
        self.assertEqual(goal.path.poses[-1].pose.position.x,-.35)

    def test_ideal_diagnostic_cusp_keeps_its_original_checker(self):
        self.prepare_reverse_forward_execution(dispatch=False)
        self.node.carrier_model='ideal_point'
        self.node.following=False
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.send_motion_segment()
        goal=self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.goal_checker_id,'maneuver_goal_checker')

    def test_near_cusp_action_success_uses_a_feasible_join_without_replanning(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -1.85
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 2)
        self.assertEqual(self.node.last_status['status'], 'following')
        self.node.start_segment.assert_not_called()
        self.assertEqual(len(self.node.motion_queue), 1)

    def test_reached_cusp_dispatches_only_next_checked_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -2.
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 2)
        self.assertEqual(goal.path.poses[0].pose.position.x, -2.)
        self.assertEqual(goal.goal_checker_id, 'goal_checker')
        self.assertTrue(self.node.following)

    def test_old_observation_disk_cannot_complete_an_ackermann_cusp(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -1.46  # 0.54 m from the planned cusp
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.last_status['status'], 'follow_failed')
        self.assertEqual(self.node.last_status['reason'], 'cusp_not_reached')
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.node.start_segment.assert_not_called()
        self.assertEqual(len(self.node.motion_queue), 0)

    def test_costmap_change_at_cusp_replans_instead_of_replaying_next_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -2.
        self.node.grid.data[10 * 40 + 14] = 100
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'replanning_after_maneuver')
        self.assertEqual(self.node.last_status['reason'], 'collision_cell')
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.motion_queue, [])

    def test_offset_cusp_can_join_clear_straight_branch_without_replanning(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.position.x = -1.8
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.last_status['status'], 'following')
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 2)
        self.node.start_segment.assert_not_called()

    def test_query_change_discards_unexecuted_motion_sections(self):
        self.prepare_reverse_forward_execution()
        self.node.on_query(String(data='gas station'))
        self.assertEqual(self.node.motion_queue, [])
        self.assertFalse(self.node.following)

    def test_completed_plan_is_split_before_first_follow_goal(self):
        self.prepare_reverse_forward_execution(dispatch=False)
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.following = False
        self.node.candidates = [(6., 0., 0., 6.)]
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (-2., 0.), (0., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.assertEqual(len(self.node.motion_queue), 2)
        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.path.poses[-1].pose.position.x, -2.)
        self.assertEqual(goal.goal_checker_id, 'cusp_goal_checker')

    def test_quantized_short_cusp_is_rejected_before_follow_action(self):
        self.prepare_reverse_forward_execution(dispatch=False)
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.following = False
        self.node.odom.pose.pose.position.x = -.126
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.candidate_failed = Mock()
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (-.3, 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.node.candidate_failed.assert_called_once_with('maneuver_already_within_tolerance')
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_two_headings_choose_shorter_path_without_reversals(self):
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.candidates = [(6., 0., 1., 6.), (6., 0., 0., 6.), (4., 0., 0., 4.)]
        self.node.try_candidate = Mock()
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (-2., 0.), (0., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.node.path_pub.publish.assert_not_called()
        self.node.try_candidate.assert_called_once()
        future.result.return_value.result.path = route((0., 0.), (3., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 1)
        selected = self.node.path_pub.publish.call_args.args[0]
        self.assertEqual([p.pose.position.x for p in selected.poses], [0., 3., 6.])
        self.assertEqual(self.node.try_candidate.call_count, 1)

    def test_alternative_failure_rechecks_first_path_against_updated_map(self):
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.candidates = [(6., 0., 1., 6.), (6., 0., 0., 6.)]
        self.node.try_candidate = Mock()
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (3., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.node.grid.data[10*40+13] = 100
        self.node.candidate_failed('empty_path')
        self.node.path_pub.publish.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'path_changed_before_execution')

    def test_late_follow_acceptance_after_query_is_cancelled(self):
        self.prepare_reverse_forward_execution()
        epoch = self.node.epoch
        self.node.on_query(String(data='gas station'))
        handle = Mock(accepted=True)
        future = Mock()
        future.result.return_value = handle
        self.node.on_follow_accepted(future, epoch, (-2., 0.))
        handle.cancel_goal_async.assert_called_once_with()
        handle.get_result_async.assert_not_called()
        self.assertEqual(self.node.motion_queue, [])

    def test_old_follow_result_cannot_resume_after_query_change(self):
        self.prepare_reverse_forward_execution()
        epoch = self.node.epoch
        self.node.on_query(String(data='gas station'))
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.query, 'gas station')
        self.assertEqual(self.node.motion_queue, [])

    def test_stale_odom_at_cusp_prevents_next_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom_received_at = time.monotonic() - self.node.odom_timeout - .1
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'stale_odom')
        self.assertEqual(self.node.motion_queue, [])

    def test_disable_execution_discards_queued_sections(self):
        self.prepare_reverse_forward_execution()
        result = self.node.on_parameters([SimpleNamespace(name='follow_path', value=False)])
        self.assertTrue(result.successful)
        self.assertFalse(self.node.following)
        self.assertFalse(self.node.follow_enabled)
        self.assertEqual(self.node.motion_queue, [])

    def test_missing_maneuver_checker_rejection_stops_execution(self):
        self.prepare_reverse_forward_execution()
        future = Mock()
        future.result.return_value.accepted = False
        self.node.on_follow_accepted(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.last_status['reason'], 'controller_rejected')
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.motion_queue, [])

    def test_stale_input_cancels_follow_and_fresh_data_allows_retry(self):
        handle = Mock()
        self.node.follow_handle = handle
        self.node.following = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.odom_received_at = time.monotonic() - self.node.odom_timeout - .1
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.node.following)
        self.node.odom_received_at = self.node.grid_received_at = time.monotonic()
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.start_segment = Mock()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once()

    def test_old_graph_path_timestamp_is_rejected(self):
        self.node.on_memory_goal(pose(20., 0.))
        old = route((0., 0.), (20., 0.))
        old.header.stamp.sec = old.header.stamp.nanosec = 0
        self.node.start_segment = Mock()
        self.node.on_route(old)
        self.node.start_segment.assert_not_called()

    def test_new_query_can_resume_frontier_route_without_object_waypoint(self):
        self.node.on_query(String(data="trash can"))
        self.node.start_segment = Mock()
        self.node.on_route(route((0., 0.), (10., 0.)))
        self.node.start_segment.assert_not_called()
        self.node.query_grace_until = 0.0
        self.node.on_route(route((0., 0.), (10., 0.)))
        self.node.start_segment.assert_called_once()

    def test_current_remembered_viewpoint_does_not_request_zero_length_path(self):
        self.node.on_memory_goal(pose(0.5, 0.))
        self.assertEqual(self.node.last_status["status"], "already_at_goal")
        self.assertTrue(self.node.last_status['requires_visual_confirmation'])
        self.node.waypoint_pub.publish.assert_not_called()

    def test_graph_reset_keeps_memory_arrival_visual_confirmation_required(self):
        self.send_views(self.memory_views(.5))
        self.assertEqual(self.node.last_status['status'], 'already_at_goal')
        self.assertTrue(self.node.last_status['requires_visual_confirmation'])

    def route_status(self, path, *, goal=(40., 0.), complete=False):
        stamp = path.header.stamp.sec * 10**9 + path.header.stamp.nanosec
        return String(data=json.dumps(dict(stamp_ns=stamp, goal=list(goal), complete=complete,
                                           mode='known' if complete else 'frontier')))

    def test_partial_route_waits_for_correlated_metadata_then_starts(self):
        self.node.require_route_status = True
        self.node.expected = (40., 0.)
        self.node.start_segment = Mock()
        path = route((0., 0.), (8., 0.))
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()
        self.node.on_route_status(self.route_status(path))
        self.node.start_segment.assert_called_once()
        self.assertFalse(self.node.route_complete)
        self.assertNotIn(self.node.expected, self.node.current_route_points())

    def test_metadata_for_old_goal_cannot_accept_a_new_route(self):
        self.node.require_route_status = True
        self.node.expected = (40., 0.)
        self.node.start_segment = Mock()
        path = route((0., 0.), (8., 0.))
        self.node.on_route_status(self.route_status(path, goal=(15., 0.)))
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()

    def test_partial_nearby_route_does_not_force_full_memory_target(self):
        self.send_views(self.memory_views(15.))
        path = route((0., 0.), (8., 0.))
        self.node.require_route_status = True
        self.node.on_route_status(self.route_status(path, goal=(15., 0.)))
        self.node.on_route(path)
        self.assertFalse(self.node.memory_full_check)
        self.assertTrue(self.node.candidates)
        self.assertTrue(all(c[0] <= 8. for c in self.node.candidates))

    def test_latest_partial_metadata_does_not_change_active_segment(self):
        self.node.require_route_status = True
        self.node.expected = (40., 0.)
        self.node.start_segment = Mock()
        active = route((0., 0.), (40., 0.))
        self.node.on_route_status(self.route_status(active, complete=True))
        self.node.on_route(active)
        self.node.following = True
        newer = route((0., 0.), (8., 0.))
        self.node.on_route_status(self.route_status(newer))
        self.node.on_route(newer)
        self.assertTrue(self.node.route_complete)
        self.assertFalse(self.node.latest_route_complete)
        self.assertIs(self.node.route, active)

    def test_selected_memory_prefix_goes_directly_to_follow_path(self):
        self.send_views(self.memory_views(40.))
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.on_route(route((0., 0.), (8., 0.), (40., 0.)))
        # A valid prefix is the first local option for this long graph route.
        target = self.node.candidates[self.node.candidate_index][0]
        self.valid_plan(target)
        self.assertFalse(self.node.memory_selecting)
        self.node.follow_client.send_goal_async.assert_called_once()
        sent = self.node.follow_client.send_goal_async.call_args.args[0].path
        self.assertAlmostEqual(sent.poses[-1].pose.position.x, target)
        self.assertEqual(self.node.memory_goal, (40., 0.))

    def test_selected_memory_path_is_rejected_after_map_changes(self):
        self.send_views(self.memory_views(12.))
        path = route((0., 0.), (12., 0.))
        self.node.route = path
        self.node.memory_matches = [dict(view=self.node.memory_active, cost=12., heading=0.,
            complete=True, path=path, route=path, target=(12., 0.), route_complete=True)]
        self.node.grid.data[10*40+18] = 100
        self.node.finish_memory_selection()
        self.node.path_pub.publish.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'path_changed_before_execution')
        self.assertEqual(self.node.memory_goal, (12., 0.))

    def confirmation(self, **changes):
        data = dict(query='gas station', frame='spot1/odom',
            observation_stamp_ns=self.node.get_clock().now().nanoseconds,
            object_position=[20., 0., 0.], observer_position=[12., 0., 0.],
            observer_orientation=[0., 0., 0., 1.], radius=.5, inliers=4, observations=5,
            search_confirmed=True, standoff=[15., 0.], localization_valid=True,
            quality={'confidence': .9, 'association_validated': True})
        data.update(changes)
        return String(data=json.dumps(data))

    def test_stale_clamped_approach_update_moves_goal_but_never_marks_target_found(self):
        self.node.on_query(String(data='green residential house'))
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        initial = json.loads(self.confirmation(
            query='green residential house', object_position=[20., 0., 0.],
            observer_position=[0., 0., 0.], standoff=[10., 0.],
            standoff_clamped=True, standoff_object_distance_m=10.).data)
        self.node.on_object_confirmation(String(data=json.dumps(initial)))
        self.assertEqual(self.node.expected, (10., 0.))
        self.assertEqual(self.node.active_object_confirmation['observation_stamp_ns'],
                         initial['observation_stamp_ns'])
        self.node.route = route((0., 0.), (10., 0.))

        now_ns = self.node.get_clock().now().nanoseconds + 10_000_000_000
        now_msg = self.node.get_clock().now().to_msg()
        now_msg.sec, now_msg.nanosec = divmod(now_ns, 10**9)
        fake_clock = SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=now_ns, to_msg=lambda: now_msg))
        refresh = dict(initial,
            approach_refresh=True,
            base_observation_stamp_ns=initial['observation_stamp_ns'],
            approach_update_stamp_ns=now_ns,
            standoff=[12., 0.],
            standoff_object_distance_m=8.)
        with patch.object(self.node, 'get_clock', return_value=fake_clock):
            self.node.on_object_confirmation(String(data=json.dumps(refresh)))

            self.assertFalse(self.node.object_found)
            self.assertEqual(self.node.expected, (12., 0.))
            self.assertEqual(self.node.last_status['status'], 'object_standoff_approach')

            wrong_anchor = dict(refresh, base_observation_stamp_ns=
                                initial['observation_stamp_ns'] - 1,
                                standoff=[14., 0.])
            self.node.on_object_confirmation(String(data=json.dumps(wrong_anchor)))
            self.assertEqual(self.node.expected, (12., 0.))

            self.node.odom.pose.pose.position.x = 14.5
            near_target = dict(refresh, standoff=[14.5, 0.],
                               standoff_object_distance_m=5.5)
            self.node.on_object_confirmation(String(data=json.dumps(near_target)))
            self.assertFalse(self.node.object_found)
            self.assertEqual(self.node.last_status['status'],
                             'approach_refresh_waiting_for_fresh_confirmation')

    def test_memory_object_is_found_from_new_observer_position(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_goal = self.node.expected = (0., 0.)
        self.node.memory_active = dict(object_position=[20.3, 0., 0.], object_radius=.6)
        self.node.odom.pose.pose.position.x = 14.5
        self.node.on_object_confirmation(self.confirmation(object_position=[18., 0., 0.]))
        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')
        self.assertAlmostEqual(self.node.last_status['standoff_distance_m'], 0.5)
        self.assertEqual(self.node.last_status['standoff_tolerance_m'], 0.6)

    def test_nearest_signal_hint_rejects_farther_candidate_but_allows_closer_one(self):
        self.node.on_query(String(data='traffic light'))
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()

        # The initial standoff waypoint is four metres from the nearest known
        # signal and faces it. It must become the query's distance baseline.
        hint = pose(64., 0.)
        hint.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_waypoint(hint)
        self.assertEqual(self.node.expected, (64., 0.))
        self.assertAlmostEqual(self.node.nearest_signal_distance, 68.)
        self.assertAlmostEqual(self.node.nearest_signal_position[0], 68.)
        self.assertAlmostEqual(self.node.nearest_signal_position[1], 0.)

        # A visually supported but farther signal cannot steal that route.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[85., 0., 6.],
            observer_position=[0., 0., 0.], standoff=[81., 0.]))
        self.assertAlmostEqual(self.node.nearest_signal_position[0], 68.)
        self.assertEqual(self.node.expected, (64., 0.))
        self.node.reset.assert_called_once()

        # Even if the vehicle is physically beside that farther light, it
        # cannot finish a query whose selected nearest instance is elsewhere.
        self.node.odom.pose.pose.position.x = 85.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[85., 0., 6.],
            observer_position=[85., 0., 0.], standoff=[85., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.expected, (64., 0.))

        # A confirmed instance with a meaningful distance advantage can replace it.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[64.8, 0., 6.],
            observer_position=[0., 0., 0.], standoff=[60.8, 0.]))
        self.assertAlmostEqual(self.node.nearest_signal_position[0], 64.8)
        self.assertEqual(self.node.expected, (60.8, 0.))
        self.assertEqual(self.node.reset.call_count, 2)

    def test_visible_target_status_does_not_mask_bounded_stall_retry(self):
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = self.node.expected = (10., 0.)
        self.node.blocked_endpoint = (10., 0.)
        self.node.blocked_until = time.monotonic()-1.
        self.node.last_status = dict(status='confirmed_standoff_update_deferred')
        self.node.start_segment = Mock()

        self.node.retry_when_planner_starts()

        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.recovery_attempts, 1)
        self.node.start_segment = Mock()
        self.node.on_route(route((12., 0.), (0., 0.)))
        self.node.start_segment.assert_not_called()

    def test_confirmation_before_standoff_keeps_frontier_route_active(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.expected = (10., 0.)
        self.node.query_grace_until = 0.
        self.node.start_segment = Mock()

        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[20., 0., 0.], standoff=[10., 0.]))

        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_before_standoff')
        self.node.on_route(route((0., 0.), (10., 0.)))
        self.node.start_segment.assert_called_once()

    def test_new_confirmed_standoff_replaces_active_live_target_route(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.expected = (15., 0.)
        self.node.following = True
        self.node.query_reset_pending = False
        self.node.odom.pose.pose.position.x = 12.
        self.node.odom_received_at = time.monotonic()
        self.node.reset = Mock()
        self.node.relay_destination = Mock()

        updated = self.confirmation(query='crosswalk', standoff=[17., 0.])
        self.node.on_object_confirmation(updated)

        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_standoff_approach')
        self.assertEqual(self.node.expected, (17., 0.))
        self.node.reset.assert_called_once()
        self.node.relay_destination.assert_called_once()
        goal = self.node.relay_destination.call_args.args[0]
        self.assertEqual((goal.pose.position.x, goal.pose.position.y), (17., 0.))

        # Repeated confirmations of the unchanged close standoff report the
        # distance without cancelling and resending the same route.
        self.node.on_object_confirmation(updated)
        self.node.reset.assert_called_once()
        self.node.relay_destination.assert_called_once()

    def test_confirmed_route_ignores_frame_by_frame_goal_jitter(self):
        self.node.on_query(String(data='house'))
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation(query='house', standoff=[10., 0.]))
        self.node.following = True
        for goal in ([11., 0.], [11., 1.], [12., 0.], [9., -1.]):
            self.node.on_object_confirmation(self.confirmation(query='house', standoff=goal))
        self.assertEqual(self.node.expected, (10., 0.))
        self.node.reset.assert_called_once()
        self.node.relay_destination.assert_called_once()
        self.assertEqual(self.node.last_status['status'], 'confirmed_standoff_update_deferred')
        self.node.following = False
        self.node.pending = True
        self.node.on_object_confirmation(self.confirmation(query='house', standoff=[12., 0.]))
        self.node.reset.assert_called_once()

    def test_confirmed_segment_can_advance_after_reaching_latched_goal(self):
        self.node.on_query(String(data='house'))
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation(query='house', standoff=[10., 0.]))
        self.node.route = route((0., 0.), (10., 0.))
        self.node.odom.pose.pose.position.x = 9.5
        self.node.on_object_confirmation(self.confirmation(query='house', standoff=[12., 0.]))
        self.assertEqual(self.node.expected, (12., 0.))
        self.assertEqual(self.node.relay_destination.call_count, 2)
        self.assertFalse(self.node.object_found)

    def test_memory_standoff_refinement_keeps_accepted_follow_path(self):
        self.node.on_query(String(data='gas station'))
        self.node.query_reset_pending = False
        self.node.memory_active = dict(id='passive:station:1',
            evidence='passive_semantic_landmarks', object_position=[20., 0., 0.], object_radius=.5)
        self.node.expected = self.node.memory_goal = (8., 0.)
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation())
        self.node.following = True
        self.node.on_object_confirmation(self.confirmation(standoff=[16., 1.]))
        self.assertEqual(self.node.expected, (15., 0.))
        self.node.reset.assert_called_once()
        self.node.relay_destination.assert_called_once()
        self.assertFalse(self.node.object_found)

    def test_deferred_map_boundary_goal_is_consumed_after_current_route(self):
        self.node.on_query(String(data='house'))
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation(query='house', standoff=[10., 0.]))
        self.node.following = True
        self.node.on_object_confirmation(self.confirmation(query='house', standoff=[13., 0.]))
        self.assertEqual(self.node.expected, (10., 0.))
        self.node.following = False
        self.node.route = route((0., 0.), (10., 0.))
        self.node.odom.pose.pose.position.x = 9.5
        self.node.await_object_observation()
        self.assertEqual(self.node.expected, (13., 0.))
        self.assertEqual(self.node.relay_destination.call_count, 2)
        self.assertIsNone(self.node.observation_wait_until)
        self.assertFalse(self.node.object_found)
        self.node.on_query(String(data='gas station'))
        self.assertIsNone(self.node.deferred_confirmed_approach)

    def test_memory_comparison_does_not_accept_confirmation_for_transient_view(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_selecting = True
        self.node.memory_active = dict(object_position=[20., 0., 0.], object_radius=.5)
        self.node.expected = (8., 0.)
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation())
        self.assertEqual(self.node.expected, (8., 0.))
        self.assertNotIn('confirmation_anchor', self.node.memory_active)
        self.node.relay_destination.assert_not_called()
        self.assertFalse(self.node.object_found)

    def test_found_query_is_terminal_until_next_query(self):
        self.node.on_query(String(data='house'))
        self.node.odom.pose.pose.position.x = 16.
        self.node.on_object_confirmation(self.confirmation(
            query='house', standoff=[16., 0.]))
        self.assertTrue(self.node.object_found)
        accepted = dict(self.node.last_status)
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation(
            query='house', object_position=[100., 0., 0.], standoff=[95., 0.]))
        self.assertEqual(self.node.last_status, accepted)
        self.node.relay_destination.assert_not_called()
        self.node.on_query(String(data='gas station'))
        self.assertFalse(self.node.object_found)
        self.assertIsNone(self.node.confirmed_route_goal)

    def test_confirmed_endpoint_does_not_stop_at_looser_one_metre_threshold(self):
        self.node.query_reset_pending = False
        self.node.expected = self.node.confirmed_route_goal = (.8, 0.)
        self.node.confirmed_route_object = (4., 0.)
        self.node.route = route((0., 0.), (.8, 0.))
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = False
        self.node.start_segment()
        self.assertEqual(self.node.last_status['status'], 'planner_unavailable')
        self.assertIsNone(self.node.observation_wait_until)

    def test_memory_confirmation_redirects_observer_pose_to_target_standoff(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(
            id='passive:gas station:17',
            evidence='passive_semantic_landmarks',
            object_position=[20., 0., 0.], object_radius=.5)
        self.node.memory_goal = self.node.expected = (8., 0.)
        self.node.query_reset_pending = False
        self.node.relay_destination = Mock()

        self.node.on_object_confirmation(self.confirmation(
            object_position=[20.2, 0., 0.], observer_position=[12., 0., 0.],
            standoff=[15., 0.]))

        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'memory_standoff_approach')
        self.assertEqual(self.node.memory_goal, (8., 0.))
        self.assertEqual(self.node.expected, (15., 0.))
        self.node.relay_destination.assert_called_once()
        goal = self.node.relay_destination.call_args.args[0]
        self.assertEqual((goal.pose.position.x, goal.pose.position.y), (15., 0.))
        self.assertAlmostEqual(goal.pose.orientation.z, 0.)
        self.assertAlmostEqual(goal.pose.orientation.w, 1.)

    def test_passive_memory_latches_first_fresh_match_for_nearby_facade_patches(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(
            id='passive:gas station:17',
            evidence='passive_semantic_landmarks',
            object_position=[10., 0., 0.], object_radius=.5)
        self.node.memory_goal = self.node.expected = (8., 0.)
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()

        # First live confirmation falls within the coarse passive memory
        # region and becomes the tighter association anchor for this query.
        first = self.confirmation(
            object_position=[17., 0., 0.], standoff=[15., 0.])
        self.node.on_object_confirmation(first)
        anchor = self.node.memory_active['confirmation_anchor']
        self.assertEqual(anchor['object_position'], [17., 0.])
        self.assertEqual(self.node.last_status['status'], 'memory_standoff_approach')

        # A new facade patch on the same large object can shift the localized
        # point modestly while remaining associated with that live target.
        second = self.confirmation(
            object_position=[18.8, 0., 0.], standoff=[20., 0.])
        self.node.on_object_confirmation(second)
        self.assertEqual(self.node.last_status['status'], 'confirmed_standoff_update_deferred')
        self.assertEqual(anchor['object_position'], [17., 0.])

        # The latch remains local: a later observation well away from the
        # matched instance is still rejected as a different gas station.
        self.node.on_object_confirmation(self.confirmation(
            object_position=[22., 0., 0.], standoff=[24., 0.]))
        self.assertEqual(self.node.last_status['status'], 'confirmation_outside_memory_region')
        self.assertEqual(self.node.last_status['memory_region'], [17., 0.])

    def test_memory_confirmation_outside_region_cannot_redirect_standoff(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(
            id='passive:gas station:17',
            evidence='passive_semantic_landmarks',
            object_position=[50., 0., 0.], object_radius=.5)
        self.node.memory_goal = self.node.expected = (8., 0.)
        self.node.query_reset_pending = False
        self.node.relay_destination = Mock()

        self.node.on_object_confirmation(self.confirmation(
            object_position=[20., 0., 0.], standoff=[15., 0.]))

        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_outside_memory_region')
        self.assertEqual(self.node.expected, (8., 0.))
        self.node.relay_destination.assert_not_called()

    def test_visible_distant_signal_cannot_finish_even_at_clamped_standoff(self):
        self.node.on_query(String(data='traffic light'))
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[20., 0., 6.], standoff=[0., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_visible_not_arrived')
        self.assertEqual(self.node.last_status['distance'], 20.)

    def test_signal_can_finish_within_five_metres_horizontal_distance(self):
        self.node.on_query(String(data='traffic light'))
        self.node.odom.pose.pose.position.x = 16.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[20., 0., 6.], standoff=[16., 0.]))
        self.assertTrue(self.node.object_found)

    def test_confirmation_is_accepted_after_vehicle_reaches_standoff(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.odom.pose.pose.position.x = 9.5
        self.node.memory_active = None
        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[12., 0., 0.], standoff=[10., 0.]))
        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')

    def test_reaching_a_standoff_does_not_count_while_object_is_still_far(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.odom.pose.pose.position.x = 9.5
        self.node.memory_active = None
        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[20., 0., 0.], standoff=[10., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_visible_not_arrived')
        self.assertAlmostEqual(self.node.last_status['distance'], 10.5)
        self.assertEqual(self.node.last_status['arrival_distance'], 4.5)

    def test_trial12_house_is_not_found_at_the_recorded_6_53m_pose(self):
        self.node.on_query(String(data='green residential house'))
        self.node.odom.pose.pose.position.x = 720.3802952955866
        self.node.odom.pose.pose.position.y = -60.91659203814041
        self.node.memory_active = None
        self.node.expected = (720.9613379677669, -62.087657180638644)
        self.node.following = True

        self.node.on_object_confirmation(self.confirmation(
            query='green residential house',
            object_position=[723.1890398845688, -66.81229983271444, 54.985529073031834],
            observer_position=[719.9490277734399, -59.8514387205204, 51.98424692798793],
            standoff=[720.9613379677669, -62.087657180638644],
            standoff_clamped=True, standoff_object_distance_m=5.223))

        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_before_standoff')
        self.assertAlmostEqual(self.node.last_status['distance'], 1.3072888567, places=6)
        self.assertAlmostEqual(self.node.last_status['object_distance_m'], 6.5305755156, places=6)
        self.assertEqual(self.node.last_status['tolerance'], 0.6)

    def test_trial12_house_is_found_only_after_reaching_its_clamped_safe_standoff(self):
        self.node.on_query(String(data='green residential house'))
        self.node.odom.pose.pose.position.x = 720.9613379677669
        self.node.odom.pose.pose.position.y = -62.087657180638644
        self.node.memory_active = None

        self.node.on_object_confirmation(self.confirmation(
            query='green residential house',
            object_position=[723.1890398845688, -66.81229983271444, 54.985529073031834],
            observer_position=[719.9490277734399, -59.8514387205204, 51.98424692798793],
            standoff=[720.9613379677669, -62.087657180638644],
            standoff_clamped=True, standoff_object_distance_m=5.223))

        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')
        self.assertAlmostEqual(self.node.last_status['standoff_distance_m'], 0.0)
        self.assertAlmostEqual(self.node.last_status['object_arrival_tolerance_m'], 5.823)

    def test_confirmation_outside_standoff_tolerance_is_not_arrival(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.odom.pose.pose.position.x = 8.49
        self.node.memory_active = None
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[20., 0., 0.], standoff=[10., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_standoff_approach')
        self.assertAlmostEqual(self.node.last_status['distance'], 1.51)
        self.assertEqual(self.node.expected, (10., 0.))
        self.node.reset.assert_called_once()
        self.node.relay_destination.assert_called_once()

    def test_passive_memory_cannot_confirm_a_different_building_block(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(object_position=[711.7356, -27.7877, 51.917],
            object_radius=1.28, evidence='passive_semantic_landmarks')
        self.node.on_object_confirmation(self.confirmation(
            object_position=[703.1474, -48.0556, 52.9403],
            observer_position=[706.0734, -16.8977, 52.0676], radius=.88,
            inliers=5, observations=5, standoff=[0., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_outside_memory_region')

    def test_passive_memory_accepts_another_facade_near_the_remembered_region(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(object_position=[711.5024, -20.1252, 51.906],
            object_radius=.90, evidence='passive_semantic_landmarks')
        object_position = [712.3733, -24.6344, 53.8417]
        self.node.odom.pose.pose.position.x = object_position[0]
        self.node.odom.pose.pose.position.y = object_position[1]
        self.node.on_object_confirmation(self.confirmation(
            object_position=object_position, radius=.48,
            inliers=5, observations=5, standoff=object_position[:2]))
        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')

    def test_confirmation_rejects_wrong_query_stale_stamp_and_other_instance(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(object_position=[20., 0., 0.], object_radius=.5)
        for changes in (dict(query='house'), dict(observation_stamp_ns=self.node.query_started_ns-1),
                        dict(object_position=[100., 0., 0.]), dict(radius=float('nan'))):
            self.node.on_object_confirmation(self.confirmation(**changes))
            self.assertFalse(self.node.object_found)

    def test_new_query_releases_found_latch(self):
        self.node.on_query(String(data='gas station'))
        self.node.on_object_confirmation(self.confirmation(
            object_position=[3., 0., 0.], standoff=[0., 0.]))
        self.assertTrue(self.node.object_found)
        self.node.on_query(String(data='house'))
        self.assertFalse(self.node.object_found)




    def test_new_partial_prefix_can_resume_after_nearby_frontier_was_reached(self):
        self.node.expected = (40., 0.)
        self.node.last_plan_start = (7., 0.)
        self.node.last_plan_endpoint = (8., 0.)
        self.node.odom.pose.pose.position.x = 7.5
        self.node.status('route_done')
        self.node.start_segment = Mock()
        path = route((7.5, 0.), (10., 0.))
        self.node.require_route_status = True
        self.node.on_route_status(self.route_status(path))
        self.node.on_route(path)
        self.node.start_segment.assert_called_once()
        self.assertFalse(self.node.route_complete)

    def test_old_unlabelled_waypoint_cannot_change_new_query_destination(self):
        self.node.on_query(String(data='house'))
        late = pose(15., 0.)
        late.header.stamp.sec, late.header.stamp.nanosec = divmod(self.node.query_started_ns-1, 10**9)
        self.node.on_waypoint(late)
        self.assertIsNone(self.node.expected)
        self.node.waypoint_pub.publish.assert_not_called()


class ExplorationTargetHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fixture = GraphBridgeRoutingTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (8., 0.))
        self.node.latest_route = self.node.route
        self.node.route_endpoint = (8., 0.)
        self.node.route_observation_pub = Mock()
        self.node.route_rejection_pub = Mock()
        self.node.odom.pose.pose.orientation.w = 1.
        self.metadata(self.node.route, 'frontier-a:1', (12., 0.))

    def tearDown(self):
        self.node.destroy_node()

    def metadata(self, path, identity, goal, complete=True):
        from wildos_nav2_adapter.input_quality import stamp_ns
        self.node.route_metadata[stamp_ns(path)] = dict(exploration_id=identity,
            goal=list(goal), complete=complete, observation_heading=0.)

    def test_complete_graph_connector_is_extended_to_actual_observation_destination(self):
        self.assertEqual(self.node.current_route_points()[-1], (12., 0.))
        self.assertEqual(self.node.exploration_destination(), (12., 0.))
        self.node.try_candidate = Mock()
        self.node.plan_client = Mock()
        self.node.start_segment()
        self.assertEqual(self.node.candidates[0][:2], (12., 0.))

    def test_partial_route_does_not_jump_to_disconnected_frontier(self):
        self.metadata(self.node.route, 'frontier-a:1', (12., 0.), complete=False)
        self.assertIsNone(self.node.exploration_destination())
        self.assertEqual(self.node.current_route_points()[-1], (8., 0.))

    def test_retired_tail_is_reused_only_after_new_owner_and_map_validation(self):
        from unittest.mock import patch
        self.node.retired_explorations = {'frontier-a:old'}
        self.node.retired_motion = None  # The old action has terminated.
        path = drivable_route((0., 0.), (6., 0.))
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.local_freshness_issue = Mock(return_value=None)
        self.node.execution_transport_issue = Mock(return_value=None)
        self.node.validate_path = Mock(return_value=None)
        self.node.validate_local_path = Mock(return_value=None)
        self.node.exploration_motion_issue = Mock(return_value=None)
        self.node.initial_tracking_issue = Mock(return_value=None)
        self.node.execute_checked_path = Mock()
        with patch.object(self.node, 'current_route_points', return_value=[(0., 0.), (8., 0.)]):
            self.assertTrue(self.node.try_retired_tail_candidate(path, 'frontier-a:old'))
        self.node.validate_path.assert_called_once()
        self.node.validate_local_path.assert_called_once()
        self.node.execute_checked_path.assert_called_once()
        self.assertEqual(self.node.last_status['status'], 'retired_tail_candidate_revalidated')

    def test_retired_tail_snapshot_requires_contiguous_segments(self):
        self.node.motion_queue = [drivable_route((0., 0.), (6., 0.)),
                                  drivable_route((6., 0.), (9., 0.)),
                                  drivable_route((9., 0.), (12., 0.))]
        candidate = self.node.retired_tail_candidate()
        self.assertIsNotNone(candidate)
        self.assertLessEqual(math.dist((candidate.poses[-1].pose.position.x,
                                       candidate.poses[-1].pose.position.y), (12., 0.)), .01)
        self.node.motion_queue[2] = drivable_route((10., 0.), (12., 0.))
        self.assertIsNone(self.node.retired_tail_candidate())

    def test_retired_tail_is_rejected_when_new_goal_progress_or_map_check_fails(self):
        from unittest.mock import patch
        path = drivable_route((0., 0.), (-3., 0.))
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.local_freshness_issue = Mock(return_value=None)
        self.node.execution_transport_issue = Mock(return_value=None)
        self.node.execute_checked_path = Mock()
        self.assertFalse(self.node.try_retired_tail_candidate(path, 'frontier-a:old'))
        blocked = drivable_route((0., 0.), (6., 0.))
        self.node.validate_path = Mock(return_value='collision')
        self.node.validate_local_path = Mock(return_value=None)
        self.node.exploration_motion_issue = Mock(return_value=None)
        self.node.initial_tracking_issue = Mock(return_value=None)
        with patch.object(self.node, 'current_route_points', return_value=[(0., 0.), (8., 0.)]):
            self.assertFalse(self.node.try_retired_tail_candidate(blocked, 'frontier-a:old'))
        self.node.execute_checked_path.assert_not_called()

    def test_object_goal_takes_precedence_over_exploration_metadata(self):
        self.node.expected = self.node.memory_goal = (10., 0.)
        self.assertIsNone(self.node.exploration_metadata(self.node.route))
        self.assertEqual(self.node.current_route_points()[-1], (10., 0.))
        self.assertFalse(self.node.acknowledge_exploration_observation())

    def test_graph_endpoint_and_detection_are_not_arrival_ack(self):
        self.node.odom.pose.pose.position.x = 8.
        self.assertFalse(self.node.acknowledge_exploration_observation())
        self.node.route_observation_pub.publish.assert_not_called()

    def test_arrival_ack_uses_exact_route_identity_not_destination_coordinate(self):
        self.node.odom.pose.pose.position.x = 12.
        self.assertTrue(self.node.acknowledge_exploration_observation())
        message = self.node.route_observation_pub.publish.call_args.args[0]
        self.assertEqual(message.header.stamp.sec, self.node.route.header.stamp.sec)
        self.assertEqual(message.header.stamp.nanosec, self.node.route.header.stamp.nanosec)
        self.assertEqual(message.header.frame_id, self.node.route.header.frame_id)
        self.assertEqual(message.pose.position.x, 8.)

    def test_new_snapshot_same_identity_refreshes_ack_but_other_goal_cannot(self):
        self.node.odom.pose.pose.position.x = 12.
        newest = route((1., 0.), (9., 0.))
        self.metadata(newest, 'frontier-a:1', (12., 0.))
        self.node.latest_route = newest
        self.assertTrue(self.node.acknowledge_exploration_observation())
        self.assertEqual(self.node.route_observation_pub.publish.call_args.args[0].header.stamp.sec, newest.header.stamp.sec)
        self.assertEqual(self.node.route_observation_pub.publish.call_args.args[0].header.stamp.nanosec, newest.header.stamp.nanosec)
        self.metadata(newest, 'frontier-b:2', (15., 0.))
        self.assertTrue(self.node.acknowledge_exploration_observation())
        self.assertEqual(self.node.route_observation_pub.publish.call_args.args[0].header.stamp.sec, self.node.route.header.stamp.sec)
        self.assertEqual(self.node.route_observation_pub.publish.call_args.args[0].header.stamp.nanosec, self.node.route.header.stamp.nanosec)

    def test_active_motion_bad_heading_speed_stale_input_and_query_reset_cannot_ack(self):
        self.node.odom.pose.pose.position.x = 12.
        for name in ('following', 'pending', 'query_reset_pending'):
            setattr(self.node, name, True)
            self.assertFalse(self.node.acknowledge_exploration_observation(), name)
            setattr(self.node, name, False)
        self.node.odom.twist.twist.linear.x = .3
        self.assertFalse(self.node.acknowledge_exploration_observation())
        self.node.odom.twist.twist.linear.x = 0.
        self.node.odom.pose.pose.orientation.z = math.sin(.8/2)
        self.node.odom.pose.pose.orientation.w = math.cos(.8/2)
        self.assertFalse(self.node.acknowledge_exploration_observation())
        self.node.odom.pose.pose.orientation.z = 0.
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom_received_at -= 5.
        self.assertFalse(self.node.acknowledge_exploration_observation())

    def test_query_fence_and_partial_route_refuse_ack(self):
        self.node.odom.pose.pose.position.x = 12.
        self.node.query_started_ns = self.node.get_clock().now().nanoseconds + 1
        self.assertFalse(self.node.acknowledge_exploration_observation())
        self.node.query_started_ns = 0
        self.metadata(self.node.route, 'frontier-a:1', (12., 0.), complete=False)
        self.assertFalse(self.node.acknowledge_exploration_observation())

    def test_nonfinite_velocity_cannot_acknowledge(self):
        self.node.odom.pose.pose.position.x = 12.
        self.node.odom.twist.twist.linear.x = math.nan
        self.assertFalse(self.node.acknowledge_exploration_observation())

    def test_refusal_cannot_borrow_new_owner_at_same_graph_endpoint(self):
        self.node.route_rejection_pub = Mock()
        old = self.node.route
        latest = route((1., 0.), (8., 0.))
        self.metadata(latest, 'frontier-b:2', (16., 0.))
        self.node.latest_route = latest
        self.assertTrue(self.node.reject_exploration_route())
        message = self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(message.header.stamp.sec, old.header.stamp.sec)
        self.assertEqual(message.header.stamp.nanosec, old.header.stamp.nanosec)

    def test_long_action_refusal_refreshes_from_same_destination_new_graph_connector(self):
        old = self.node.route
        latest = route((3., 0.), (9., 0.))
        self.metadata(latest, 'frontier-a:1', (12., 0.))
        self.node.latest_route = latest
        self.assertTrue(self.node.reject_exploration_route())
        message = self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(message.header.stamp.sec, latest.header.stamp.sec)
        self.assertEqual(message.header.stamp.nanosec, latest.header.stamp.nanosec)
        self.assertEqual(message.pose.position.x, 9.)

    def test_route_done_publishes_ack_and_shared_identity(self):
        self.node.odom.pose.pose.position.x = 12.
        self.node.start_segment()
        self.assertEqual(self.node.last_status['status'], 'route_done')
        self.assertEqual(self.node.last_status['exploration_id'], 'frontier-a:1')
        self.assertEqual(self.node.last_status['exploration_goal'], [12., 0.])
        self.node.route_observation_pub.publish.assert_called_once()

    def retirement(self, identity='frontier-a:1', stamp=None):
        return String(data=json.dumps(dict(exploration_id=identity,
            stamp_ns=stamp or self.node.get_clock().now().nanoseconds,
            reason='graph_path_unavailable')))

    def test_retirement_cancels_active_owner_and_fences_late_path(self):
        path = self.node.route
        self.node.following = True
        handle = self.node.follow_handle = Mock()
        self.node.follow_active = dict(serial=7, handle=handle)
        self.node.on_exploration_retired(self.retirement())
        self.assertFalse(self.node.following)
        self.assertIsNone(self.node.route)
        handle.cancel_goal_async.assert_called_once()
        self.assertIn(7, self.node.follow_quarantine)
        self.assertTrue(self.node.follow_hold)
        self.node.start_segment = Mock()
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()
        self.assertIsNone(self.node.latest_route)

    def accepted_short_section(self, length=6.):
        self.node.motion_queue = [drivable_route((0., 0.), (length, 0.)),
                                  drivable_route((length, 0.), (12., 0.))]
        self.node.following = True
        handle = self.node.follow_handle = Mock()
        self.node.follow_active = dict(serial=7, handle=handle)
        self.node.active_follow_serial = 7
        return handle

    def resolve_frontier(self, reason='frontier_resolved'):
        message = json.loads(self.retirement().data)
        message['reason'] = reason
        self.node.on_exploration_retired(String(data=json.dumps(message)))

    def complete_section(self, status=4, epoch=None, serial=7):
        future = Future()
        future.set_result(SimpleNamespace(status=status))
        self.node.on_follow_result(future, self.node.epoch if epoch is None else epoch,
                                   (6., 0.), serial)

    def test_resolved_frontier_finishes_safe_bounded_accepted_section(self):
        path = self.node.route
        handle = self.accepted_short_section()
        epoch = self.node.epoch
        self.node.escape_trace.append((0., 0., 0., time.monotonic()))
        self.resolve_frontier()
        handle.cancel_goal_async.assert_not_called()
        self.assertEqual(self.node.epoch, epoch)
        self.assertEqual(self.node.active_follow_serial, 7)
        self.assertTrue(self.node.following)
        self.assertFalse(self.node.follow_hold)
        self.assertEqual(len(self.node.escape_trace), 1)
        self.assertEqual(self.node.last_status['status'], 'exploration_finishing_segment')
        self.node.route_observation_pub.publish.assert_not_called()
        self.node.route_rejection_pub.publish.assert_not_called()
        self.node.start_segment = Mock()
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()

    def test_retired_section_cannot_extend_or_execute_old_tail(self):
        handle = self.accepted_short_section()
        self.resolve_frontier()
        self.node.start_segment = Mock()
        self.node.maybe_extend_path()
        self.node.start_segment.assert_not_called()
        self.node.send_motion_segment = Mock()
        self.node.odom.pose.pose.position.x = 6.
        self.complete_section()
        self.assertFalse(self.node.following)
        self.assertIsNone(self.node.retired_motion)
        self.assertIsNone(self.node.route)
        self.assertEqual(self.node.motion_queue, [])
        self.node.send_motion_segment.assert_not_called()
        self.node.start_segment.assert_not_called()
        handle.cancel_goal_async.assert_not_called()
        self.node.route_observation_pub.publish.assert_not_called()

    def test_new_route_without_useful_tail_waits_for_terminal_then_is_replanned(self):
        self.accepted_short_section()
        self.resolve_frontier()
        # The old tail goes east; the new owner requests west. It cannot be
        # reused merely because the old controller action has completed.
        latest = drivable_route((6., 0.), (-6., 0.))
        self.metadata(latest, 'frontier-b:2', (-6., 0.))
        self.node.start_segment = Mock()
        self.node.on_route(latest)
        self.node.start_segment.assert_not_called()
        self.node.odom.pose.pose.position.x = 6.
        self.complete_section()
        self.assertIs(self.node.route, latest)
        self.node.start_segment.assert_called_once()

    def test_expired_replacement_is_not_executed_after_terminal(self):
        self.accepted_short_section()
        self.resolve_frontier()
        latest = route((6., 0.), (16., 0.))
        self.metadata(latest, 'frontier-b:2', (16., 0.))
        self.node.latest_route = latest
        latest.header.stamp.sec -= 6
        self.node.start_segment = Mock()
        self.node.odom.pose.pose.position.x = 6.
        self.complete_section()
        self.assertIsNone(self.node.route)
        self.node.start_segment.assert_not_called()

    def test_resolved_frontier_keeps_cancel_for_unbounded_reverse_section(self):
        handle = self.accepted_short_section(9.)
        self.node.motion_queue[0] = drivable_route((0., 0.), (-9., 0.))
        self.resolve_frontier()
        handle.cancel_goal_async.assert_called_once()
        self.assertIn(7, self.node.follow_quarantine)
        self.assertIsNone(self.node.route)

    def test_resolved_frontier_keeps_cancel_for_local_collision(self):
        handle = self.accepted_short_section()
        self.node.local_grid.data[10*40+13] = 100
        self.resolve_frontier()
        handle.cancel_goal_async.assert_called_once()
        self.assertTrue(self.node.follow_hold)

    def test_resolved_frontier_keeps_cancel_for_global_collision(self):
        handle = self.accepted_short_section()
        self.node.grid.data[10*40+15] = 100
        self.resolve_frontier()
        handle.cancel_goal_async.assert_called_once()

    def test_stale_inputs_during_drain_still_cancel_and_clear_owner(self):
        handle = self.accepted_short_section()
        self.resolve_frontier()
        self.node.odom_received_at -= 5.
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called()
        self.assertIsNone(self.node.retired_motion)
        self.assertTrue(self.node.follow_hold)

    def test_native_failure_during_drain_still_cancels_and_clears_owner(self):
        handle = self.accepted_short_section()
        self.resolve_frontier()
        self.node.native_status['actuator_ok'] = False
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called()
        self.assertIsNone(self.node.retired_motion)
        self.assertTrue(self.node.follow_hold)

    def test_stale_cusp_terminal_does_not_leave_drain_marker_for_next_action(self):
        self.accepted_short_section()
        self.resolve_frontier()
        self.node.odom_received_at -= 5.
        self.complete_section()
        self.assertIsNone(self.node.retired_motion)

    def test_late_wrong_action_terminal_does_not_finish_draining_section(self):
        self.accepted_short_section()
        self.resolve_frontier()
        self.node.odom.pose.pose.position.x = 6.
        self.complete_section(serial=6)
        self.assertTrue(self.node.following)
        self.assertIsNotNone(self.node.retired_motion)

    def test_resolved_frontier_keeps_cancel_for_stale_or_uncertain_execution(self):
        for field, value in (('pending', True), ('replacement_pending', True),
                             ('follow_request', {'serial': 8}),
                             ('odom_received_at', time.monotonic()-5.)):
            with self.subTest(field=field):
                old = getattr(self.node, field)
                handle = self.accepted_short_section()
                setattr(self.node, field, value)
                self.resolve_frontier()
                handle.cancel_goal_async.assert_called()
                setattr(self.node, field, old)
                # Reset clears route ownership; rebuild it for the next case.
                self.node.route = self.node.latest_route = route((0., 0.), (8., 0.))
                self.metadata(self.node.route, 'frontier-a:1', (12., 0.))
                self.node.follow_quarantine.clear()
                self.node.follow_hold = False

    def test_duplicate_retirement_does_not_cancel_draining_action(self):
        handle = self.accepted_short_section()
        self.resolve_frontier()
        self.resolve_frontier()
        self.assertTrue(self.node.following)
        handle.cancel_goal_async.assert_not_called()

    def test_query_change_still_cancels_draining_action_and_late_result(self):
        handle = self.accepted_short_section()
        epoch = self.node.epoch
        self.resolve_frontier()
        self.node.on_query(String(data='bus stop'))
        handle.cancel_goal_async.assert_called()
        self.assertFalse(self.node.following)
        self.assertIsNone(self.node.retired_motion)
        self.node.send_motion_segment = Mock()
        self.complete_section(epoch=epoch)
        self.node.send_motion_segment.assert_not_called()

    def test_frame_change_still_cancels_draining_action(self):
        handle = self.accepted_short_section()
        self.resolve_frontier()
        message = json.loads(self.retirement().data)
        message['reason'] = 'graph_frame_changed'
        self.node.on_exploration_retired(String(data=json.dumps(message)))
        handle.cancel_goal_async.assert_called()
        self.assertFalse(self.node.following)
        self.assertIsNone(self.node.retired_motion)

    def test_draining_action_still_checks_actual_cusp_and_aborts(self):
        self.accepted_short_section()
        self.resolve_frontier()
        self.node.follow_failed = Mock()
        self.complete_section()
        self.node.follow_failed.assert_called_once_with('cusp_not_reached', goal=[6., 0.])

    def test_draining_action_failure_uses_existing_failure_handler(self):
        self.accepted_short_section()
        self.resolve_frontier()
        self.node.follow_failed = Mock()
        self.complete_section(status=6)
        self.node.follow_failed.assert_called_once_with('follow_aborted', action_status=6)

    def test_resolved_frontier_preserves_same_map_history_and_independent_escape(self):
        self.node.escape_trace.append((0., 0., 0., time.monotonic()))
        self.node.recovering = self.node.following = True
        handle = self.node.follow_handle = Mock()
        self.node.follow_active = dict(serial=9, handle=handle)
        self.resolve_frontier()
        self.assertTrue(self.node.recovering)
        self.assertTrue(self.node.following)
        self.assertEqual(len(self.node.escape_trace), 1)
        handle.cancel_goal_async.assert_not_called()

    def test_old_retirement_cannot_cancel_new_owner_or_object_destination(self):
        self.metadata(self.node.route, 'frontier-b:2', (12., 0.))
        self.node.reset = Mock()
        self.node.on_exploration_retired(self.retirement('frontier-a:1'))
        self.node.reset.assert_not_called()
        self.node.expected = self.node.memory_goal = (12., 0.)
        self.node.on_exploration_retired(self.retirement('frontier-b:2'))
        self.node.reset.assert_not_called()

    def test_retirement_before_path_and_status_cannot_be_reversed_by_late_delivery(self):
        path = self.node.route
        self.node.route = self.node.latest_route = None
        self.node.route_metadata = {}
        self.node.require_route_status = True
        self.node.on_exploration_retired(self.retirement())
        self.node.on_route(path)
        from wildos_nav2_adapter.input_quality import stamp_ns
        self.node.start_segment = Mock()
        self.node.on_route_status(String(data=json.dumps(dict(stamp_ns=stamp_ns(path),
            exploration_id='frontier-a:1',goal=[12., 0.],complete=True))))
        self.node.start_segment.assert_not_called()
        self.assertIsNone(self.node.latest_route)

    def test_stale_future_and_previous_query_retirements_cannot_cancel(self):
        self.node.reset = Mock()
        now = self.node.get_clock().now().nanoseconds
        self.node.on_exploration_retired(self.retirement(stamp=now-6*10**9))
        self.node.on_exploration_retired(self.retirement(stamp=now+10**9))
        self.node.query_started_ns = now
        self.node.on_exploration_retired(self.retirement(stamp=now-1))
        self.node.on_exploration_retired(String(data='{"stamp_ns":Infinity,"exploration_id":"x"}'))
        self.node.reset.assert_not_called()

    def test_retirement_cache_is_bounded_and_duplicate_does_not_reset_again(self):
        self.node.on_exploration_retired(self.retirement())
        self.node.reset = Mock()
        self.node.on_exploration_retired(self.retirement())
        self.node.reset.assert_not_called()
        for index in range(80):
            self.node.on_exploration_retired(self.retirement('other:'+str(index)))
        self.assertEqual(len(self.node.retired_explorations), 64)

    def test_frontier_retirement_preserves_recent_measured_escape_corridor(self):
        self.node.escape_trace.extend([(-3., 0., 0., time.monotonic()),
                                      (0., 0., 0., time.monotonic())])
        before = list(self.node.escape_trace)
        self.node.on_exploration_retired(self.retirement())
        self.assertEqual(list(self.node.escape_trace), before)

    def test_rejected_frontier_retirement_does_not_cancel_independent_escape(self):
        self.node.recovering = True
        escape = self.node.escape_path = drivable_route((0., 0.), (-3., 0.))
        self.node.motion_queue = [escape]
        self.node.following = True
        handle = self.node.follow_handle = Mock()
        self.node.follow_active = dict(serial=9, handle=handle)
        self.node.on_exploration_retired(String(data=json.dumps(dict(
            exploration_id='frontier-a:1', stamp_ns=self.node.get_clock().now().nanoseconds,
            reason='vehicle_route_rejected'))))
        self.assertTrue(self.node.recovering)
        self.assertIs(self.node.escape_path, escape)
        handle.cancel_goal_async.assert_not_called()

    def prepare_planning_escape(self):
        self.node.escape_trace.extend((x*.3, 0., 0., time.monotonic())
                                      for x in range(-10, 1))
        self.node.failure_reasons = ['empty_path']
        self.node.send_motion_segment = Mock()

    def second_failed_destination(self):
        self.metadata(self.node.route, 'frontier-b:2', (12., 0.))

    def test_distinct_failed_destinations_use_one_measured_reverse_corridor(self):
        self.prepare_planning_escape()
        self.assertFalse(self.node.try_planning_escape())
        self.assertFalse(self.node.try_planning_escape())  # same goal is not a second region
        self.second_failed_destination()
        self.assertTrue(self.node.try_planning_escape())
        self.assertTrue(self.node.recovering)
        self.assertLess(self.node.escape_path.poses[-1].pose.position.x, -1.9)
        self.node.send_motion_segment.assert_called_once()
        self.assertFalse(self.node.try_planning_escape())

    def test_planning_escape_needs_measured_recent_corridor(self):
        self.prepare_planning_escape()
        self.node.escape_trace.clear()
        self.assertFalse(self.node.try_planning_escape())
        self.second_failed_destination()
        self.assertFalse(self.node.try_planning_escape())
        self.node.send_motion_segment.assert_not_called()

    def test_planning_escape_rejects_old_corridor_and_repeated_attempt_position(self):
        self.prepare_planning_escape()
        self.node.escape_trace = type(self.node.escape_trace)(
            [(x, y, h, t-61.) for x,y,h,t in self.node.escape_trace], maxlen=96)
        self.assertFalse(self.node.try_planning_escape())
        self.second_failed_destination()
        self.assertFalse(self.node.try_planning_escape())
        self.node.escape_trace.clear()
        self.node.escape_trace.extend((x*.3,0.,0.,time.monotonic()) for x in range(-10,1))
        self.node.escape_attempt_position = (0.,0.)
        self.assertFalse(self.node.try_planning_escape())

    def test_planning_escape_does_not_replace_object_goal_or_uncertain_action(self):
        self.prepare_planning_escape()
        for attribute,value in [('following',True),('pending',True),('replacement_pending',True),
                ('query_reset_pending',True),('memory_selecting',True),('expected',(12.,0.)),
                ('memory_goal',(12.,0.)),('follow_enabled',False)]:
            before=getattr(self.node,attribute)
            setattr(self.node,attribute,value)
            self.assertFalse(self.node.try_planning_escape(),attribute)
            setattr(self.node,attribute,before)
        self.node.follow_quarantine[8] = dict(terminal=False)
        self.assertFalse(self.node.try_planning_escape())
        self.node.send_motion_segment.assert_not_called()

    def test_planning_escape_not_triggered_by_protocol_or_geometry_errors(self):
        self.prepare_planning_escape()
        self.node.failure_reasons = ['planner_rejected']
        self.assertFalse(self.node.try_planning_escape())
        self.node.failure_reasons = ['empty_path','excess_curvature']
        self.assertFalse(self.node.try_planning_escape())
        self.assertEqual(len(self.node.planning_escape_failures),0)

    def test_planning_escape_requires_fresh_maps_transport_and_stopped_vehicle(self):
        self.prepare_planning_escape()
        self.node.odom.twist.twist.linear.x=.3
        self.assertFalse(self.node.try_planning_escape())
        self.node.odom.twist.twist.linear.x=0.
        self.node.execution_transport_issue=Mock(return_value='unavailable')
        self.assertFalse(self.node.try_planning_escape())
        self.node.execution_transport_issue=Mock(return_value=None)
        self.node.local_grid_received_at-=5.
        self.assertFalse(self.node.try_planning_escape())
        self.node.send_motion_segment.assert_not_called()

    def test_planning_failures_survive_temporary_local_staleness_and_idle_retirement(self):
        self.prepare_planning_escape()
        self.node.local_grid_received_at -= 5.
        self.assertFalse(self.node.try_planning_escape())
        self.second_failed_destination()
        self.assertFalse(self.node.try_planning_escape())
        self.node.on_exploration_retired(String(data=json.dumps(dict(
            exploration_id='frontier-b:2', stamp_ns=self.node.get_clock().now().nanoseconds,
            reason='vehicle_route_rejected'))))
        self.assertIsNone(self.node.route)
        self.node.local_grid_received_at = time.monotonic()
        self.node.native_status_at = time.monotonic()
        self.node.monitor_inputs()
        self.node.send_motion_segment.assert_called_once()
        self.assertTrue(self.node.recovering)

    def test_idle_planning_escape_intent_cannot_cross_a_query_reset(self):
        self.prepare_planning_escape()
        self.node.execution_transport_issue = Mock(return_value='native_telemetry_stale')
        self.assertFalse(self.node.try_planning_escape())
        self.second_failed_destination()
        self.assertFalse(self.node.try_planning_escape())
        self.node.reset()
        self.node.execution_transport_issue.return_value = None
        self.node.monitor_inputs()
        self.node.send_motion_segment.assert_not_called()

    def defer_planning_escape(self):
        self.prepare_planning_escape()
        self.node.local_grid_received_at -= 5.
        self.node.try_planning_escape()
        self.second_failed_destination()
        self.node.try_planning_escape()
        self.assertIsNotNone(self.node.deferred_planning_escape)
        self.node.local_grid_received_at = time.monotonic()
        self.node.native_status_at = time.monotonic()

    def test_deferred_planning_escape_checks_evidence_age_position_frame_and_goal_scope(self):
        for case in ('expired_failure','moved','frame','object','memory','disabled'):
            self.node.reset()
            self.node.route=self.node.latest_route=route((0.,0.),(8.,0.))
            self.node.route_endpoint=(8.,0.)
            self.node.expected=self.node.memory_goal=None
            self.node.follow_enabled=True
            self.node.odom.pose.pose.position.x=0.
            self.node.odom.header.frame_id='spot1/odom'
            self.metadata(self.node.route,'frontier-a:1',(12.,0.))
            self.defer_planning_escape()
            if case=='expired_failure':
                self.node.planning_escape_failures[0]=(*self.node.planning_escape_failures[0][:3],time.monotonic()-61.)
            elif case=='moved':self.node.odom.pose.pose.position.x=1.
            elif case=='frame':self.node.odom.header.frame_id='other'
            elif case=='object':self.node.expected=(12.,0.)
            elif case=='memory':self.node.memory_goal=(12.,0.)
            else:self.node.follow_enabled=False
            self.assertFalse(self.node.resume_planning_escape(),case)
            self.assertIsNone(self.node.deferred_planning_escape,case)
            self.node.send_motion_segment.assert_not_called()

    def test_deferred_planning_escape_does_not_steal_active_or_uncertain_motion(self):
        self.defer_planning_escape()
        for name,value in [('pending',True),('following',True),('replacement_pending',True),
                ('follow_request',{'serial':7}),('follow_active',{'serial':7}),
                ('follow_quarantine',{7:{'terminal':False}})]:
            old=getattr(self.node,name);setattr(self.node,name,value)
            self.assertFalse(self.node.resume_planning_escape(),name)
            setattr(self.node,name,old)
        self.node.send_motion_segment.assert_not_called()

    def test_deferred_escape_dispatch_and_acceptance_work_without_retired_graph_route(self):
        self.defer_planning_escape()
        self.node.on_exploration_retired(String(data=json.dumps(dict(
            exploration_id='frontier-b:2',stamp_ns=self.node.get_clock().now().nanoseconds,
            reason='vehicle_route_rejected'))))
        self.node.send_motion_segment=GraphPathBridge.send_motion_segment.__get__(self.node)
        self.node.follow_client=Mock()
        accepted=Future();terminal=Future()
        self.node.follow_client.send_goal_async.return_value=accepted
        self.node.monitor_inputs()
        self.node.follow_client.send_goal_async.assert_called_once()
        handle=Mock(accepted=True);handle.get_result_async.return_value=terminal
        accepted.set_result(handle)
        self.assertTrue(self.node.following)
        self.assertTrue(self.node.recovering)
        self.assertIsNone(self.node.last_status['graph_goal'])
        self.assertIsNone(self.node.deferred_planning_escape)

    def test_retirement_frame_change_cancels_escape_and_clears_corridor(self):
        self.prepare_planning_escape()
        self.node.recovering=True
        handle=self.node.follow_handle=Mock()
        self.node.on_exploration_retired(String(data=json.dumps(dict(
            exploration_id='frontier-a:1',stamp_ns=self.node.get_clock().now().nanoseconds,
            reason='graph_frame_changed'))))
        self.assertFalse(self.node.recovering)
        self.assertEqual(len(self.node.escape_trace),0)
        handle.cancel_goal_async.assert_called_once()

    def test_escape_completion_cannot_resume_retired_graph_route(self):
        self.prepare_planning_escape()
        self.node.plan_client=Mock()
        self.node.retired_explorations['frontier-a:1']=self.node.get_clock().now().nanoseconds
        self.node.latest_route=None
        self.node.finish_execution_escape(True,'escape_arrived')
        self.assertEqual(self.node.last_status['status'],'awaiting_graph_route')
        self.node.plan_client.send_goal_async.assert_not_called()

    def test_planning_failure_and_trace_survive_frontier_retirement_for_next_goal(self):
        self.prepare_planning_escape()
        old=self.node.route
        self.assertFalse(self.node.try_planning_escape())
        self.node.on_exploration_retired(self.retirement())
        self.assertEqual(len(self.node.planning_escape_failures),1)
        self.node.route=self.node.latest_route=old
        self.node.route_endpoint=(8.,0.)
        self.second_failed_destination()
        self.node.failure_reasons=['empty_path']
        self.assertTrue(self.node.try_planning_escape())

    def test_planning_escape_still_checks_local_obstacles_before_dispatch(self):
        self.prepare_planning_escape()
        self.node.send_motion_segment=GraphPathBridge.send_motion_segment.__get__(self.node)
        self.node.follow_client=Mock()
        self.node.local_grid.data[10*40+8]=100
        self.assertFalse(self.node.try_planning_escape())
        self.second_failed_destination()
        self.assertTrue(self.node.try_planning_escape())
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertFalse(self.node.recovering)
        self.assertEqual(self.node.last_status['status'],'follow_failed')

    def test_escape_controller_absence_does_not_latch_recovering_without_action(self):
        self.prepare_planning_escape()
        self.node.send_motion_segment=GraphPathBridge.send_motion_segment.__get__(self.node)
        self.node.follow_client=Mock()
        self.node.follow_client.wait_for_server.return_value=False
        self.assertFalse(self.node.try_planning_escape())
        self.second_failed_destination()
        self.assertTrue(self.node.try_planning_escape())
        self.assertFalse(self.node.recovering)
        self.assertIsNone(self.node.escape_path)
        self.assertEqual(self.node.motion_queue,[])
        self.assertIsNone(self.node.escape_attempt_position)
        self.assertEqual(self.node.last_status['status'],'controller_unavailable')
        self.node.follow_client.send_goal_async.assert_not_called()
        self.node.follow_client.wait_for_server.return_value=True
        self.node.follow_client.send_goal_async.return_value=Future()
        self.assertFalse(self.node.try_planning_escape())  # controller retries are rate bounded
        self.node.deferred_planning_escape['retry_at']=time.monotonic()
        self.assertTrue(self.node.try_planning_escape())
        self.assertTrue(self.node.recovering)
        self.assertTrue(self.node.following)
        self.assertIsNotNone(self.node.follow_request)
        self.node.follow_client.send_goal_async.assert_called_once()

    def test_escape_map_expiring_at_dispatch_releases_recovery_owner(self):
        self.prepare_planning_escape()
        poses=[(-i*.2,0.,0.) for i in range(16)]
        self.node.send_motion_segment=GraphPathBridge.send_motion_segment.__get__(self.node)
        self.node.follow_client=Mock()
        self.node.input_freshness_issue=Mock(return_value=('stale_costmap',4.))
        self.node.start_measured_escape((0.,0.),poses)
        self.assertFalse(self.node.recovering)
        self.assertIsNone(self.node.escape_path)
        self.assertEqual(self.node.motion_queue,[])
        self.assertIsNone(self.node.escape_attempt_position)
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_measured_escape_cannot_take_ownership_of_an_uncertain_action(self):
        poses=[(-i*.2,0.,0.) for i in range(16)]
        self.node.send_motion_segment=Mock()
        self.node.reject_exploration_route=Mock()
        for name,value in [('follow_quarantine',{9:dict(terminal=False)}),
                           ('follow_request',dict(serial=9)),
                           ('follow_active',dict(serial=9)),('following',True),
                           ('pending',True),('replacement_pending',True)]:
            old=getattr(self.node,name)
            setattr(self.node,name,value)
            self.assertFalse(self.node.start_measured_escape((0.,0.),poses),name)
            self.assertFalse(self.node.recovering,name)
            setattr(self.node,name,old)
        self.node.reject_exploration_route.assert_not_called()
        self.node.send_motion_segment.assert_not_called()

    def _zigzag_path(self, reversals):
        # Alternate forward/backward along x so each edge flips direction.
        path = Path()
        path.header.frame_id = "spot1/odom"
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.poses = [pose(2. * (i % 2), 0.) for i in range(reversals + 1)]
        return path

    def _ready_for_checked_path(self):
        self.node.follow_enabled = True
        self.node.odom = SimpleNamespace(
            pose=SimpleNamespace(pose=pose(0., 0.).pose), header=Mock())
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.validate_path = Mock(return_value=None)
        self.node.initial_tracking_issue = Mock(return_value=None)
        self.node.candidate_failed = Mock()
        self.node.send_motion_segment = Mock()

    def test_excessive_motion_sections_rejects_the_candidate(self):
        # 181516 epoch 6: a 10-section path shuffled inside 9 m for 180 s.
        self._ready_for_checked_path()
        path = self._zigzag_path(8)
        self.assertEqual(len(motion_sections([(p.pose.position.x, p.pose.position.y)
                                              for p in path.poses])), 8)
        self.node.execute_checked_path(path, (10., 0.))
        self.node.candidate_failed.assert_called_once_with("excessive_motion_sections")
        self.assertEqual(self.node.motion_queue, [])
        self.node.send_motion_segment.assert_not_called()

    def test_four_section_maneuver_still_runs(self):
        # 130635 escaped the start area with at most four sections.
        self._ready_for_checked_path()
        path = self._zigzag_path(4)
        self.node.execute_checked_path(path, (10., 0.))
        self.node.candidate_failed.assert_not_called()
        self.node.send_motion_segment.assert_called_once()

    def test_excessive_sections_are_rejected_before_retaining_heading_candidate(self):
        self._ready_for_checked_path()
        self.node.candidates=[(10.,0.,1.,10.),(10.,0.,0.,10.)]
        self.node.candidate_index=0
        self.node.try_candidate=Mock()
        path=drivable_route(*[(x,0.) for x in
            (0.,2.,1.,2.,1.,2.,1.,2.,1.,10.)])
        future=Mock()
        future.result.return_value=SimpleNamespace(status=4,
            result=SimpleNamespace(path=path))
        self.node.on_plan_result(future,self.node.epoch,0)
        self.node.candidate_failed.assert_called_once_with('excessive_motion_sections')
        self.assertIsNone(self.node.normal_best)
        self.node.send_motion_segment.assert_not_called()

    def test_rejected_heading_preserves_and_executes_previous_admissible_path(self):
        self._ready_for_checked_path()
        # Exercise candidate_failed rather than the execution-only fixture mock.
        self.node.candidate_failed=GraphPathBridge.candidate_failed.__get__(self.node)
        self.node.candidates=[(10.,0.,1.,10.),(10.,0.,0.,10.)]
        self.node.candidate_index=0
        self.node.try_candidate=Mock()
        future=Mock()
        good=drivable_route((0.,0.),(10.,0.))
        future.result.return_value=SimpleNamespace(status=4,
            result=SimpleNamespace(path=good))
        self.node.on_plan_result(future,self.node.epoch,0)
        self.assertIsNotNone(self.node.normal_best)
        future.result.return_value.result.path=drivable_route(*[(x,0.) for x in
            (0.,2.,1.,2.,1.,2.,1.,2.,1.,10.)])
        self.node.on_plan_result(future,self.node.epoch,1)
        self.assertIn('excessive_motion_sections',self.node.failure_reasons)
        self.node.send_motion_segment.assert_called_once()
        self.assertEqual(len(self.node.motion_queue),1)
        self.assertEqual(
            [(p.pose.position.x,p.pose.position.y) for p in self.node.motion_queue[0].poses],
            [(p.pose.position.x,p.pose.position.y) for p in good.poses])

    def test_section_cap_also_applies_to_object_and_memory_legs(self):
        for attribute, value in (('expected', (5., 5.)), ('memory_goal', (5., 5.)),
                                 ('memory_selecting', True), ('lookback_active', True)):
            with self.subTest(attribute):
                self._ready_for_checked_path()
                setattr(self.node, attribute, value)
                path = self._zigzag_path(8)
                self.node.execute_checked_path(path, (10., 0.))
                self.node.candidate_failed.assert_called_once_with('excessive_motion_sections')
                self.node.send_motion_segment.assert_not_called()
                setattr(self.node, attribute, False if isinstance(value,bool) else None)

    def test_excessive_extension_preserves_accepted_motion(self):
        self._ready_for_checked_path()
        accepted = drivable_route((0.,0.),(12.,0.))
        self.node.motion_queue = [accepted]
        self.node.following = True
        self.node.extending_path = True
        self.node.execute_checked_path(self._zigzag_path(16), (10.,0.))
        self.assertTrue(self.node.following)
        self.assertFalse(self.node.extending_path)
        self.assertEqual(self.node.motion_queue, [accepted])
        self.assertEqual(self.node.last_status.get('extension_rejected'), 'excessive_motion_sections')
        self.node.candidate_failed.assert_not_called()
        self.node.send_motion_segment.assert_not_called()

    def test_bounded_object_maneuver_still_runs(self):
        self._ready_for_checked_path()
        self.node.expected = (10.,0.)
        self.node.execute_checked_path(self._zigzag_path(4), (10.,0.))
        self.node.candidate_failed.assert_not_called()
        self.node.send_motion_segment.assert_called_once()

    def _ready_for_failure_streak(self):
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.candidates = [(5., 0., 0., 5.), (6., 0., 0., 6.)]
        self.node.candidate_index = 0
        self.node.try_candidate = Mock()
        self.node.block_current_route = Mock()
        self.node.reject_exploration_route = Mock()
        self.node.status = Mock()
        self.node.empty_path_streak = 0
        self.node.empty_path_streak_epoch = self.node.epoch

    def test_repeated_empty_paths_retire_blocked_exploration_route(self):
        # 183339 epoch 4: 19 consecutive empty_path candidates burned ~33 s.
        self._ready_for_failure_streak()
        for _ in range(self.node.max_consecutive_empty_paths):
            self.node.candidate_failed('empty_path')
        self.node.block_current_route.assert_called_once()
        self.node.reject_exploration_route.assert_called_once()
        # The streak is cleared with the retirement; a later failure starts over.
        self.node.candidate_failed('empty_path')
        self.node.block_current_route.assert_called_once()

    def test_empty_path_streak_resets_on_other_reason_and_new_epoch(self):
        for reset in ('other_reason', 'epoch'):
            with self.subTest(reset):
                self._ready_for_failure_streak()
                for _ in range(self.node.max_consecutive_empty_paths - 1):
                    self.node.candidate_failed('empty_path')
                if reset == 'other_reason':
                    self.node.candidate_failed('wrong_start')
                else:
                    self.node.epoch += 1
                for _ in range(self.node.max_consecutive_empty_paths - 1):
                    self.node.candidate_failed('empty_path')
                self.node.block_current_route.assert_not_called()

    def test_empty_path_streak_resets_on_a_new_route_identity(self):
        self._ready_for_failure_streak()
        for _ in range(self.node.max_consecutive_empty_paths - 1):
            self.node.candidate_failed('empty_path')
        fresh = route((0., 0.), (10., 0.))
        fresh.header.stamp.nanosec += 1_000_000
        if fresh.header.stamp.nanosec >= 1_000_000_000:
            fresh.header.stamp.sec += 1
            fresh.header.stamp.nanosec -= 1_000_000_000
        self.node.route = fresh
        self.node.candidate_failed('empty_path')
        self.node.block_current_route.assert_not_called()

    def test_route_points_without_route_is_empty_not_a_crash(self):
        # 205615: a late planner result called current_route_points after the
        # route was cleared and died on None.poses.
        self.node.route = None
        self.node.recovering = False
        self.assertEqual(self.node.current_route_points(), [])

    def test_exhausted_candidates_without_route_endpoint_do_not_crash(self):
        # 205052: the bridge died here with
        # TypeError: 'NoneType' object is not iterable (list(None)).
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = None
        self.node.candidates = []
        self.node.candidate_index = 0
        self.node.failure_reasons = ['empty_path']
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.try_planning_escape = Mock(return_value=False)
        self.node.memory_candidates_exhausted = Mock(return_value=False)
        self.node.fail_confirmed_approach = Mock(return_value=False)
        self.node.try_candidate()
        self.assertEqual(self.node.last_status['status'], 'no_vehicle_path')
        self.assertIsNone(self.node.last_status['goal'])

    def test_object_and_memory_goals_keep_trying_after_empty_paths(self):
        for attribute in ('expected', 'memory_goal'):
            with self.subTest(attribute):
                self._ready_for_failure_streak()
                setattr(self.node, attribute, (5., 5.))
                for _ in range(self.node.max_consecutive_empty_paths + 1):
                    self.node.candidate_failed('empty_path')
                self.node.block_current_route.assert_not_called()
                self.node.try_candidate.assert_called()
                setattr(self.node, attribute, None)


if __name__ == "__main__":
    unittest.main()
