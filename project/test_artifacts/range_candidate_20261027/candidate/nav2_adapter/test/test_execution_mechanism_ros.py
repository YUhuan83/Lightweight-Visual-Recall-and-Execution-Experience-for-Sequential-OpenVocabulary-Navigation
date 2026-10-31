"""Actual callbacks for freshness, action handover and terminal observation."""
from concurrent.futures import Future
from copy import deepcopy
import json
import math
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
from builtin_interfaces.msg import Time
from graphnav_msgs.msg import NavigationGraph, UUID, Node as GraphNode
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
from wildos_nav2_adapter.continuous_path import trim_moving_path
from test_graph_path_bridge_routing import configure_ackermann, pose, route, drivable_route


class ExecutionMechanismTests(unittest.TestCase):
    def setUp(self):
        self.node = configure_ackermann(GraphPathBridge())
        for name in ('execution_feedback_pub','execution_event_pub','route_rejection_pub'):
            setattr(self.node,name,Mock())
        self.now = 100 * 10**9
        self.node.get_clock = lambda: S(now=lambda: S(nanoseconds=self.now, to_msg=lambda:
            Time(sec=self.now//10**9, nanosec=self.now%10**9)))
        self.node.query = 'gas station'
        self.node.query_started_ns = self.now
        self.node.query_grace_until = 0.
        self.node.graph_reset_client = Mock()
        self.node.graph_search_reset_client = self.node.graph_reset_client
        self.node.graph_reset_client.service_is_ready.return_value = False
        odom = Odometry()
        odom.header.frame_id = 'spot1/odom'
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.
        self.node.on_odom(odom)
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/odom'
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.width = grid.info.height = 160
        grid.info.resolution = 1.
        grid.info.origin.position.x = grid.info.origin.position.y = -80.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 25600
        self.node.on_map(grid)
        self.node.on_local_map(deepcopy(grid))
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.follow_client.send_goal_async.side_effect = lambda _goal, **_kwargs: Future()
        self.node.follow_enabled = True
        self.node.route = self.node.latest_route = route((0., 0.), (40., 0.))
        self.node.route_endpoint = (40., 0.)
        self.node.expected = (40., 0.)

    def tearDown(self):
        self.node.destroy_node()

    def confirmation(self, query, object_position, observer_position, standoff):
        return String(data=json.dumps({
            'query': query,
            'observation_stamp_ns': self.now,
            'frame': 'spot1/odom',
            'search_confirmed': True,
            'localization_valid': True,
            'quality': {'confidence': .9, 'association_validated': True},
            'object_position': object_position,
            'observer_position': observer_position,
            'standoff': standoff,
            'radius': 0.5,
            'inliers': 3,
            'observations': 3,
        }))

    def test_live_target_arrival_waits_then_releases_unconfirmed_goal(self):
        self.move(40.)
        self.node.start_segment()
        self.assertEqual(self.node.last_status['status'], 'awaiting_visual_confirmation')
        self.assertFalse(self.node.object_found)
        self.node.observation_wait_until = time.monotonic()-1
        self.node.monitor_observation()
        self.assertIsNone(self.node.expected)
        self.assertIsNone(self.node.route)
        self.assertTrue(self.node.query_reset_pending)
        self.assertTrue(self.node.recently_unconfirmed((40., 0.)))
        self.assertFalse(self.node.object_found)

    def test_unconfirmed_goal_suppression_is_bounded_and_query_scoped(self):
        self.node.unconfirmed_approaches = [((40., 0.), time.monotonic()+60.)]
        self.node.expected = None
        msg = pose(41., 0.)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_waypoint(msg)
        self.assertIsNone(self.node.expected)
        msg.pose.position.x = 50.
        self.node.relay_destination = Mock()
        self.node.on_waypoint(msg)
        self.assertEqual(self.node.expected, (50., 0.))
        self.node.on_query(String(data='traffic light'))
        self.assertFalse(self.node.recently_unconfirmed((40., 0.)))
        self.node.unconfirmed_approaches = [((40., 0.), time.monotonic()-1)]
        self.assertFalse(self.node.recently_unconfirmed((40., 0.)))

    def test_live_waypoint_does_not_retarget_an_active_route_between_instances(self):
        self.node.expected = (10., 0.)
        self.node.following = True
        self.node.reset = Mock()
        self.node.relay_destination = Mock()
        msg = pose(30., 0.)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_waypoint(msg)
        self.assertEqual(self.node.expected, (10., 0.))
        self.node.reset.assert_not_called()
        self.node.relay_destination.assert_not_called()

    def test_nearest_supported_signal_preempts_farther_route_and_stays_selected(self):
        self.node.on_query(String(data='traffic light'))
        self.node.query_reset_pending = False
        # Start at the selected standoff so the expected visible-but-not-arrived
        # status tests object range, rather than the earlier route-approach state.
        self.node.odom.pose.pose.position.x = 14.5
        self.node.expected = (65., 0.)
        self.node.following = True
        self.node.reset = Mock()
        self.node.relay_destination = Mock()

        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[20., 0., 6.],
            observer_position=[0., 0., 0.], standoff=[15., 0.]))

        self.assertEqual(self.node.nearest_signal_position, (20., 0.))
        self.assertEqual(self.node.expected, (15., 0.))
        self.node.reset.assert_called_once()
        self.node.relay_destination.assert_called_once()
        self.assertEqual(self.node.last_status['status'], 'object_visible_not_arrived')

        # A later, farther light cannot replace the first nearest supported one.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[50., 0., 6.],
            observer_position=[0., 0., 0.], standoff=[45., 0.]))
        self.assertEqual(self.node.nearest_signal_position, (20., 0.))
        self.assertEqual(self.node.expected, (15., 0.))
        self.node.reset.assert_called_once()

        # A genuinely closer supported light may replace it.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[10., 0., 6.],
            observer_position=[0., 0., 0.], standoff=[5., 0.]))
        self.assertEqual(self.node.nearest_signal_position, (10., 0.))
        self.assertEqual(self.node.expected, (5., 0.))
        self.assertEqual(self.node.reset.call_count, 2)
        self.assertEqual(self.node.relay_destination.call_count, 2)

    def move(self, x, y=0.):
        self.now += 100000000
        odom = deepcopy(self.node.odom)
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        self.node.on_odom(odom)

    def test_republished_old_measurements_do_not_refresh_inputs(self):
        original = self.node.odom_received_at
        self.now += 2 * 10**9
        self.node.on_odom(deepcopy(self.node.odom))
        self.assertEqual(original, self.node.odom_received_at)
        self.node.odom_received_at = time.monotonic()-2.
        self.assertEqual(self.node.input_freshness_issue()[0], 'stale_odom')
        old_map = deepcopy(self.node.grid)
        old_map.header.stamp.sec = 80
        receipt = self.node.grid_received_at
        self.node.on_map(old_map)
        self.assertEqual(receipt, self.node.grid_received_at)

    def test_invalid_frame_nan_pose_and_malformed_map_do_not_replace_inputs(self):
        original = self.node.odom
        bad = deepcopy(original)
        bad.pose.pose.position.x = float('nan')
        self.node.on_odom(bad)
        self.assertIs(self.node.odom, original)
        bad_map = deepcopy(self.node.grid)
        bad_map.data.pop()
        self.node.on_map(bad_map)
        self.assertIsNot(self.node.grid, bad_map)

    def test_moving_path_pruning_never_skips_a_reverse_cusp(self):
        path = route((0., 0.), (1., 0.), (2., 0.), (1., 0.), (0., 0.))
        result = trim_moving_path(path, (1., 0.), 0.)
        self.assertEqual(len(result.poses), 4)
        self.assertEqual(result.poses[1].pose.position.x, 2.)
        self.assertIsNone(trim_moving_path(path, (10., 0.), 0.))
        self.assertIsNone(trim_moving_path(path, (1., 0.), math.pi))

    def test_forward_extension_is_requested_before_arrival(self):
        self.node.motion_queue = [route((0., 0.), (12., 0.))]
        self.node.follow_handle = Mock()
        self.node.following = True
        self.move(6.)
        self.node.start_segment = Mock()
        self.node.maybe_extend_path()
        self.node.start_segment.assert_called_once_with(continuous=True)

    def test_reverse_cusp_is_not_preempted_as_a_continuous_section(self):
        self.node.motion_queue = [route((6., 0.), (0., 0.))]
        self.node.follow_handle = Mock()
        self.node.following = True
        self.move(3.)
        self.node.start_segment = Mock()
        self.node.maybe_extend_path()
        self.node.start_segment.assert_not_called()

    def test_stale_or_future_routes_cannot_replace_current_route(self):
        current = self.node.latest_route
        for age in (6., -1.):
            bad = route((0., 0.), (30., 0.))
            stamp = self.now - int(age * 1e9)
            bad.header.stamp = Time(sec=stamp//10**9, nanosec=stamp%10**9)
            self.node.on_route_status(String(data=json.dumps(dict(
                stamp_ns=stamp, complete=True, goal=[40., 0.]))))
            self.node.on_route(bad)
            self.assertIs(self.node.latest_route, current)
            self.assertNotIn(stamp, self.node.route_metadata)

    def test_temporarily_blocked_extension_can_retry_without_cancelling_old_action(self):
        old = Mock()
        self.node.follow_handle, self.node.following = old, True
        self.node.motion_queue = [route((0., 0.), (12., 0.))]
        self.node.grid.data = [100] * len(self.node.grid.data)
        self.node.extending_path = True
        self.node.start_segment(continuous=True)
        self.assertFalse(self.node.extending_path)
        self.assertTrue(self.node.following)
        self.assertEqual(self.node.last_status['extension_rejected'], 'start_footprint_blocked')
        old.cancel_goal_async.assert_not_called()

    def test_failed_extension_preserves_current_action(self):
        old = Mock()
        self.node.following, self.node.follow_handle = True, old
        self.node.motion_queue = [route((0., 0.), (12., 0.))]
        self.node.extending_path = True
        self.node.candidates = []
        self.node.try_candidate()
        self.assertTrue(self.node.following)
        self.assertIs(self.node.follow_handle, old)
        old.cancel_goal_async.assert_not_called()

    def test_replacement_changes_active_action_only_after_acceptance_and_ignores_old_result(self):
        self.move(6.)
        old = Mock()
        old_queue = [route((6., 0.), (12., 0.))]
        self.node.motion_queue, self.node.follow_handle = old_queue, old
        self.node.following = True
        self.node.active_follow_serial = 1
        self.node.follow_request_serial = 1
        replacement = [drivable_route((6., 0.), (7., 0.), (24., 0.))]
        accepted = Future()
        self.node.follow_client.send_goal_async.side_effect = lambda goal, **_kwargs: accepted
        self.node.send_motion_segment(replacement=replacement)
        self.assertIs(self.node.motion_queue, old_queue)
        new = Mock(accepted=True)
        new.get_result_async.return_value = Future()
        accepted.set_result(new)
        self.assertIs(self.node.follow_handle, new)
        self.assertIs(self.node.motion_queue, replacement)
        self.node.on_follow_result(Mock(), self.node.epoch, (12., 0.), serial=1)
        self.assertIs(self.node.follow_handle, new)
        self.assertTrue(self.node.following)
        old.cancel_goal_async.assert_not_called()

    def test_old_action_cancelled_before_replacement_ack_cannot_cancel_new_request(self):
        self.node.motion_queue = [route((0., 0.), (12., 0.))]
        self.node.following = True
        self.node.follow_handle = Mock()
        self.node.active_follow_serial = self.node.follow_request_serial = 1
        accepted = Future()
        self.node.follow_client.send_goal_async.side_effect = lambda goal, **_kwargs: accepted
        replacement = [drivable_route((0., 0.), (24., 0.))]
        self.node.send_motion_segment(replacement=replacement)
        epoch = self.node.epoch
        cancelled = Mock()
        cancelled.result.return_value.status = 5
        self.node.on_follow_result(cancelled, epoch, (12., 0.), serial=1)
        self.assertEqual(self.node.epoch, epoch)
        self.assertIsNotNone(self.node.deferred_follow_result)
        new = Mock(accepted=True)
        new.get_result_async.return_value = Future()
        accepted.set_result(new)
        self.assertTrue(self.node.following)
        self.assertIs(self.node.follow_handle, new)
        self.assertEqual(self.node.active_follow_serial, 2)
        self.assertIsNone(self.node.deferred_follow_result)

    def test_rejected_replacement_processes_old_goal_result_instead_of_assuming_it_is_running(self):
        self.node.motion_queue = [route((0., 0.), (12., 0.))]
        self.node.following = True
        self.node.follow_handle = Mock()
        self.node.active_follow_serial = self.node.follow_request_serial = 1
        accepted = Future()
        self.node.follow_client.send_goal_async.side_effect = lambda goal, **_kwargs: accepted
        self.node.send_motion_segment(replacement=[route((0., 0.), (24., 0.))])
        cancelled = Mock()
        cancelled.result.return_value.status = 5
        self.node.on_follow_result(cancelled, self.node.epoch, (12., 0.), serial=1)
        accepted.set_result(Mock(accepted=False))
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.last_status['status'], 'follow_failed')

    def test_unconfirmed_passive_region_releases_goal_when_no_observation_approach_remains(self):
        view = dict(id='passive:gas station:1', evidence='passive_semantic_landmarks',
                    frame='spot1/odom', position=[0., 0., 0.], heading=0.,
                    object_position=[10., 0., 0.], object_radius=1.)
        self.node.memory_active = view
        self.node.memory_source_views = [view]
        self.node.memory_views = [view]
        self.node.memory_selecting = False
        self.node.following = False
        self.node.memory_goal = self.node.expected = (0., 0.)
        self.node.observation_wait_until = time.monotonic()-1
        self.node.grid.data = [100]*len(self.node.grid.data)
        self.node.memory_feedback_pub = Mock()
        self.node.begin_memory_selection = Mock()
        self.node.monitor_observation()
        self.node.begin_memory_selection.assert_called_once_with([view], {'passive:gas station:1'})
        feedback = json.loads(self.node.memory_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['outcome'], 'unconfirmed')

    def test_disabled_or_cancelled_action_never_produces_training_feedback(self):
        self.node.start_execution_feedback(route((0., 0.), (4., 0.)))
        self.move(1.)
        self.node.finish_execution_feedback(True, 'no_actuation')
        self.node.execution_feedback_pub.publish.assert_not_called()
        data = json.loads(self.node.execution_event_pub.publish.call_args.args[0].data)
        self.assertFalse(data['learning_eligible'])
        self.node.start_execution_feedback(route((1., 0.), (4., 0.)))
        self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now, actuating=True))))
        self.move(2.)
        self.node.reset()
        self.node.execution_feedback_pub.publish.assert_not_called()

    def test_success_feedback_requires_actual_motion_and_node_anchors(self):
        graph = NavigationGraph()
        graph.header.frame_id = 'spot1/odom'
        graph.header.stamp = self.node.get_clock().now().to_msg()
        graph.nodes = [GraphNode(pose=pose(x, 0.).pose, uuid=UUID(id=[n]+[0]*15)) for n, x in ((1, 0.), (2, 4.))]
        self.node.on_feedback_graph(graph)
        self.node.start_execution_feedback(route((0., 0.), (4., 0.)))
        self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now, actuating=True))))
        for x in (1., 2., 3., 4.):
            self.move(x)
        self.node.finish_execution_feedback(True, 'segment_completed')
        msg = self.node.execution_feedback_pub.publish.call_args.args[0]
        self.assertEqual(msg.from_node.id[0], 1)
        self.assertEqual(msg.to_node.id[0], 2)
        self.assertAlmostEqual(msg.observed_distance, 4.)
        self.assertEqual(msg.outcome, 1)
        self.assertEqual(list(msg.observations[0].value), [1.])

    def feedback_graph(self):
        graph = NavigationGraph()
        graph.header.frame_id = 'spot1/odom'
        graph.header.stamp = self.node.get_clock().now().to_msg()
        graph.nodes = [GraphNode(pose=pose(x, 0.).pose, uuid=UUID(id=[n]+[0]*15))
                       for n, x in ((1, 0.), (2, 4.), (3, 12.))]
        self.node.on_feedback_graph(graph)

    def test_path_extension_learns_only_the_reached_prefix(self):
        self.feedback_graph()
        self.node.start_execution_feedback(route((0., 0.), (12., 0.)))
        self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now, actuating=True))))
        for x in (1., 2., 3., 4.):
            self.move(x)
        self.node.checkpoint_execution_feedback(route((4., 0.), (20., 0.)))
        msg = self.node.execution_feedback_pub.publish.call_args.args[0]
        self.assertEqual(msg.to_node.id[0], 2)
        self.assertAlmostEqual(msg.commanded_distance, 4.)
        self.assertAlmostEqual(msg.observed_distance, 4.)
        self.assertEqual(list(msg.observations[-1].value), [1.])
        self.assertEqual(self.node.execution_attempt['start'].pose.pose.position.x, 4.)

    def test_extension_between_graph_nodes_keeps_evidence_and_distance(self):
        self.feedback_graph()
        self.node.start_execution_feedback(route((0., 0.), (12., 0.)))
        self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now, actuating=True))))
        self.move(1.5)
        attempt = self.node.execution_attempt
        self.node.checkpoint_execution_feedback(route((1.5, 0.), (20., 0.)))
        self.assertIs(self.node.execution_attempt, attempt)
        self.node.execution_feedback_pub.publish.assert_not_called()
        for x in (2.5, 3.5, 4.):
            self.move(x)
        self.node.checkpoint_execution_feedback(route((4., 0.), (22., 0.)))
        msg = self.node.execution_feedback_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.commanded_distance, 4.)
        self.assertAlmostEqual(msg.observed_distance, 4.)

    def test_extension_without_actuation_does_not_learn_a_prefix(self):
        self.feedback_graph()
        self.node.start_execution_feedback(route((0., 0.), (12., 0.)))
        for x in (1., 2., 3., 4.):
            self.move(x)
        self.node.checkpoint_execution_feedback(route((4., 0.), (20., 0.)))
        self.node.execution_feedback_pub.publish.assert_not_called()

    def test_control_wait_between_monitor_ticks_cannot_age_the_stall_anchor(self):
        self.node.following=True
        self.node.motion_queue=[route((0.,0.),(4.,0.))]
        with patch('time.monotonic',return_value=10.):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=True))))
            self.assertIsNone(self.node.execution_stall())
        # Input/costmap recovery can skip monitor execution during this wait.
        with patch('time.monotonic',return_value=14.):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=False))))
        with patch('time.monotonic',return_value=21.):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=True))))
            self.assertIsNone(self.node.execution_stall())
            self.assertEqual(self.node.stall_anchor[2],21.)

    def test_native_full_brake_is_not_continuous_physical_actuation(self):
        self.node.following=True
        self.node.motion_queue=[route((0.,0.),(4.,0.))]
        with patch('time.monotonic',return_value=10.):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=True))))
            self.assertIsNone(self.node.execution_stall())
        with patch('time.monotonic',return_value=21.),patch('time.time',return_value=1000.):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=True))))
            self.node.on_native_status(String(data=json.dumps(dict(wall_time=1000.,speed=0.,
                telemetry_ok=True,actuator_ok=True,control_acknowledged=True,
                telemetry_age_sec=0.,watchdog_stop=False,actual={'gear':'D'},
                sent={'throttle':0.,'brake':1.,'gear':2},wheel_contact=None))))
            self.assertIsNone(self.node.execution_stall())
            self.assertIsNone(self.node.stall_anchor)

    def test_late_control_wait_cannot_replace_a_newer_actuation_lease(self):
        self.node.following=True;self.node.motion_queue=[route((0.,0.),(4.,0.))]
        with patch('time.monotonic',return_value=10.):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=True))))
            self.node.execution_stall();anchor=self.node.stall_anchor
        with patch('time.monotonic',return_value=10.2):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now-100000000,actuating=False))))
        self.assertTrue(self.node.control_status['actuating'])
        self.assertEqual(self.node.stall_anchor,anchor)

    def test_late_native_brake_cannot_clear_a_newer_command_window(self):
        self.node.following=True;self.node.motion_queue=[route((0.,0.),(4.,0.))]
        with patch('time.monotonic',return_value=10.),patch('time.time',return_value=1000.5):
            self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now,actuating=True))))
            self.node.on_native_status(String(data=json.dumps(dict(wall_time=1000.5,speed=0.,
                actual={},sent={'throttle':.3,'brake':0.},wheel_contact=None))))
            self.node.execution_stall();anchor=self.node.stall_anchor;native=self.node.native_status
        with patch('time.monotonic',return_value=10.2),patch('time.time',return_value=1000.6):
            self.node.on_native_status(String(data=json.dumps(dict(wall_time=1000.4,speed=0.,
                actual={},sent={'throttle':0.,'brake':1.},wheel_contact=None))))
        self.assertIs(self.node.native_status,native)
        self.assertEqual(self.node.stall_anchor,anchor)

    def test_stall_timer_ignores_gear_changes_and_resets_on_progress(self):
        self.node.following = True
        self.node.control_status = {'actuating': True}
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1000.):
            self.node.control_status_at = 1000.
            self.assertIsNone(self.node.execution_stall())
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1006.):
            self.node.control_status_at = 1006.
            self.assertEqual(self.node.execution_stall()['stall_class'], 'actuated_no_path_progress')
            self.node.control_status['actuating'] = False
            self.assertIsNone(self.node.execution_stall())
            self.node.control_status['actuating'] = True
            self.assertIsNone(self.node.execution_stall())
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1010.):
            self.node.odom.pose.pose.position.x = .25
            self.node.control_status_at = 1010.
            self.assertIsNone(self.node.execution_stall())

    def test_stall_classifies_stale_beamng_rpc_before_generic_watchdog(self):
        self.node.following = True
        self.node.control_status = {'actuating': True}
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1000.):
            self.node.control_status_at = 1000.
            self.assertIsNone(self.node.execution_stall())
        self.node.native_status = dict(telemetry_ok=False, telemetry_age_sec=2.1,
            watchdog_stop=True, speed=0., actual={}, wheel_contact=None)
        self.node.native_status_at = 1006.
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1006.):
            self.node.control_status_at = 1006.
            self.assertEqual(self.node.execution_stall()['stall_class'], 'native_telemetry_stale')

    def test_stall_timer_does_not_reset_on_lateral_oscillation(self):
        self.node.following = True
        self.node.control_status = {'actuating': True}
        self.node.motion_queue = [route((0., 0.), (10., 0.))]
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1000.):
            self.node.control_status_at = 1000.
            self.assertIsNone(self.node.execution_stall())
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1002.):
            self.node.odom.pose.pose.position.y = .25
            self.node.control_status_at = 1002.
            self.assertIsNone(self.node.execution_stall())
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1004.):
            self.node.odom.pose.pose.position.x = .1
            self.node.odom.pose.pose.position.y = -.25
            self.node.control_status_at = 1004.
            self.assertIsNone(self.node.execution_stall())
        with patch('wildos_nav2_adapter.execution_feedback.time.monotonic', return_value=1005.1):
            self.node.odom.pose.pose.position.x = .02
            self.node.odom.pose.pose.position.y = .1
            self.node.control_status_at = 1005.1
            self.assertEqual(self.node.execution_stall()['stall_class'], 'actuated_no_path_progress')

    def test_stall_monitor_cancels_without_erasing_the_memory_goal(self):
        self.node.following = True
        self.node.motion_queue = [route((0., 0.), (12., 0.))]
        handle = self.node.follow_handle = Mock()
        self.node.memory_goal = (40., 0.)
        with patch.object(self.node, 'execution_stall', return_value=dict(
                stall_class='wheel_airborne_spin', no_progress_sec=5.1)):
            self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.node.last_status['reason'], 'execution_stalled')
        self.assertEqual(self.node.memory_goal, (40., 0.))

    def test_new_odometry_guard_rejects_previous_half_second_future_budget(self):
        old = self.node.odom
        future = deepcopy(old)
        future.header.stamp.nanosec = 200_000_000
        self.node.on_odom(future)
        self.assertIs(self.node.odom, old)

    def test_confirmed_support_loss_postpones_exploration_with_recent_matching_route(self):
        self.node.expected = None
        self.node.latest_route = deepcopy(self.node.route)
        self.node.latest_route.header.stamp.sec += 1
        self.node.follow_failed('execution_stalled', stall_class='wheel_airborne_spin')
        sent = self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(sent.header.stamp.sec, self.node.latest_route.header.stamp.sec)
        self.assertEqual(sent.pose.position.x, 40.)

    def test_reverse_feedback_supervises_travel_bearing_instead_of_body_heading(self):
        graph = NavigationGraph()
        graph.header.frame_id = 'spot1/odom'
        graph.header.stamp = self.node.get_clock().now().to_msg()
        graph.nodes = [GraphNode(pose=pose(x, 0.).pose, uuid=UUID(id=[n]+[0]*15)) for n, x in ((1, 0.), (2, -4.))]
        self.node.on_feedback_graph(graph)
        self.node.start_execution_feedback(route((0., 0.), (-4., 0.)))
        self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now, actuating=True))))
        for x in (-1., -2., -3., -4.):
            self.move(x)
        self.node.finish_execution_feedback(True, 'reverse_completed')
        msg = self.node.execution_feedback_pub.publish.call_args.args[0]
        self.assertAlmostEqual(abs(msg.commanded_heading), math.pi)
        self.assertAlmostEqual(msg.observed_heading, 0.)

    def test_feedback_keeps_starting_node_anchors_without_copying_unused_edges(self):
        class UnusedEdges(list):
            def __deepcopy__(self, memo):
                raise AssertionError('Unused graph edges must not block action acceptance')

        graph = S(header=deepcopy(self.node.odom.header),
                  nodes=[GraphNode(pose=pose(x, 0.).pose, uuid=UUID(id=[n]+[0]*15))
                         for n, x in ((1, 0.), (2, 4.))], edges=UnusedEdges())
        self.node.on_feedback_graph(graph)
        self.node.start_execution_feedback(route((0., 0.), (4., 0.)))
        replacement = NavigationGraph()
        replacement.header = deepcopy(graph.header)
        replacement.nodes = [GraphNode(pose=pose(x, 0.).pose, uuid=UUID(id=[n]+[0]*15))
                             for n, x in ((3, 0.), (4, 4.))]
        self.node.on_feedback_graph(replacement)
        self.node.on_control_status(String(data=json.dumps(dict(stamp_ns=self.now, actuating=True))))
        for x in (1., 2., 3., 4.):
            self.move(x)
        self.node.finish_execution_feedback(True, 'segment_completed')
        msg = self.node.execution_feedback_pub.publish.call_args.args[0]
        self.assertEqual(msg.from_node.id[0], 1)
        self.assertEqual(msg.to_node.id[0], 2)

    def test_arrival_without_new_visual_confirmation_starts_an_alternative_approach(self):
        self.node.memory_goal = (0., 0.)
        self.node.expected = (0., 0.)
        self.node.memory_active = dict(id='1', frame='spot1/odom', position=[0., 0., 0.],
            object_position=[10., 0., 0.], heading=0., object_radius=.5, horizontal_fov=1.4)
        self.node.await_object_observation()
        self.node.observation_wait_until = time.monotonic()-1
        self.node.monitor_observation()
        self.assertTrue(self.node.observation_active)
        self.assertNotEqual(self.node.expected, self.node.memory_goal)
        self.assertEqual(self.node.last_status['status'], 'memory_observation_approach')
        headings = self.node.observation_headings(self.node.expected)
        target = self.node.memory_active['object_position']
        desired = math.atan2(target[1]-self.node.expected[1], target[0]-self.node.expected[0])
        self.assertTrue(all(abs(h-desired) <= .35+1e-8 for h in headings))

    def test_observation_exhaustion_keeps_object_and_does_not_restart_frontiers(self):
        self.node.memory_goal = self.node.expected = (0., 0.)
        self.node.memory_active = dict(id='1', frame='spot1/odom', position=[0., 0., 0.],
            object_position=[10., 0., 0.], heading=0.)
        for _ in range(10):
            self.node.observation_wait_until = time.monotonic()-1
            self.node.monitor_observation()
        self.assertEqual(self.node.last_status['status'], 'memory_observation_exhausted')
        self.assertEqual(self.node.memory_goal, (0., 0.))
        self.assertLessEqual(len(self.node.observation_attempts), 6)

    def test_blocked_old_observer_is_not_the_only_possible_memory_approach(self):
        self.node.memory_goal = self.node.expected = (0., 0.)
        self.node.memory_active = dict(id='1', frame='spot1/odom', position=[0., 0., 0.],
            object_position=[10., 0., 0.], heading=0.)
        self.node.memory_candidates_exhausted()
        self.node.observation_wait_until = time.monotonic()-1
        self.node.monitor_observation()
        self.assertNotEqual(self.node.expected, (0., 0.))
        self.assertEqual(self.node.memory_goal, (0., 0.))
        self.assertEqual(self.node.last_status['status'], 'memory_observation_approach')

    def test_infeasible_curvature_is_reported_without_a_speed_ceiling(self):
        from geometry_msgs.msg import Twist
        node = CmdVelToControl()
        try:
            node.control_status_pub=Mock()
            node.enabled = True
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = 4., 2.
            node.on_cmd_vel(cmd)
            self.assertEqual(node.target_speed, 4.)
            self.assertAlmostEqual(node.target_angular, 4./node.physical_turning_radius)
            node.last_state_time = node.last_command_time
            node.native_control_ok = True
            node.native_control_at = node._safety_clock.now().nanoseconds * 1e-9
            node.tick()
            data = json.loads(node.control_status_pub.publish.call_args.args[0].data)
            self.assertTrue(data['curvature_infeasible'])
            self.assertLessEqual(abs(data['applied_curvature']), 1/node.physical_turning_radius + 1e-9)
        finally:
            node.destroy_node()

    def test_limited_steering_keeps_a_path_with_small_measured_tracking_error(self):
        handle = Mock()
        self.node.following, self.node.follow_handle = True, handle
        self.node.motion_queue = [route((0., 0.), (20., 0.))]
        self.node.maybe_extend_path = Mock()
        base = time.monotonic()
        for step in range(4):
            with patch('time.monotonic', return_value=base+step):
                self.move(float(step), .15)
                self.node.grid_received_at = base+step
                self.node.local_grid_received_at = base+step
                self.node.native_status_at = base+step
                self.node.on_control_status(String(data=json.dumps(dict(
                    stamp_ns=self.now, actuating=True, curvature_infeasible=True,
                    raw_curvature=.19, applied_curvature=.125))))
                self.node.monitor_inputs()
        handle.cancel_goal_async.assert_not_called()
        self.assertTrue(self.node.following)

    def test_stale_local_costmap_cancels_follow_without_erasing_memory(self):
        handle = Mock()
        self.node.following, self.node.follow_handle = True, handle
        self.node.memory_goal = (40., 0.)
        self.node.local_grid_received_at = time.monotonic()-2.
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.memory_goal, (40., 0.))
        self.assertEqual(self.node.last_status['status'], 'stale_local_costmap')

    def test_limited_steering_still_stops_an_actual_tracking_deviation(self):
        handle = Mock()
        self.node.following, self.node.follow_handle = True, handle
        self.node.motion_queue = [route((0., 0.), (20., 0.))]
        self.move(1., 1.5)
        self.node.on_control_status(String(data=json.dumps(dict(
            stamp_ns=self.now, actuating=True, curvature_infeasible=True))))
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.last_status['reason'], 'tracking_deviation')

    def test_candidate_search_yields_to_new_costmap_without_changing_destination(self):
        self.node.candidates = [(20.,0.,0.,20.)]
        self.node.grid_received_at = time.monotonic()-1.2
        self.node.grid_arrived_at = time.monotonic()-1.2
        self.node.plan_client.send_goal_async = Mock(return_value=Future())
        self.node.try_candidate()
        self.node.plan_client.send_goal_async.assert_not_called()
        self.assertTrue(self.node.pending)
        self.assertTrue(self.node.candidate_waiting_for_map)
        self.now += 100000000
        updated = deepcopy(self.node.grid)
        updated.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_map(updated)
        self.node.plan_client.send_goal_async.assert_called_once()
        goal = self.node.plan_client.send_goal_async.call_args.args[0]
        self.assertEqual((goal.goal.pose.position.x,goal.goal.pose.position.y),(20.,0.))
        self.assertFalse(self.node.candidate_waiting_for_map)

    def test_query_change_discards_search_waiting_for_costmap(self):
        self.node.candidates = [(20.,0.,0.,20.)]
        self.node.grid_received_at = time.monotonic()-1.2
        self.node.grid_arrived_at = time.monotonic()-1.2
        self.node.plan_client.send_goal_async = Mock(return_value=Future())
        self.node.try_candidate()
        self.node.on_query(String(data='house'))
        self.now += 100000000
        updated = deepcopy(self.node.grid)
        updated.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_map(updated)
        self.node.plan_client.send_goal_async.assert_not_called()
        self.assertFalse(self.node.candidate_waiting_for_map)

    def test_cusp_offset_rechecks_actual_footprint_before_next_direction(self):
        self.move(.28)
        grid = self.node.grid
        grid.info.resolution = .25
        grid.info.origin.position.x = grid.info.origin.position.y = -20.
        grid.data[80*160+94] = 100  # Actual nose extends into this cell; planned nose does not.
        self.node.motion_queue = [route((0.,0.),(-4.,0.))]
        self.node.send_motion_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status['reason'],'motion_section_invalid')
        self.assertEqual(self.node.last_status['detail'],'start_footprint_blocked')
