"""Confirmed approach failure ownership, real callbacks; no vehicle simulation."""
from concurrent.futures import Future
from copy import deepcopy
import time
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
from nav_msgs.msg import Path
from std_msgs.msg import String
import json
from beamng_wildos_adapter.goal_standoff import GoalStandoff
import test_execution_mechanism_ros as fixture
from test_graph_path_bridge_routing import route


class ConfirmedRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.ExecutionMechanismTests()
        self.fixture.setUp()
        self.node = n = self.fixture.node
        n.waypoint_pub = Mock()
        n.instance_release_pub = Mock()
        n.plan_client = Mock()
        n.plan_client.wait_for_server.return_value = True
        n.execution_transport_issue = Mock(return_value=None)
        self.data = dict(query=n.query, object_position=[20., 0., 2.], radius=.5,
                         standoff=[16., 0.], observation_stamp_ns=self.fixture.now,
                         instance_id='target-a')
        n.expected = n.confirmed_route_goal = (16., 0.)
        n.confirmed_route_object = (20., 0.)
        n.confirmed_route_radius = .5
        n.confirmed_route_started_at = time.monotonic()-11
        n.active_object_confirmation = dict(self.data)
        n.route = n.latest_route = route((0., 0.), (16., 0.))
        n.route_endpoint = (16., 0.)

    def tearDown(self):
        self.fixture.tearDown()

    def exhaust_plans(self, reasons=('empty_path', 'footprint_collision')):
        n = self.node
        n.pending = n.following = False
        n.candidates = [(16., 0., 0., 16.)]*len(reasons)
        n.candidate_index = len(reasons)
        n.failure_reasons = list(reasons)
        n.try_candidate()

    def test_all_vehicle_plans_failed_enters_observation_instead_of_old_backoff(self):
        self.exhaust_plans()
        self.assertIsNotNone(self.node.observation_wait_until)
        self.assertIsNone(self.node.route)
        self.node.monitor_observation()
        self.assertNotEqual(self.node.expected, (16., 0.))
        self.assertEqual(self.node.last_status['status'], 'target_observation_approach')
        self.assertFalse(self.node.object_found)

    def test_empty_action_result_reaches_failure_handoff(self):
        n = self.node
        n.candidates = [(16., 0., 0., 16.)]
        n.candidate_index = 0
        n.pending = True
        f = Future(); f.set_result(S(status=6, result=S(path=Path())))
        n.on_plan_result(f, n.epoch, 0)
        self.assertIsNotNone(n.observation_wait_until)
        self.assertFalse(n.pending)
        self.assertIsNone(n.route)

    def test_sideways_deferred_same_instance_view_is_tried_after_failure(self):
        n = self.node
        n.deferred_confirmed_approach = dict(self.data, standoff=[14., 4.])
        self.exhaust_plans()
        n.monitor_observation()
        self.assertEqual(n.expected, (14., 4.))
        self.assertTrue(n.observation_active)
        self.assertFalse(n.object_found)

    def test_failed_point_refresh_cannot_resurrect_old_goal(self):
        n = self.node
        self.exhaust_plans()
        n.route_confirmed_target(self.data, (0., 0.))
        n.monitor_observation()
        alternative = n.expected
        n.route_confirmed_target(self.data, (0., 0.))
        self.assertEqual(n.expected, alternative)
        self.assertNotEqual(alternative, (16., 0.))

    def test_retry_budget_exhaustion_hands_explicit_goal_to_observation(self):
        n = self.node
        n.recovery_attempts = 3
        n.blocked_endpoint = n.expected
        n.blocked_until = time.monotonic()-1
        n.last_status = dict(status='follow_failed')
        n.retry_when_planner_starts()
        self.assertIsNotNone(n.observation_wait_until)
        self.assertIsNone(n.route)

    def test_repeated_controller_abort_retires_waypoint_not_target_identity(self):
        n = self.node
        n.recovery_attempts = 2
        n.follow_failed('follow_aborted', action_status=6)
        self.assertIsNotNone(n.observation_wait_until)
        self.assertEqual(n.active_object_confirmation['instance_id'], 'target-a')
        self.assertIsNone(n.instance_release_pending)

    def test_no_graph_route_has_bounded_exit_with_fresh_inputs(self):
        n = self.node
        n.route = n.latest_route = None
        n.monitor_observation()
        self.assertNotEqual(n.expected, (16., 0.))
        self.assertEqual(n.last_status['status'], 'target_observation_approach')

    def test_observation_timeout_does_not_cancel_measured_escape(self):
        n = self.node
        n.recovering = True
        n.live_observation_deadline = time.monotonic()-1
        epoch = n.epoch
        n.monitor_observation()
        self.assertEqual(n.epoch, epoch)
        self.assertTrue(n.recovering)

    def test_bounded_failed_alternatives_release_before_resuming_exploration(self):
        n = self.node
        self.exhaust_plans()
        for _ in range(6):
            n.monitor_observation()
            if n.instance_release_pending is not None:
                break
            n.route = n.latest_route = route((0., 0.), n.expected)
            n.route_endpoint = n.expected
            self.exhaust_plans()
        self.assertIsNotNone(n.instance_release_pending)
        self.assertTrue(n.query_reset_pending)
        self.assertIsNone(n.expected)
        self.assertFalse(n.object_found)
        n.graph_reset_client.call_async.assert_not_called()

    def test_planner_protocol_failure_does_not_mark_viewpoint_unreachable(self):
        self.exhaust_plans(('planner_rejected',))
        self.assertIsNone(self.node.observation_wait_until)
        self.assertIsNotNone(self.node.route)

    def test_recovery_does_not_transfer_motion_before_old_terminal(self):
        n = self.node
        n.follow_quarantine = {7: dict(handle=None, terminal=False)}
        self.exhaust_plans()
        n.monitor_observation()
        self.assertEqual(n.expected, (16., 0.))
        n.waypoint_pub.publish.assert_not_called()
        n.follow_quarantine[7]['terminal'] = True
        n.monitor_observation()
        n.waypoint_pub.publish.assert_not_called()  # Standstill barrier still owns motion.
        n.follow_quarantine.clear()
        n.monitor_observation()
        self.assertNotEqual(n.expected, (16., 0.))

    def test_stale_inputs_pause_alternative_and_old_route_cannot_resume(self):
        n = self.node
        self.exhaust_plans()
        n.grid_received_at = time.monotonic()-100
        n.monitor_observation()
        n.on_route(route((0., 0.), (16., 0.)))
        self.assertIsNone(n.route)
        n.waypoint_pub.publish.assert_not_called()

    def test_occupied_deferred_endpoint_is_not_dispatched(self):
        n = self.node
        n.deferred_confirmed_approach = dict(self.data, standoff=[14., 4.])
        n.pose_blockers = Mock(side_effect=lambda _g, p, _h: ['occupied'] if p == (14., 4.) else [])
        self.exhaust_plans()
        n.monitor_observation()
        self.assertNotEqual(n.expected, (14., 4.))
        self.assertEqual(n.last_status['status'], 'target_observation_approach')

    def test_other_instance_deferred_point_cannot_be_used(self):
        n = self.node
        n.deferred_confirmed_approach = dict(self.data, object_position=[45., 0., 2.], standoff=[40., 0.])
        self.exhaust_plans()
        n.monitor_observation()
        self.assertNotEqual(n.expected, (40., 0.))
        self.assertEqual(n.confirmed_route_object, (20., 0.))

    def test_query_change_clears_failed_views_and_old_plan_result_cannot_return(self):
        n = self.node
        old_epoch = n.epoch
        self.exhaust_plans()
        n.on_query(String(data='bus stop'))
        f = Future(); f.set_result(S(status=6, result=S(path=Path())))
        n.on_plan_result(f, old_epoch, 0)
        self.assertIsNone(n.expected)
        self.assertEqual(n.failed_confirmed_goals, [])
        self.assertIsNone(n.confirmed_approach_failure)

    def test_release_cools_failed_region_without_blocking_other_objects(self):
        n = self.node
        g = GoalStandoff()
        try:
            g.get_clock = n.get_clock
            g.on_query(String(data=n.query))
            g.confirmed_instance_anchor = dict(self.data)
            g.instance_ack_pub = Mock()
            self.exhaust_plans()
            n.observation_attempts = {(i, i) for i in range(4)}
            n.monitor_observation()
            request = n.instance_release_pub.publish.call_args.args[0]
            g.on_instance_release(request)
            self.fixture.now += 100_000_000
            def locate(x):
                self.fixture.now += 100_000_000
                g.on_localization(String(data=json.dumps(dict(query=n.query, frame='spot1/odom',
                    observation_stamp_ns=self.fixture.now, localization_valid=True, source='lidar',
                    position=[x, 0., 2.], observer_position=[0., 0., 0.],
                    observer_orientation=[0., 0., 0., 1.],
                    quality=dict(confidence=.9, association_validated=True)))))
            locate(20.)
            self.assertEqual(len(g.observations), 0)
            locate(40.)
            self.assertEqual(len(g.observations), 1)
            self.assertEqual(g.observations[-1]['position'][0], 40.)
            deadline = g.navigation_region_backoff[0]['until']
            g.on_instance_release(request)
            self.assertEqual(g.navigation_region_backoff[0]['until'], deadline)
            g.observations.clear()
            with patch('time.monotonic', return_value=deadline+1):
                locate(20.)
            self.assertEqual(len(g.observations), 1)  # Temporary navigation backoff expired.
            g.on_query(String(data='new target'))
            self.assertFalse(g.navigation_region_backoff)
        finally:
            g.destroy_node()

    def test_healthy_route_keeps_deferred_update_without_cancellation(self):
        n = self.node
        n.following = True
        epoch = n.epoch
        n.route_confirmed_target(dict(self.data, standoff=[14., 4.]), (0., 0.))
        self.assertEqual(n.expected, (16., 0.))
        self.assertEqual(n.epoch, epoch)
        self.assertTrue(n.following)
        n.waypoint_pub.publish.assert_not_called()

    def test_no_route_timeout_does_not_advance_on_stale_map(self):
        n = self.node
        n.route = None
        n.grid_received_at = time.monotonic()-100.
        n.monitor_observation()
        self.assertIsNone(n.confirmed_approach_failure)
        self.assertEqual(n.expected, (16., 0.))


if __name__ == '__main__':
    unittest.main()
