"""Recovery credit belongs to a navigation destination, across local actions."""
from copy import deepcopy
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

import test_execution_mechanism_ros as fixtures
from test_graph_path_bridge_routing import drivable_route
from wildos_nav2_adapter.input_quality import stamp_ns


class ExecutionProgressTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.ExecutionMechanismTests()
        self.fixture.setUp()
        self.node = n = self.fixture.node
        n.expected = None
        self.set_route('frontier:1', 10.)
        n.try_candidate = Mock()
        n.plan_client = Mock()
        n.start_segment()
        n.pending = False
        n.send_motion_segment = Mock()
        n.finish_execution_feedback = Mock()
        n.start_segment = Mock()

    def tearDown(self):
        self.fixture.tearDown()

    def set_route(self, identity, endpoint, complete=True):
        n = self.node
        n.route = n.latest_route = drivable_route((0., 0.), (endpoint, 0.))
        n.route.header.stamp = n.get_clock().now().to_msg()
        n.route_endpoint = (endpoint, 0.)
        n.route_complete = n.latest_route_complete = complete
        n.route_metadata[stamp_ns(n.route)] = dict(
            exploration_id=identity, goal=[10., 0.], complete=complete)

    def succeed(self, start, end, cusp=True, epoch=None):
        n = self.node
        n.motion_queue = [drivable_route((start, 0.), (end, 0.))]
        if cusp:
            n.motion_queue.append(drivable_route((end, 0.), (start, 0.)))
        n.odom.pose.pose.position.x = end
        n.on_follow_result(Mock(result=lambda: S(status=4)),
                           n.epoch if epoch is None else epoch, (end, 0.))

    def test_repeated_local_advance_cannot_recredit_same_ground(self):
        n = self.node
        n.recovery_attempts = 3
        self.succeed(0., 1.)
        self.assertEqual(n.recovery_attempts, 0)
        for attempt in (1, 2, 3):
            n.recovery_attempts = attempt
            self.succeed(1., 0.)
            self.succeed(0., 1.)
            self.assertEqual(n.recovery_attempts, attempt)

    def test_completed_prefix_is_not_new_destination_progress(self):
        self.succeed(0., 1.)
        self.node.recovery_attempts = 3
        self.succeed(0., 1., cusp=False)
        self.assertEqual(self.node.recovery_attempts, 3)

    def test_small_successes_accumulate_one_metre_of_new_progress(self):
        n = self.node
        n.recovery_attempts = 2
        self.succeed(0., .5)
        self.assertEqual(n.recovery_attempts, 2)
        self.succeed(.5, 1.)
        self.assertEqual(n.recovery_attempts, 0)
        n.recovery_attempts = 2
        self.succeed(1., .5)
        self.succeed(.5, 1.)
        self.assertEqual(n.recovery_attempts, 2)

    def test_detour_can_move_away_then_earn_genuinely_new_progress(self):
        n = self.node
        n.recovery_attempts = 2
        self.succeed(0., -2.)
        self.assertEqual(n.recovery_attempts, 2)
        self.succeed(-2., 2.)
        self.assertEqual(n.recovery_attempts, 0)

    def test_same_owner_partial_endpoint_change_keeps_budget(self):
        n = self.node
        old = deepcopy(n.route)
        self.set_route('frontier:1', 16., complete=False)
        incoming = n.route
        n.route = old
        n.route_endpoint = (10., 0.)
        n.recovery_attempts = 3
        n.last_plan_start = None
        n.on_route(incoming)
        self.assertEqual(n.recovery_attempts, 3)
        n.start_segment.assert_not_called()
        self.assertEqual(n.last_status['status'], 'route_recovery_exhausted')

    def test_changed_map_cannot_reopen_exhausted_same_owner(self):
        n = self.node
        n.block_current_route(backoff=-1.)
        n.recovery_attempts = 3
        n.grid.data[0] = 100
        n.on_route(n.route)
        self.assertEqual(n.recovery_attempts, 3)
        n.start_segment.assert_not_called()
        self.assertEqual(n.last_status['status'], 'route_recovery_exhausted')

    def test_different_polyline_to_same_owner_keeps_budget(self):
        n = self.node
        n.block_current_route()
        n.recovery_attempts = 3
        incoming = drivable_route((0., 0.), (4., 5.), (10., 0.))
        incoming.header.stamp = n.get_clock().now().to_msg()
        n.route_metadata[stamp_ns(incoming)] = dict(
            exploration_id='frontier:1', goal=[10., 0.], complete=True)
        n.on_route(incoming)
        self.assertEqual(n.recovery_attempts, 3)

    def test_changed_polylines_consume_same_owner_retry_budget(self):
        n = self.node
        n.recovery_attempts = 0
        for attempt in range(1, 4):
            n.block_current_route()
            incoming = drivable_route((0., 0.), (4., 5. if attempt % 2 else -5.), (10., 0.))
            incoming.header.stamp = n.get_clock().now().to_msg()
            n.route_metadata[stamp_ns(incoming)] = dict(
                exploration_id='frontier:1', goal=[10., 0.], complete=True)
            n.start_segment.reset_mock()
            n.on_route(incoming)
            self.assertEqual(n.recovery_attempts, attempt)
            n.start_segment.assert_called_once()
        n.block_current_route()
        n.start_segment.reset_mock()
        n.on_route(n.route)
        n.start_segment.assert_not_called()
        self.assertEqual(n.last_status['status'], 'route_recovery_exhausted')

    def test_unchanged_blocked_route_does_not_spend_retry(self):
        n = self.node
        n.recovery_attempts = 1
        n.block_current_route()
        for _ in range(3):
            n.on_route(n.route)
        self.assertEqual(n.recovery_attempts, 1)
        n.start_segment.assert_not_called()

    def test_changed_partial_endpoint_is_still_same_owner_retry(self):
        n = self.node
        n.recovery_attempts = 1
        n.block_current_route()
        incoming = drivable_route((0., 0.), (16., 0.))
        incoming.header.stamp = n.get_clock().now().to_msg()
        n.route_metadata[stamp_ns(incoming)] = dict(
            exploration_id='frontier:1', goal=[10., 0.], complete=False)
        n.on_route(incoming)
        self.assertEqual(n.recovery_attempts, 2)
        self.assertIsNone(n.blocked_endpoint)
        n.start_segment.assert_called_once()

    def test_timer_exhaustion_requests_other_frontier_without_new_route_message(self):
        n = self.node
        n.block_current_route(backoff=-1.)
        n.recovery_attempts = 3
        n.status('follow_failed')
        n.reject_exploration_route = Mock(return_value=True)
        n.retry_when_planner_starts()
        n.reject_exploration_route.assert_called_once()
        n.start_segment.assert_not_called()
        self.assertEqual(n.last_status['status'], 'route_recovery_exhausted')
        n.retry_when_planner_starts()
        n.reject_exploration_route.assert_called_once()

    def test_timer_exhaustion_waits_for_fresh_inputs_and_action_release(self):
        n = self.node
        n.block_current_route(backoff=-1.)
        n.recovery_attempts = 3
        n.status('follow_failed')
        n.reject_exploration_route = Mock(return_value=True)
        n.following = True
        n.retry_when_planner_starts()
        n.following = False
        n.follow_quarantine = {1: object()}
        n.retry_when_planner_starts()
        n.follow_quarantine = {}
        n.input_freshness_issue = lambda: ('stale_odom', 2.)
        n.retry_when_planner_starts()
        n.reject_exploration_route.assert_not_called()
        n.start_segment.assert_not_called()

    def test_timer_exhaustion_cannot_reject_explicit_or_memory_goal(self):
        n = self.node
        n.block_current_route(backoff=-1.)
        n.recovery_attempts = 3
        n.status('follow_failed')
        n.route_rejection_pub = Mock()
        for memory in (None, (10., 0.)):
            n.expected, n.memory_goal = (10., 0.), memory
            n.retry_when_planner_starts()
        n.route_rejection_pub.publish.assert_not_called()
        n.start_segment.assert_not_called()

    def test_changed_route_after_transport_outage_is_not_execution_failure(self):
        n = self.node
        n.block_current_route()
        n.status('execution_unavailable')
        incoming = drivable_route((0., 0.), (4., 5.), (10., 0.))
        incoming.header.stamp = n.get_clock().now().to_msg()
        n.route_metadata[stamp_ns(incoming)] = dict(
            exploration_id='frontier:1', goal=[10., 0.], complete=True)
        n.on_route(incoming)
        self.assertEqual(n.recovery_attempts, 0)
        n.start_segment.assert_called_once()

    def test_repeated_route_during_active_retry_is_not_charged_twice(self):
        n = self.node
        n.block_current_route()
        incoming = drivable_route((0., 0.), (4., 5.), (10., 0.))
        incoming.header.stamp = n.get_clock().now().to_msg()
        n.route_metadata[stamp_ns(incoming)] = dict(
            exploration_id='frontier:1', goal=[10., 0.], complete=True)
        n.on_route(incoming)
        n.pending = True
        for _ in range(3):
            n.on_route(incoming)
            n.retry_when_planner_starts()
        self.assertEqual(n.recovery_attempts, 1)
        n.start_segment.assert_called_once()

    def test_new_owner_at_same_position_gets_its_own_budget(self):
        n = self.node
        n.block_current_route()
        n.last_plan_start, n.last_plan_endpoint = (0., 0.), (10., 0.)
        n.recovery_attempts = 3
        self.set_route('frontier:2', 10.)
        n.on_route(n.route)
        self.assertEqual(n.recovery_attempts, 0)
        self.assertIsNone(n.blocked_endpoint)
        n.start_segment.assert_called_once()

    def test_query_generation_cannot_reuse_previous_goal_progress(self):
        n = self.node
        self.succeed(0., 2.)
        n.recovery_attempts = 3
        n.query_generation += 1
        n.odom.pose.pose.position.x = 0.
        n.sync_recovery_progress()
        self.assertEqual(n.recovery_attempts, 0)
        n.recovery_attempts = 2
        self.succeed(0., 1.)
        self.assertEqual(n.recovery_attempts, 0)

    def test_memory_destination_keeps_its_budget_across_short_prefixes(self):
        n = self.node
        n.memory_goal = n.expected = (10., 0.)
        n.sync_recovery_progress()
        self.succeed(0., 1.)
        n.recovery_attempts = 3
        self.succeed(0., 1., cusp=False)
        self.assertEqual(n.recovery_attempts, 3)
        self.assertEqual(n.memory_goal, (10., 0.))

    def test_navigation_reset_clears_old_credit(self):
        n = self.node
        self.succeed(0., 2.)
        n.reset()
        self.assertIsNone(n.recovery_progress)
        self.assertEqual(n.recovery_attempts, 0)

    def test_old_callback_cannot_credit_current_owner(self):
        n = self.node
        n.recovery_attempts = 3
        self.succeed(0., 2., epoch=n.epoch-1)
        self.assertEqual(n.recovery_attempts, 3)

    def test_stale_pose_cannot_credit_success(self):
        n = self.node
        n.recovery_attempts = 3
        n.input_freshness_issue = lambda: ('stale_odom', 2.)
        self.succeed(0., 2.)
        self.assertEqual(n.recovery_attempts, 3)


if __name__ == '__main__':
    unittest.main()
