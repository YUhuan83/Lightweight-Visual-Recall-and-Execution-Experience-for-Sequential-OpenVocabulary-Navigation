"""Exploration intention changes must not implicitly revoke a safe motion lease."""
from concurrent.futures import Future
from copy import deepcopy
import json
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import test_graph_path_bridge_routing as fixtures
from std_msgs.msg import String


class MotionIntentHandoffTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.ExplorationTargetHandoffTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.handle = self.fixture.accepted_short_section(12.)

    def tearDown(self):
        self.fixture.tearDown()

    def resolve(self):
        self.fixture.resolve_frontier()

    def new_route(self):
        n = self.node
        route = fixtures.drivable_route((0., 0.), (18., 0.))
        self.fixture.metadata(route, 'frontier-b:2', (18., 0.))
        n.latest_route = route
        n.latest_route_complete = True
        return route

    def test_long_forward_action_gets_bounded_handoff_without_cancel(self):
        n = self.node
        epoch = n.epoch
        self.resolve()
        self.handle.cancel_goal_async.assert_not_called()
        self.assertTrue(n.following)
        self.assertEqual(n.epoch, epoch)
        self.assertEqual(n.active_follow_serial, 7)
        self.assertTrue(n.retired_motion['bounded'])
        n.route_observation_pub.publish.assert_not_called()

    def test_new_intention_is_planned_while_old_action_still_runs(self):
        n = self.node
        self.resolve()
        route = self.new_route()
        n.start_segment = Mock()
        n.maybe_extend_path()
        self.assertIs(n.route, route)
        n.start_segment.assert_called_once_with(continuous=True)
        self.assertTrue(n.extending_path)
        self.assertTrue(n.following)
        self.handle.cancel_goal_async.assert_not_called()

    def test_expired_or_retired_new_route_cannot_be_planned(self):
        n = self.node
        self.resolve()
        route = self.new_route()
        n.retired_explorations['frontier-b:2'] = 1
        n.start_segment = Mock()
        n.maybe_extend_path()
        n.retired_explorations.clear()
        route.header.stamp.sec -= 6
        n.maybe_extend_path()
        n.start_segment.assert_not_called()

    def test_borrowed_motion_times_out_through_cancellation_quarantine(self):
        n = self.node
        self.resolve()
        n.retired_motion['deadline'] = time.monotonic()-1.
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertIn(7, n.follow_quarantine)
        self.assertTrue(n.follow_hold)
        self.assertIsNone(n.retired_motion)

    def test_distance_budget_is_checked_while_background_planning(self):
        n = self.node
        self.resolve()
        n.pending = n.extending_path = True
        n.odom.pose.pose.position.x = 8.1
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertTrue(n.follow_hold)

    def test_retiring_only_background_intention_does_not_cancel_old_motion(self):
        n = self.node
        self.resolve()
        old = n.route
        n.route = self.new_route()
        n.route_endpoint = (18., 0.)
        n.pending = n.extending_path = True
        plan = n.plan_handle = Mock()
        n.active_plan_serial = 99
        data = json.loads(self.fixture.retirement().data)
        data.update(exploration_id='frontier-b:2', reason='frontier_resolved')
        n.on_exploration_retired(String(data=json.dumps(data)))
        plan.cancel_goal_async.assert_called_once()
        self.handle.cancel_goal_async.assert_not_called()
        self.assertIs(n.route, old)
        self.assertTrue(n.following)
        self.assertFalse(n.pending)
        self.assertIsNone(n.active_plan_serial)

    def test_replacement_acceptance_transfers_ownership_and_fences_old_result(self):
        n = self.node
        self.resolve()
        n.route = self.new_route()
        n.follow_request_serial = 8
        n.follow_request = dict(serial=8, target=(18., 0.), heading=0., xy_tolerance=1.,
                                yaw_tolerance=.7, started_at=time.monotonic(), replacement=True)
        n.replacement_pending = True
        n.checkpoint_execution_feedback = Mock()
        accepted = Mock(accepted=True)
        future = Future(); future.set_result(accepted)
        n.on_follow_accepted(future, n.epoch, (18., 0.), 8,
                             [fixtures.drivable_route((0., 0.), (18., 0.))], True)
        self.assertIsNone(n.retired_motion)
        self.assertEqual(n.active_follow_serial, 8)
        self.assertIs(n.follow_handle, accepted)
        self.fixture.complete_section(status=5, serial=7)
        self.assertIs(n.follow_handle, accepted)
        self.assertTrue(n.following)

    def test_old_terminal_does_not_discard_pending_new_intention_plan(self):
        n = self.node
        self.resolve()
        route = self.new_route()
        n.route = route
        n.pending = n.extending_path = True
        plan = n.plan_handle = Mock()
        n.active_plan_serial = 99
        n.motion_queue = [fixtures.drivable_route((0., 0.), (6., 0.))]
        n.odom.pose.pose.position.x = 6.
        self.fixture.complete_section()
        self.assertIsNone(n.retired_motion)
        self.assertTrue(n.pending)
        self.assertTrue(n.extending_path)
        self.assertEqual(n.active_plan_serial, 99)
        self.assertIs(n.route, route)
        plan.cancel_goal_async.assert_not_called()

    def test_local_obstacle_during_borrowed_motion_still_stops(self):
        n = self.node
        self.resolve()
        n.validate_local_path = Mock(return_value='local_footprint_collision')
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertTrue(n.follow_hold)

    def test_unhealthy_native_still_stops_during_background_plan(self):
        n = self.node
        self.resolve()
        n.pending = n.extending_path = True
        n.execution_transport_issue = Mock(return_value='source_expired')
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertTrue(n.follow_hold)

    def test_late_cancelled_background_response_cannot_dispatch_motion(self):
        n = self.node
        self.resolve()
        n.route = self.new_route()
        n.pending = n.extending_path = True
        n.active_plan_serial = 99
        n.candidate_index = 0
        epoch = n.epoch
        self.assertTrue(n.retire_background_exploration('frontier-b:2'))
        n.candidate_failed = Mock()
        late = Future(); handle = Mock(accepted=True); late.set_result(handle)
        n.on_plan_accepted(late, epoch, 0, 99)
        handle.cancel_goal_async.assert_called_once()
        n.candidate_failed.assert_not_called()
        self.handle.cancel_goal_async.assert_not_called()

    def test_retirement_during_replacement_ack_gap_quarantines_both_actions(self):
        n = self.node
        self.resolve()
        n.route = self.new_route()
        n.follow_request = dict(serial=8, replacement=True, started_at=time.monotonic())
        n.replacement_pending = True
        data = json.loads(self.fixture.retirement().data)
        data.update(exploration_id='frontier-b:2', reason='frontier_resolved')
        n.on_exploration_retired(String(data=json.dumps(data)))
        self.assertTrue(n.follow_hold)
        self.assertEqual(set(n.follow_quarantine), {7, 8})
        self.assertFalse(n.following)

    def test_query_change_revokes_long_borrowed_motion(self):
        n = self.node
        self.resolve()
        n.on_query(String(data='gas station'))
        self.handle.cancel_goal_async.assert_called_once()
        self.assertIsNone(n.retired_motion)
        self.assertTrue(n.follow_hold)

    def test_failed_new_plan_keeps_old_motion_only_within_existing_lease(self):
        n = self.node
        self.resolve()
        lease = n.retired_motion
        n.route = self.new_route()
        n.extending_path = n.pending = True
        n.extension_failed('empty_path')
        self.assertIs(n.retired_motion, lease)
        self.assertTrue(n.following)
        self.assertFalse(n.pending)
        self.handle.cancel_goal_async.assert_not_called()
        lease['deadline'] = time.monotonic()-1.
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()

    def test_old_abort_does_not_block_new_intention_and_waits_for_standstill(self):
        n = self.node
        self.resolve()
        n.start_segment = Mock()
        route = self.new_route()
        n.maybe_extend_path()
        n.try_execution_escape = Mock(return_value=False)
        self.fixture.complete_section(status=6)
        self.assertIs(n.route, route)
        self.assertIsNone(n.blocked_endpoint)
        n.route_rejection_pub.publish.assert_not_called()
        self.assertEqual(n.recovery_attempts, 0)
        self.assertTrue(n.follow_hold)
        self.assertTrue(n.follow_quarantine[7]['terminal'])
        self.assertEqual(n.last_status['retired_exploration_id'], 'frontier-a:1')

    def test_old_tracking_failure_does_not_reject_unexecuted_new_owner(self):
        n = self.node
        self.resolve()
        n.start_segment = Mock()
        route = self.new_route()
        n.maybe_extend_path()
        n.recovery_attempts = 2
        n.try_execution_escape = Mock(return_value=False)
        n.follow_failed('tracking_deviation', cross_track_error=3.)
        self.assertIs(n.route, route)
        self.assertIsNone(n.blocked_endpoint)
        n.route_rejection_pub.publish.assert_not_called()
        self.assertTrue(n.follow_hold)
        self.assertFalse(n.follow_quarantine[7]['terminal'])
        self.handle.cancel_goal_async.assert_called_once()

    def test_new_obstacle_in_current_forward_sweep_stops_borrowed_motion(self):
        n = self.node
        self.resolve()
        n.odom.pose.pose.position.x = 5.
        n.local_grid.data[10*40+19] = 100  # x9, beyond current body, in next 2m.
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertTrue(n.follow_hold)

    def test_executed_prefix_outside_rolling_map_does_not_cancel_clear_forward_sweep(self):
        n = self.node
        self.resolve()
        n.odom.pose.pose.position.x = 5.
        n.local_grid.info.origin.position.x = 1.
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_not_called()
        self.assertTrue(n.following)
        self.assertIsNotNone(n.retired_motion)

    def test_borrowed_motion_cannot_continue_if_actual_pose_lost_the_path(self):
        n = self.node
        self.resolve()
        n.odom.pose.pose.position.y = 1.1
        n.maybe_extend_path()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertTrue(n.follow_hold)

    def test_new_intention_can_start_only_after_old_terminal_and_measured_stop(self):
        n = self.node
        self.resolve()
        n.start_segment = Mock()
        self.new_route()
        n.maybe_extend_path()
        self.fixture.complete_section(status=6)
        n.start_segment.reset_mock()
        n.native_status.update(speed=.5, sent={'brake': 1.})
        n.monitor_follow_transactions()
        n.start_segment.assert_not_called()
        self.assertTrue(n.follow_hold)
        n.native_status['speed'] = 0.
        n.monitor_follow_transactions()
        n.start_segment.assert_not_called()
        n.follow_stop_since = time.monotonic() - 1.
        n.monitor_follow_transactions()
        self.assertFalse(n.follow_hold)
        self.assertFalse(n.follow_quarantine)
        n.start_segment.assert_called_once()


if __name__ == '__main__':
    unittest.main()
