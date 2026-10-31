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
import rclpy


class StopPlanPreparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fixture=fixtures.GraphBridgeRoutingTests()
        self.fixture.setUp()
        self.node=n=self.fixture.node
        n.follow_enabled=True
        n.odom.pose.pose.orientation.w=1.
        n.route=n.latest_route=fixtures.route((0.,0.),(10.,0.))
        n.route_endpoint=(10.,0.)
        n.plan_client=Mock();n.plan_client.wait_for_server.return_value=True
        n.plan_client.send_goal_async.return_value=Future()
        n.follow_client=Mock();n.follow_client.send_goal_async.return_value=Future()
        n.follow_quarantine={7:dict(handle=Mock(),terminal=False)}

    def tearDown(self):
        self.fixture.tearDown()

    def prepare_result(self):
        n=self.node
        n.candidates=[(10.,0.,0.,10.)];n.candidate_index=0
        n.pending=True;n.active_plan_serial=42
        result=Future();result.set_result(S(status=4,result=S(
            path=fixtures.drivable_route((0.,0.),(10.,0.)))))
        n.on_plan_result(result,n.epoch,0,42)
        self.assertIsNotNone(getattr(n,'prepared_stop_plan',None))
        n.follow_client.send_goal_async.assert_not_called()

    def release_barrier(self):
        self.node.follow_quarantine.clear()
        self.node.follow_release=None

    def test_new_route_can_compute_while_old_action_is_quarantined(self):
        self.node.start_segment()
        self.node.plan_client.send_goal_async.assert_called_once()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertTrue(self.node.pending)

    def test_early_result_waits_for_barrier_then_dispatches_once(self):
        self.prepare_result()
        self.node.start_segment()
        self.node.plan_client.send_goal_async.assert_not_called()
        self.release_barrier()
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_called_once()
        self.assertIsNone(self.node.prepared_stop_plan)

    def test_prepared_plan_rechecks_new_local_obstacle(self):
        self.prepare_result()
        self.node.local_grid.data[10*40+14]=100
        self.release_barrier()
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status.get('detail'),'local_footprint_collision')

    def test_actual_stop_pose_trims_prepared_plan_before_execution(self):
        self.prepare_result()
        self.node.odom.pose.pose.position.x=.5
        self.release_barrier()
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_called_once()
        goal=self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(goal.path.poses[0].pose.position.x,.5,delta=.05)

    def test_unjoinable_stop_pose_replans_without_sending_cached_path(self):
        self.prepare_result()
        self.node.odom.pose.pose.position.y=2.
        self.release_barrier()
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.node.plan_client.send_goal_async.assert_called_once()
        self.assertIsNone(self.node.prepared_stop_plan)

    def test_reset_drops_prepared_path_and_old_epoch_result(self):
        self.prepare_result()
        old_epoch=self.node.epoch
        self.node.reset()
        self.assertIsNone(self.node.prepared_stop_plan)
        late=Mock()
        self.node.on_plan_result(late,old_epoch,0,42)
        late.result.assert_not_called()
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_source_fault_cannot_dispatch_prepared_plan(self):
        self.prepare_result()
        self.release_barrier()
        self.node.native_status['actuator_ok']=False
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_release_before_planner_result_keeps_one_request(self):
        self.node.start_segment()
        self.assertTrue(self.node.pending)
        self.release_barrier()
        self.node.start_segment()
        self.node.plan_client.send_goal_async.assert_called_once()

    def test_planner_timeout_is_still_monitored_during_stop_quarantine(self):
        n=self.node
        n.pending=True
        n.plan_request_started_at=time.monotonic()-n.compute_path_timeout-.1
        n.monitor_follow_transactions=Mock()
        n.recover_timed_out_plan=Mock()
        n.monitor_inputs()
        n.recover_timed_out_plan.assert_called_once()

    def test_changed_route_drops_prepared_plan_without_dispatching_old_owner(self):
        self.prepare_result()
        self.node.route=fixtures.route((0.,0.),(0.,10.))
        self.node.route_endpoint=(0.,10.)
        self.release_barrier()
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.node.plan_client.send_goal_async.assert_called_once()
        self.assertIsNone(self.node.prepared_stop_plan)

    def test_cached_plan_waits_for_terminal_standstill_and_release_ack(self):
        self.prepare_result()
        n=self.node
        n.native_status.update(sent={'brake':1.,'throttle':0.},speed=.2)
        n.monitor_follow_transactions()
        n.follow_client.send_goal_async.assert_not_called()
        n.follow_quarantine[7]['terminal']=True
        n.monitor_follow_transactions()
        self.assertIsNone(n.follow_release)
        n.native_status['speed']=0.
        n.follow_stop_since=time.monotonic()-.6
        n.monitor_follow_transactions()
        self.assertIsNotNone(n.follow_release)
        n.follow_client.send_goal_async.assert_not_called()
        control=dict(stamp_ns=n.follow_release['stamp_ns']+1,reason='execution_hold')
        n.control_status_snapshot=Mock(return_value=(control,time.monotonic()))
        n.monitor_follow_transactions()
        n.follow_client.send_goal_async.assert_not_called()
        control['reason']='stale_command'
        n.monitor_follow_transactions()
        n.follow_client.send_goal_async.assert_called_once()

    def pending_source_generation(self):
        n=self.node
        from wildos_nav2_adapter.source_control import PROTOCOL
        now=time.time_ns()
        brake=dict(throttle=0.,brake=1.,steering=0.,parkingbrake=0.,clutch=0.,gear=2)
        receipt=dict(version=PROTOCOL,clock='unix_ns',sender_id='a'*32,generation=4,
            sequence=10,issued_wall_ns=now-20_000_000,expires_wall_ns=now+480_000_000,
            applied_wall_ns=now-10_000_000,applied=True,control=brake)
        n.require_source_control=True
        n.native_status.update(wall_time=now/1e9,source_receipt=receipt,control_protocol=PROTOCOL,sent=brake)
        n.native_status_at=time.monotonic()
        n.control_status=dict(stamp_ns=now,reason='execution_hold',control_protocol=PROTOCOL,
            source_ack_ready=False,source_control=dict(sender_id='a'*32,generation=5,sequence=1))
        n.control_status_at=time.monotonic()
        self.assertEqual(n.execution_transport_issue(),'source_owner_unconfirmed')
        self.assertIsNone(n.execution_transport_issue(for_stop=True))

    def test_hold_source_generation_transition_does_not_abort_computation(self):
        self.pending_source_generation()
        self.node.start_segment()
        self.node.plan_client.send_goal_async.assert_called_once()
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_release_source_generation_transition_does_not_abort_computation(self):
        self.pending_source_generation()
        self.node.follow_quarantine.clear()
        self.node.follow_release=dict(started_at=time.monotonic(),stamp_ns=self.node.get_clock().now().nanoseconds)
        self.node.start_segment()
        self.node.plan_client.send_goal_async.assert_called_once()
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_prepared_execution_still_requires_current_source_generation_ack(self):
        self.prepare_result()
        self.pending_source_generation()
        self.release_barrier()
        self.node.start_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status.get('reason'),'source_owner_unconfirmed')


class MotionIntentHandoffTests(unittest.TestCase):
    retirement_reason = 'frontier_resolved'
    def setUp(self):
        self.fixture = fixtures.ExplorationTargetHandoffTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.handle = self.fixture.accepted_short_section(12.)

    def tearDown(self):
        self.fixture.tearDown()

    def resolve(self):
        self.fixture.resolve_frontier(self.retirement_reason)

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

    def test_exploration_handoff_lease_is_independent_and_bounded(self):
        n = self.node
        self.resolve()
        remaining = n.retired_motion['deadline'] - time.monotonic()
        self.assertGreater(remaining, 14.8)
        self.assertLess(remaining, 15.1)
        self.assertEqual(n.get_parameter('follow_response_timeout_sec').value, 5.)

    def test_visible_target_timeout_drains_only_current_validated_forward_section(self):
        n = self.node
        handle = self.fixture.accepted_short_section(6.)
        n.expected = (12., 1.)
        n.visible_approach_active = True
        n.request_graph_reset = Mock()
        n.retry_graph_reset = Mock()
        epoch = n.epoch

        n.release_visible_approach('visible_target_approach_timeout')

        handle.cancel_goal_async.assert_not_called()
        self.assertTrue(n.following)
        self.assertEqual(n.epoch, epoch)
        self.assertIsNotNone(n.retired_motion)
        self.assertEqual(n.retired_motion['owner_kind'], 'visible_target_timeout')
        self.assertEqual(n.retired_motion['distance_limit'], 5.)
        self.assertLess(n.retired_motion['deadline']-time.monotonic(), 5.1)
        self.assertIsNone(n.expected)
        self.assertIsNotNone(n.visible_blocked)
        self.assertIsNone(n.route)
        self.assertIsNone(n.latest_route)
        n.request_graph_reset.assert_called_once_with()
        self.assertEqual(n.last_status['status'], 'visible_target_approach_timeout')
        self.assertTrue(n.last_status['retained_current_section'])

    def test_visible_target_timeout_still_cancels_reverse_or_blocked_section(self):
        for case in ('reverse', 'blocked'):
            with self.subTest(case=case):
                n = self.node
                handle = self.fixture.accepted_short_section(9. if case == 'reverse' else 6.)
                if case == 'reverse':
                    n.motion_queue[0] = fixtures.drivable_route((0., 0.), (-9., 0.))
                else:
                    n.validate_local_path = Mock(return_value='local_footprint_collision')
                n.expected = (12., 1.)
                n.visible_approach_active = True
                n.request_graph_reset = Mock()
                n.retry_graph_reset = Mock()

                n.release_visible_approach('visible_target_approach_timeout')

                handle.cancel_goal_async.assert_called_once()
                self.assertTrue(n.follow_hold)
                self.assertIsNone(n.retired_motion)
                self.assertFalse(n.last_status.get('retained_current_section', False))

    def test_visible_target_timeout_finishes_only_a_short_checked_reverse_cusp(self):
        n = self.node
        handle = self.fixture.accepted_short_section(6.)
        n.motion_queue[0] = fixtures.drivable_route((0., 0.), (-6., 0.))
        n.expected = (12., 1.)
        n.visible_approach_active = True
        n.request_graph_reset = Mock()
        n.retry_graph_reset = Mock()

        n.release_visible_approach('visible_target_approach_timeout')

        handle.cancel_goal_async.assert_not_called()
        self.assertTrue(n.following)
        self.assertFalse(n.retired_motion['forward'])
        self.assertFalse(n.retired_motion['bounded'])
        self.assertTrue(n.last_status['retained_current_section'])

    def test_visible_target_handoff_does_not_reuse_expired_target_tail(self):
        n = self.node
        self.fixture.accepted_short_section(6.)
        n.expected = (12., 1.)
        n.visible_approach_active = True
        n.request_graph_reset = Mock()
        n.retry_graph_reset = Mock()
        n.release_visible_approach('visible_target_approach_timeout')
        replacement = fixtures.drivable_route((6., 0.), (18., 0.))
        self.fixture.metadata(replacement, 'frontier-b:2', (18., 0.))
        n.latest_route = replacement
        n.latest_route_complete = True
        n.odom.pose.pose.position.x = 6.
        n.try_retired_tail_candidate = Mock(return_value=True)
        n.on_route = Mock()

        self.fixture.complete_section()

        n.try_retired_tail_candidate.assert_not_called()
        self.assertIsNone(n.retired_motion)
        n.on_route.assert_called_once_with(replacement)

    def test_resolved_frontier_retires_pending_extension_not_accepted_motion(self):
        n = self.node
        epoch = n.epoch
        n.pending = n.extending_path = True
        planner = n.plan_handle = Mock()
        n.active_plan_serial = 99
        self.resolve()
        self.handle.cancel_goal_async.assert_not_called()
        planner.cancel_goal_async.assert_called_once()
        self.assertTrue(n.following)
        self.assertEqual(n.epoch, epoch)
        self.assertIsNotNone(n.retired_motion)
        self.assertFalse(n.pending)
        self.assertFalse(n.extending_path)
        self.assertIsNone(n.active_plan_serial)

    def test_resolved_frontier_fences_unaccepted_background_plan_response(self):
        n = self.node
        epoch = n.epoch
        n.pending = n.extending_path = True
        n.plan_handle = None  # ComputePath response has not arrived yet.
        n.active_plan_serial = 99
        self.resolve()
        self.handle.cancel_goal_async.assert_not_called()
        late = Future(); accepted = Mock(accepted=True); late.set_result(accepted)
        n.on_plan_accepted(late, epoch, 0, 99)
        accepted.cancel_goal_async.assert_called_once()
        self.assertTrue(n.following)
        self.assertIsNotNone(n.retired_motion)

    def test_pending_extension_does_not_hide_new_local_obstacle_on_retirement(self):
        n = self.node
        n.pending = n.extending_path = True
        n.validate_local_path = Mock(return_value='local_footprint_collision')
        self.resolve()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertTrue(n.follow_hold)

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
        self.handle.cancel_goal_async.assert_not_called()
        self.assertIsNotNone(n.retired_motion)
        n.odom.pose.pose.position.x = 16.1
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
        data.update(exploration_id='frontier-b:2', reason=self.retirement_reason)
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
        data.update(exploration_id='frontier-b:2', reason=self.retirement_reason)
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
        self.assertIsNotNone(n.follow_release)
        n.start_segment.assert_not_called()
        n.control_status=dict(stamp_ns=n.follow_release['stamp_ns']+1,reason='stale_command')
        n.monitor_follow_transactions()
        self.assertIsNone(n.follow_release)
        n.start_segment.assert_called_once()


class MissingFrontierMotionIntentHandoffTests(MotionIntentHandoffTests):
    retirement_reason = 'frontier_missing'


if __name__ == '__main__':
    unittest.main()
