"""Fault delivery tests of production transactions; no DDS or vehicle physics."""
from concurrent.futures import Future
from copy import deepcopy
import json
import time
from queue import SimpleQueue
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
from std_msgs.msg import String
import test_execution_mechanism_ros as mechanisms
from test_graph_path_bridge_routing import route, drivable_route
from wildos_nav2_adapter.navigation_transactions import NavigationTransactions


class NavigationResultQueueTests(unittest.TestCase):
    def test_completed_results_wait_for_group_dispatch_and_keep_order(self):
        node = S(executor=object(), navigation_results=SimpleQueue(),
                 navigation_result_guard=Mock())
        first, second = Future(), Future()
        observed = []
        for future in (first, second):
            NavigationTransactions.add_navigation_done_callback(
                node, future, lambda done: observed.append(done.result()))
        second.set_result('second')
        first.set_result('first')
        self.assertEqual(observed, [])
        self.assertEqual(node.navigation_result_guard.trigger.call_count, 2)
        NavigationTransactions.drain_navigation_results(node)
        self.assertEqual(observed, ['second', 'first'])
        NavigationTransactions.drain_navigation_results(node)
        self.assertEqual(observed, ['second', 'first'])

    def test_reset_before_dispatch_is_visible_to_existing_epoch_gate(self):
        node = S(executor=object(), navigation_results=SimpleQueue(),
                 navigation_result_guard=Mock(), epoch=5)
        future, dispatched = Future(), []
        original_epoch = node.epoch
        NavigationTransactions.add_navigation_done_callback(
            node, future, lambda done: dispatched.append(done.result())
            if node.epoch == original_epoch else None)
        future.set_result('old_path')
        node.epoch += 1
        NavigationTransactions.drain_navigation_results(node)
        self.assertEqual(dispatched, [])


class NavigationTransactionTests(unittest.TestCase):
    def setUp(self):
        self.wall = 1000.
        self.monotonic = patch('time.monotonic', side_effect=lambda: self.wall)
        self.utc = patch('time.time', side_effect=lambda: 1790000000.+self.wall)
        self.monotonic.start(); self.utc.start()
        self.fixture = mechanisms.ExecutionMechanismTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        # These tests inject deliveries and inspect calls explicitly. Actual
        # ROS publishers do not expose Mock's side_effect/call_count API.
        self.node.hold_pub = Mock()
        self.node.waypoint_pub = Mock()
        self.node.query = self.node.expected = None
        self.node.motion_queue = [route((0., 0.), (6., 0.))]
        self.responses = []
        self.feedback = []
        def dispatch(goal, **kwargs):
            response = Future()
            self.responses.append(response)
            self.feedback.append(kwargs.get('feedback_callback'))
            return response
        self.node.follow_client.send_goal_async.side_effect = dispatch
        self.refresh()

    def tearDown(self):
        self.fixture.tearDown()
        self.utc.stop(); self.monotonic.stop()

    def refresh(self):
        n = self.node
        for value, receive in ((n.odom,n.on_odom),(n.grid,n.on_map),(n.local_grid,n.on_local_map)):
            value = deepcopy(value)
            value.header.stamp = n.get_clock().now().to_msg()
            receive(value)
        n.on_native_status(String(data=json.dumps(dict(wall_time=time.time(), speed=0.,
            telemetry_age_sec=0., actuator_ok=True, telemetry_ok=True,
            control_acknowledged=True, actual={'brake':1.}, sent={'brake':1.}))))
        n.on_control_status(String(data=json.dumps(dict(stamp_ns=self.fixture.now,
            actuating=False,target_speed=0.,target_angular=0.))))

    def tick(self, seconds=12):
        for _ in range(seconds):
            self.wall += 1.
            self.fixture.now += 10**9
            self.refresh()
            self.node.monitor_inputs()
            self.node.retry_graph_reset()
            if self.node.follow_release is not None:
                # Model the following 20Hz control/Native acknowledgement;
                # dedicated tests below intentionally withhold this delivery.
                self.wall += .05
                self.fixture.now += 50_000_000
                self.refresh()
                self.node.monitor_follow_transactions()

    def prepare_control_release(self):
        n=self.node
        n.follow_hold=True
        n.follow_quarantine={17:dict(handle=None,terminal=True)}
        n.follow_stop_since=self.wall-1.
        n.start_segment=Mock()
        n.resume_execution_escape_after_stop=Mock(return_value=False)
        n.execution_transport_issue=Mock(return_value=None)
        self.assertTrue(n.monitor_follow_transactions())
        self.assertIsNotNone(n.follow_release)
        self.assertFalse(n.follow_hold)
        n.start_segment.assert_not_called()
        return n

    def test_control_release_waits_for_new_generation_ack_without_reset(self):
        n=self.prepare_control_release();epoch=n.epoch
        n.control_status.update(stamp_ns=self.fixture.now+1,reason='stale_command')
        n.execution_transport_issue.return_value='source_owner_unconfirmed'
        self.wall+=.04
        self.assertTrue(n.monitor_follow_transactions())
        self.assertIsNotNone(n.follow_release)
        self.assertEqual(n.epoch,epoch)
        self.assertFalse(n.follow_quarantine)
        n.start_segment.assert_not_called()
        n.execution_transport_issue.return_value=None
        self.assertTrue(n.monitor_follow_transactions())
        self.assertIsNone(n.follow_release)
        self.assertEqual(n.epoch,epoch)
        n.start_segment.assert_called_once()

    def test_old_or_still_held_status_cannot_acknowledge_control_release(self):
        n=self.prepare_control_release()
        n.control_status.update(stamp_ns=self.fixture.now,reason='stale_command')
        n.monitor_follow_transactions()
        self.assertIsNotNone(n.follow_release)
        n.control_status.update(stamp_ns=self.fixture.now+1,reason='execution_hold')
        n.monitor_follow_transactions()
        self.assertIsNotNone(n.follow_release)
        n.start_segment.assert_not_called()

    def test_missing_release_ack_times_out_without_authorizing_motion(self):
        n=self.prepare_control_release()
        n.follow_transaction_timeout=Mock()
        n.control_status.update(stamp_ns=self.fixture.now+1,reason='stale_command')
        n.execution_transport_issue.return_value='source_not_applied'
        self.wall+=6.
        n.monitor_follow_transactions()
        self.assertIsNone(n.follow_release)
        n.follow_transaction_timeout.assert_called_once_with('control_release_ack_timeout')
        n.start_segment.assert_not_called()

    def test_new_query_revokes_pending_control_release(self):
        n=self.prepare_control_release()
        n.on_query(String(data='new target'))
        self.assertIsNone(n.follow_release)

    def accept(self, index=-1):
        result = Future()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        self.responses[index].set_result(handle)
        return handle,result

    def prepare_quarantined_escape(self):
        self.node.send_motion_segment()
        handle,result=self.accept()
        self.node.escape_trace.extend((i*.3,0.,0.,self.wall) for i in range(-10,1))
        self.node.escape_failures.append((0.,0.,self.wall))
        self.node.follow_failed('execution_stalled',stall_class='actuated_no_path_progress')
        return handle,result

    def prepare_overdue_visible_recovery(self):
        responses = self.service()
        responses[-1].set_result(S(success=True))
        self.node.query = 'green residential house'
        self.node.expected = (6.,0.)
        self.node.visible_approach_active = True
        self.node.visible_approach_started_at = self.wall-11.
        return responses

    def test_visual_release_retains_physical_failure_history_and_stop_barrier(self):
        n=self.node; clears=self.prepare_overdue_visible_recovery()
        n.visible_approach_deadline = self.wall-1.
        n.send_motion_segment(); _,result=self.accept()
        n.escape_trace.extend((i*.3,0.,0.,self.wall) for i in range(-10,1))
        n.follow_failed('execution_stalled',stall_class='actuated_no_path_progress')
        n.escape_attempt_position=(-4.,0.)
        n.activate_visible_approach()
        self.assertFalse(n.visible_approach_active)
        self.assertIsNone(n.expected)
        self.assertEqual(len(n.escape_trace),11)
        self.assertEqual(len(n.escape_failures),1)
        self.assertEqual(n.escape_attempt_position,(-4.,0.))
        self.assertTrue(n.follow_quarantine)
        self.assertTrue(n.follow_hold)
        self.assertTrue(n.query_reset_pending)
        self.assertEqual(len(self.responses),1)
        # Complete the prior stop and semantic clear, then fail a new action
        # at the same physical position. The earlier contact still counts.
        result.set_result(S(status=6))
        n.start_segment=Mock(); self.tick(2)
        clears[-1].set_result(S(success=True))
        n.route=n.latest_route=drivable_route((0.,0.),(6.,0.))
        n.route_endpoint=(6.,0.)
        n.motion_queue=[drivable_route((0.,0.),(6.,0.))]
        n.send_motion_segment(); self.accept()
        n.follow_failed('execution_stalled',stall_class='actuated_no_path_progress')
        self.assertIsNotNone(n.deferred_execution_escape)

    def test_visual_timeout_waits_for_physical_recovery_and_measured_stop(self):
        n=self.node; clears=self.prepare_overdue_visible_recovery()
        _,result=self.prepare_quarantined_escape()
        intent=n.deferred_execution_escape; epoch=n.epoch
        n.activate_visible_approach()
        self.assertIs(n.deferred_execution_escape,intent)
        self.assertEqual(n.epoch,epoch)
        self.assertEqual(n.expected,(6.,0.))
        self.assertFalse(n.query_reset_pending)
        self.assertEqual(len(clears),1)
        self.tick(1)
        self.assertEqual(len(self.responses),1)
        result.set_result(S(status=6))
        self.tick(2)
        self.assertTrue(n.recovering)
        self.assertEqual(len(self.responses),2)
        self.assertIsNone(n.deferred_execution_escape)
        self.assertLess(n.motion_queue[0].poses[-1].pose.position.x,0.)

    def test_visual_release_cannot_revoke_queued_physical_recovery(self):
        n=self.node; self.prepare_overdue_visible_recovery()
        self.prepare_quarantined_escape()
        intent=n.deferred_execution_escape; epoch=n.epoch
        n.release_visible_approach('visible_target_route_timeout')
        self.assertIs(n.deferred_execution_escape,intent)
        self.assertEqual(n.epoch,epoch)
        self.assertTrue(n.visible_approach_active)
        self.assertFalse(n.query_reset_pending)

    def test_visual_hint_cannot_cancel_an_accepted_physical_escape(self):
        n=self.node; n.query='green residential house'
        _,result=self.prepare_quarantined_escape()
        result.set_result(S(status=6)); self.tick(2)
        self.assertTrue(n.recovering)
        handle,_=self.accept(); epoch=n.epoch
        n.on_visible_approach(String(data=json.dumps(dict(query=n.query,frame='spot1/odom',
            observation_stamp_ns=self.fixture.now,point=[6.,0.],bearing=0.,
            evidence='bearing_only',position_confirmed=False,search_confirmed=False))))
        self.assertTrue(n.recovering)
        self.assertIs(n.follow_handle,handle)
        self.assertEqual(n.epoch,epoch)
        handle.cancel_goal_async.assert_not_called()
        self.assertFalse(n.visible_approach_active)

    def test_visual_activation_retains_measured_history_without_reviving_actions(self):
        n=self.node; n.query='green residential house'
        n.escape_trace.append((-1.,0.,0.,self.wall))
        n.escape_failures.append((0.,0.,self.wall))
        n.escape_attempt_position=(-2.,0.)
        n.on_visible_approach(String(data=json.dumps(dict(query=n.query,frame='spot1/odom',
            observation_stamp_ns=self.fixture.now,point=[6.,0.],bearing=0.,
            evidence='bearing_only',position_confirmed=False,search_confirmed=False))))
        self.assertTrue(n.visible_approach_active)
        self.assertEqual(len(n.escape_trace),1)
        self.assertEqual(len(n.escape_failures),1)
        self.assertEqual(n.escape_attempt_position,(-2.,0.))
        self.assertIsNone(n.follow_request)
        self.assertFalse(n.recovering)
        self.assertFalse(self.responses)

    def test_visual_observation_completion_retains_physical_history(self):
        n=self.node; self.prepare_overdue_visible_recovery()
        n.escape_trace.append((-1.,0.,0.,self.wall))
        n.escape_failures.append((0.,0.,self.wall))
        n.finish_visible_approach()
        self.assertEqual(len(n.escape_trace),1)
        self.assertEqual(len(n.escape_failures),1)
        self.assertIsNotNone(n.visible_reobserve_until)
        self.assertFalse(n.object_found)

    def test_visual_handoff_cannot_extend_expired_measured_corridor(self):
        n=self.node; clears=self.prepare_overdue_visible_recovery()
        n.escape_trace.extend((i*.3,0.,0.,self.wall-61.) for i in range(-10,1))
        n.escape_failures.append((0.,0.,self.wall))
        n.release_visible_approach('visible_target_route_timeout')
        self.assertEqual(len(n.escape_trace),11)
        clears[-1].set_result(S(success=True))
        self.assertFalse(n.try_execution_escape('execution_stalled',
            dict(stall_class='actuated_no_path_progress')))
        self.assertFalse(self.responses)
        self.assertFalse(n.recovering)

    def test_query_change_still_revokes_physical_recovery_after_visual_timeout(self):
        n=self.node; self.prepare_overdue_visible_recovery()
        _,result=self.prepare_quarantined_escape()
        n.activate_visible_approach()
        n.on_query(String(data='gas station'))
        self.assertIsNone(n.deferred_execution_escape)
        self.assertFalse(n.escape_trace)
        n.start_segment=Mock()
        result.set_result(S(status=6)); self.tick(2)
        self.assertEqual(len(self.responses),1)
        self.assertFalse(n.recovering)
        self.assertFalse(n.visible_approach_active)

    def test_visual_timeout_does_not_bypass_missing_terminal_or_stale_inputs(self):
        n=self.node; self.prepare_overdue_visible_recovery()
        _,result=self.prepare_quarantined_escape()
        n.activate_visible_approach()
        self.tick(3)
        self.assertTrue(n.follow_hold)
        self.assertEqual(len(self.responses),1)
        result.set_result(S(status=6))
        n.input_freshness_issue=lambda: ('stale_odom',2.)
        self.tick(2)
        self.assertTrue(n.follow_hold)
        self.assertEqual(len(self.responses),1)
        self.assertFalse(n.recovering)

    def test_visual_timeout_releases_after_physical_escape_finishes(self):
        n=self.node; self.prepare_overdue_visible_recovery()
        n.visible_approach_deadline = self.wall-1.
        _,result=self.prepare_quarantined_escape()
        n.activate_visible_approach()
        result.set_result(S(status=6)); self.tick(2)
        _,escaped=self.accept()
        endpoint=n.escape_path.poses[-1].pose.position
        n.odom.pose.pose.position.x,n.odom.pose.pose.position.y=endpoint.x,endpoint.y
        n.start_segment=Mock()
        escaped.set_result(S(status=4))
        self.assertFalse(n.recovering)
        self.assertIsNone(n.deferred_execution_escape)
        n.activate_visible_approach()
        self.assertFalse(n.visible_approach_active)
        self.assertIsNone(n.expected)
        self.assertTrue(n.query_reset_pending)

    def test_visual_hint_waits_for_deferred_planning_escape(self):
        n=self.node; n.query='green residential house'
        n.failure_reasons=['empty_path']
        n.escape_trace.extend((i*.3,0.,0.,self.wall) for i in range(-10,1))
        n.exploration_metadata=Mock(return_value=dict(exploration_id='frontier-a'))
        n.local_grid_received_at=self.wall-5.
        self.assertFalse(n.try_planning_escape())
        n.exploration_metadata.return_value=dict(exploration_id='frontier-b')
        self.assertFalse(n.try_planning_escape())
        intent=n.deferred_planning_escape; self.assertIsNotNone(intent)
        n.on_visible_approach(String(data=json.dumps(dict(query=n.query,frame='spot1/odom',
            observation_stamp_ns=self.fixture.now,point=[6.,0.],bearing=0.,
            evidence='bearing_only',position_confirmed=False,search_confirmed=False))))
        self.assertIs(n.deferred_planning_escape,intent)
        self.assertIsNone(n.expected)
        self.assertFalse(n.visible_approach_active)
        self.wall+=.1; self.fixture.now+=100_000_000
        self.refresh(); n.monitor_inputs()
        self.assertTrue(n.recovering,(n.last_status,n.deferred_planning_escape,
                                     list(n.planning_escape_failures)))
        self.assertEqual(len(self.responses),1)

    def test_escape_waits_for_old_terminal_result_and_settled_stop(self):
        handle,result=self.prepare_quarantined_escape()
        self.assertEqual(len(self.responses),1)
        self.assertFalse(self.node.recovering)
        self.node.send_motion_segment=Mock()
        self.node.start_segment=Mock()
        self.tick(1)
        self.node.send_motion_segment.assert_not_called()
        result.set_result(S(status=6))
        self.tick(2)
        self.node.send_motion_segment.assert_called_once()
        self.assertTrue(self.node.recovering)
        self.node.start_segment.assert_not_called()
        self.assertIsNone(self.node.deferred_execution_escape)

    def test_new_query_revokes_escape_intent_during_cancellation_wait(self):
        handle,result=self.prepare_quarantined_escape()
        self.node.query_generation+=1
        self.node.send_motion_segment=Mock()
        self.node.start_segment=Mock()
        result.set_result(S(status=6))
        self.tick(2)
        self.node.send_motion_segment.assert_not_called()
        self.assertFalse(self.node.recovering)
        self.assertIsNone(self.node.deferred_execution_escape)
        self.node.start_segment.assert_called_once()

    def test_expired_or_moved_escape_intent_falls_back_after_stop(self):
        for change in ('expired','moved'):
            with self.subTest(change=change):
                self.node.init_execution_recovery()
                self.node.following=False
                self.node.follow_quarantine.clear()
                self.node.motion_queue=[drivable_route((0.,0.),(6.,0.))]
                self.node.odom.pose.pose.position.x=0.
                handle,result=self.prepare_quarantined_escape()
                if change=='expired':
                    self.node.deferred_execution_escape['time']-=61.
                else:
                    self.node.odom.pose.pose.position.x=2.
                result.set_result(S(status=6))
                with patch.object(self.node,'start_segment') as normal:
                    self.tick(2)
                    self.assertFalse(self.node.recovering)
                    self.assertIsNone(self.node.deferred_execution_escape)
                    normal.assert_called_once()

    def test_acceptance_gap_becomes_explicit_failure_in_both_profiles(self):
        for profile in ('ackermann','ideal_point'):
            with self.subTest(profile=profile):
                self.node.carrier_model = profile
                self.node.send_motion_segment()
                self.tick()
                self.assertFalse(self.node.following)
                self.assertEqual(self.node.last_status['reason'], 'follow_acceptance_timeout')
                self.assertFalse(self.node.object_found)
                # Each profile has its own fresh node/transaction.
                if profile == 'ackermann':
                    self.tearDown(); self.setUp()

    def test_late_acceptance_is_cancelled_and_requires_terminal_and_stop_evidence(self):
        n = self.node
        n.send_motion_segment(); self.tick()
        n.start_segment = Mock()
        handle,result = self.accept()
        handle.cancel_goal_async.assert_called_once_with()
        self.assertTrue(n.follow_hold)
        self.tick(2)
        n.start_segment.assert_not_called()
        result.set_result(S(status=5))
        self.tick(2)
        self.assertFalse(n.follow_hold)
        n.start_segment.assert_called_once_with()
        self.assertFalse(n.object_found)

    def test_acceptance_exception_cannot_be_treated_as_rejection(self):
        n = self.node
        n.send_motion_segment()
        self.responses[-1].set_exception(RuntimeError('transport error'))
        self.assertEqual(n.last_status['reason'],'follow_acceptance_error')
        self.assertTrue(n.follow_hold)
        n.start_segment = Mock(); self.tick(20)
        n.start_segment.assert_not_called()

    def test_send_exception_does_not_restore_old_replacement(self):
        n = self.node
        n.send_motion_segment(); handle,_ = self.accept()
        n.follow_client.send_goal_async.side_effect = RuntimeError('unknown send outcome')
        n.send_motion_segment(replacement=[drivable_route((0.,0.),(12.,0.))])
        self.assertEqual(n.last_status['reason'],'controller_send_error')
        self.assertTrue(n.follow_hold)
        handle.cancel_goal_async.assert_called()
        self.assertFalse(n.following)

    def test_rejection_after_timeout_can_recover_without_inventing_success(self):
        n = self.node
        n.send_motion_segment(); self.tick()
        n.start_segment = Mock()
        self.responses[-1].set_result(Mock(accepted=False))
        self.tick(2)
        self.assertFalse(n.follow_hold)
        n.start_segment.assert_called_once_with()
        self.assertFalse(n.object_found)

    def test_cancellation_ack_alone_cannot_release_hold(self):
        n = self.node
        n.send_motion_segment(); handle,result = self.accept()
        cancellation = Future(); cancellation.set_result(S(goals_canceling=[S()]))
        handle.cancel_goal_async.return_value = cancellation
        self.tick(12)
        self.assertTrue(n.follow_hold)
        self.assertFalse(result.done())

    def test_result_exception_keeps_unproven_action_held(self):
        n = self.node
        n.send_motion_segment(); _,result = self.accept()
        result.set_exception(RuntimeError('get_result failed'))
        self.assertEqual(n.last_status['reason'],'follow_result_error')
        self.assertTrue(n.follow_hold)
        self.assertFalse(n.following)

    def test_nonterminal_result_does_not_grant_cancellation_permission(self):
        n = self.node
        n.send_motion_segment(); _,result = self.accept()
        result.set_result(S(status=2))
        self.assertEqual(n.last_status['reason'],'follow_result_invalid')
        self.assertTrue(n.follow_hold)

    def test_late_success_after_timeout_is_stop_evidence_only(self):
        n = self.node
        n.send_motion_segment(); _,result = self.accept()
        self.tick()
        n.odom.pose.pose.position.x = 6.
        n.start_segment = Mock()
        result.set_result(S(status=4))
        self.assertFalse(n.object_found)
        self.assertNotEqual(n.last_status['status'],'segment_done')
        n.native_status['speed'] = .2
        n.monitor_inputs()
        self.assertTrue(n.follow_hold)
        self.tick(2)
        self.assertFalse(n.follow_hold)
        n.start_segment.assert_called_once_with()

    def test_missing_replacement_requires_both_actions_to_terminate(self):
        n = self.node
        n.send_motion_segment(); old,old_result = self.accept()
        n.send_motion_segment(replacement=[drivable_route((0.,0.),(12.,0.))])
        self.tick()
        n.start_segment = Mock()
        new,new_result = self.accept()
        new_result.set_result(S(status=5)); self.tick(2)
        self.assertTrue(n.follow_hold)
        old_result.set_result(S(status=5)); self.tick(2)
        self.assertFalse(n.follow_hold)
        old.cancel_goal_async.assert_called()
        new.cancel_goal_async.assert_called()
        n.start_segment.assert_called_once_with()

    def test_query_switch_preserves_uncertain_motion_fence(self):
        n = self.node
        n.send_motion_segment(); self.tick()
        n.on_query(String(data='new target'))
        n.start_segment = Mock()
        handle,result = self.accept()
        self.assertTrue(n.follow_hold)
        self.assertEqual(n.query,'new target')
        result.set_result(S(status=4)); self.tick(2)
        self.assertFalse(n.object_found)
        self.assertFalse(n.follow_hold)

    def test_stale_input_cancels_and_waits_for_actual_terminal(self):
        n = self.node
        n.send_motion_segment(); handle,result = self.accept()
        n.odom_received_at = self.wall-2.
        n.monitor_inputs()
        self.assertTrue(n.follow_hold)
        handle.cancel_goal_async.assert_called_once_with()
        result.set_result(S(status=5))
        n.start_segment = Mock(); self.tick(2)
        self.assertFalse(n.follow_hold)

    def test_disable_enable_cannot_bypass_unresolved_acceptance(self):
        n = self.node
        n.send_motion_segment()
        n.on_parameters([S(name='follow_path',value=False)])
        n.on_parameters([S(name='follow_path',value=True)])
        self.assertTrue(n.follow_hold)
        n.send_motion_segment()
        self.assertEqual(len(self.responses),1)

    def test_disabled_mode_still_requires_fresh_physical_stop_evidence(self):
        n = self.node
        n.send_motion_segment(); _,result = self.accept()
        n.on_parameters([S(name='follow_path',value=False)])
        result.set_result(S(status=5))
        for _ in range(2):
            self.wall += 1.; self.fixture.now += 10**9
            self.refresh(); n.native_status_at = self.wall-2.
            n.monitor_inputs()
        self.assertTrue(n.follow_hold)

    def test_live_feedback_does_not_expire_a_long_valid_action(self):
        n = self.node
        n.send_motion_segment(); self.accept()
        for _ in range(35):
            self.feedback[-1](S())
            self.tick(1)
        self.assertTrue(n.following)
        self.assertFalse(n.follow_hold)

    def test_old_feedback_cannot_extend_new_action_deadline(self):
        n = self.node
        n.send_motion_segment(); _,result = self.accept()
        n.start_segment = Mock(); n.odom.pose.pose.position.x = 6.
        result.set_result(S(status=4))
        n.motion_queue = [route((6.,0.),(12.,0.))]
        n.send_motion_segment(); self.accept()
        for _ in range(12):
            self.feedback[0](S()); self.tick(1)
        self.assertTrue(n.follow_hold)
        self.assertEqual(n.last_status['reason'],'follow_result_timeout')

    def test_endpoint_wait_is_bounded_even_with_continuing_feedback(self):
        n = self.node
        n.send_motion_segment(); self.accept()
        n.odom.pose.pose.position.x = 6.
        for _ in range(7):
            self.feedback[-1](S()); self.tick(1)
        self.assertTrue(n.follow_hold)
        self.assertEqual(n.last_status['reason'],'follow_result_timeout')

    def test_duplicate_acceptance_registers_one_result_callback(self):
        n = self.node
        n.send_motion_segment(); handle,_ = self.accept()
        n.on_follow_accepted(self.responses[-1],n.epoch,(6.,0.),n.active_follow_serial)
        handle.get_result_async.assert_called_once_with()
        handle.cancel_goal_async.assert_not_called()

    def service(self):
        responses = []
        def dispatch(_):
            future = Future(); responses.append(future); return future
        self.node.graph_reset_client.service_is_ready.return_value = True
        self.node.graph_reset_client.call_async.side_effect = dispatch
        self.node.request_graph_reset(); self.node.retry_graph_reset()
        return responses

    def separate_goal_reset_services(self):
        n=self.node; responses=dict(release=[],search=[])
        for attr,key in (('graph_reset_client','release'),('graph_search_reset_client','search')):
            client=Mock();client.service_is_ready.return_value=True
            def dispatch(_,key=key):
                future=Future();responses[key].append(future);return future
            client.call_async.side_effect=dispatch;setattr(n,attr,client)
        return responses

    def request_search_reset(self):
        import inspect
        self.assertIn('reset_search',inspect.signature(self.node.request_graph_reset).parameters)
        self.node.request_graph_reset(reset_search=True)
        self.node.retry_graph_reset()

    def test_goal_reset_clients_bind_distinct_server_semantics(self):
        from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
        from rclpy.node import Node
        with patch.object(Node,'create_client',return_value=Mock()) as create:
            n=GraphPathBridge()
        try:
            services=[c.args[1] for c in create.call_args_list]
            self.assertIn('/spot1/graphnav_planner/release_goal',services)
            self.assertIn('/spot1/graphnav_planner/clear_goal',services)
        finally:n.destroy_node()

    def test_goal_reset_new_query_uses_full_search_reset(self):
        n=self.node; replies=self.separate_goal_reset_services()
        n.on_query(String(data='new target'))
        self.assertEqual(len(replies['search']),1)
        self.assertFalse(replies['release'])
        replies['search'][0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_goal_reset_completed_search_then_release_uses_independent_budget(self):
        n=self.node; replies=self.separate_goal_reset_services()
        self.request_search_reset();replies['search'][0].set_result(S(success=True))
        n.request_graph_reset();n.retry_graph_reset()
        self.assertEqual(len(replies['release']),1)
        self.assertEqual(n.graph_reset_attempts,1)
        replies['release'][0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_goal_reset_pending_search_cannot_be_downgraded_by_release(self):
        n=self.node; replies=self.separate_goal_reset_services()
        self.request_search_reset();request=n.graph_reset_request
        for _ in range(3):n.request_graph_reset();n.retry_graph_reset()
        self.assertIs(n.graph_reset_request,request)
        self.assertFalse(replies['release'])
        self.assertEqual(len(replies['search']),1)
        replies['search'][0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_goal_reset_promotion_drains_both_service_types_in_either_order(self):
        n=self.node; replies=self.separate_goal_reset_services()
        for order in (('release','search'),('search','release')):
            n.request_graph_reset();n.retry_graph_reset()
            old_serial=n.graph_reset_current_serial
            self.request_search_reset()
            self.assertNotEqual(n.graph_reset_current_serial,old_serial)
            first,second=order
            replies[first][-1].set_result(S(success=True))
            self.assertTrue(n.query_reset_pending)
            replies[second][-1].set_result(S(success=True))
            self.assertFalse(n.query_reset_pending)

    def test_goal_reset_fence_change_preserves_unfinished_full_reset_requirement(self):
        n=self.node; replies=self.separate_goal_reset_services()
        self.request_search_reset()
        n.confirmation_not_before_ns+=1
        n.request_graph_reset();n.retry_graph_reset()
        self.assertEqual(len(replies['search']),2)
        self.assertFalse(replies['release'])
        replies['search'][1].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        replies['search'][0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_goal_reset_missing_release_service_never_falls_back_to_full_reset(self):
        n=self.node; replies=self.separate_goal_reset_services()
        n.graph_reset_client.service_is_ready.return_value=False
        n.request_graph_reset();self.tick(3)
        self.assertFalse(replies['release'])
        self.assertFalse(replies['search'])
        self.assertTrue(n.query_reset_pending)
        self.assertEqual(n.last_status['status'],'graph_reset_unavailable')
        self.assertEqual(n.last_status.get('operation'),'release_goal')

    def test_goal_reset_late_failed_release_cannot_undo_full_search_success(self):
        n=self.node; replies=self.separate_goal_reset_services()
        n.request_graph_reset();n.retry_graph_reset()
        self.request_search_reset()
        replies['search'][0].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        replies['release'][0].set_result(S(success=False))
        self.assertFalse(n.query_reset_pending)
        self.assertFalse(n.graph_reset_exhausted)

    def test_successful_visible_releases_have_independent_clear_budgets(self):
        n=self.node;responses=self.service()
        responses[-1].set_result(S(success=True))
        for index in range(5):
            n.expected=(8.,0.)
            n.visible_approach_active=True
            n.release_visible_approach('visible_target_route_timeout')
            self.assertEqual(len(responses),index+2)
            responses[-1].set_result(S(success=True))
            self.assertFalse(n.query_reset_pending)
            self.assertFalse(n.graph_reset_exhausted)

    def test_duplicate_clear_intent_preserves_current_request_authority(self):
        n=self.node;responses=self.service()
        request=n.graph_reset_request
        n.request_graph_reset()
        self.assertIs(n.graph_reset_request,request)
        n.retry_graph_reset()
        self.assertEqual(len(responses),1)
        responses[0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_duplicate_clear_intents_cannot_renew_unanswered_response_budget(self):
        n=self.node;responses=self.service()
        for _ in range(25):
            n.request_graph_reset()
            self.tick(1)
        self.assertEqual(len(responses),3)
        self.assertTrue(n.query_reset_pending)
        self.assertTrue(n.graph_reset_exhausted)

    def test_changed_scope_gets_new_budget_but_still_drains_old_requests(self):
        n=self.node;responses=self.service()
        n.query_generation+=1
        n.request_graph_reset();n.retry_graph_reset()
        self.assertEqual(len(responses),2)
        responses[1].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        responses[0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_service_budget_is_bounded_and_final_late_ack_can_recover(self):
        n = self.node; responses = self.service()
        self.tick(25)
        self.assertEqual(len(responses),3)
        self.assertFalse(n.query_reset_inflight)
        self.assertEqual(n.last_status['status'],'graph_reset_failed')
        self.assertTrue(n.query_reset_pending)
        for response in responses:
            response.set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_new_ack_waits_for_prior_service_to_drain_before_destination(self):
        n = self.node; responses = self.service()
        goal = deepcopy(n.route.poses[-1])
        n.relay_destination(goal)
        self.tick(7)
        responses[-1].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        n.waypoint_pub.publish.assert_not_called()
        responses[0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)
        n.waypoint_pub.publish.assert_called_once()

    def test_query_switch_drains_old_clear_before_opening_new_goal(self):
        n = self.node; responses = self.service()
        n.on_query(String(data='new target'))
        responses[-1].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        responses[0].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)
        self.assertEqual(n.query,'new target')

    def test_old_duplicate_service_response_cannot_clear_new_intent(self):
        n = self.node; responses = self.service()
        serial = n.graph_reset_serial
        responses[0].set_result(S(success=True))
        n.request_graph_reset(); n.retry_graph_reset()
        n.on_graph_reset_done(responses[0],n.query_generation,n.confirmation_not_before_ns,serial)
        self.assertTrue(n.query_reset_pending)
        self.assertTrue(n.query_reset_inflight)

    def test_service_exception_is_not_proof_prior_clear_has_finished(self):
        n = self.node; responses = self.service()
        responses[0].set_exception(RuntimeError('transport lost'))
        self.tick(2)
        responses[-1].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        self.assertGreater(len(n.graph_reset_waiting),0)

    def test_service_rejection_retries_and_success_opens_fence(self):
        n = self.node; responses = self.service()
        responses[0].set_result(S(success=False))
        self.tick(2)
        responses[-1].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_physical_gate_brakes_and_rejects_commands_until_ordered_release(self):
        from geometry_msgs.msg import Twist
        from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
        n = self.node; driver = CmdVelToControl()
        try:
            driver.pub = Mock()
            driver._safety_clock = S(now=lambda:S(nanoseconds=int(self.wall*1e9)))
            driver.enabled = True
            driver.last_state_time = driver.last_command_time = self.wall
            driver.native_control_ok = True; driver.native_control_at = self.wall
            driver.native_sample_age_at_receipt = 0.
            command = Twist(); command.linear.x = 1.2
            driver.on_cmd_vel(command)
            n.hold_pub.publish.side_effect = driver.on_execution_hold
            n.send_motion_segment(); self.tick()
            self.assertEqual(driver.pub.publish.call_args.args[0].brake,1.)
            driver.on_cmd_vel(command)
            self.assertEqual(driver.target_speed,0.)
            old = String(data=json.dumps(dict(owner=n.hold_owner,revision=n.hold_revision-1,
                                              held=False,wall_time=time.time())))
            driver.on_execution_hold(old)
            self.assertTrue(driver.execution_hold.held)
            self.responses[-1].set_result(Mock(accepted=False)); self.tick(2)
            self.assertFalse(driver.execution_hold.held)
            self.assertEqual(driver.target_speed,0.)
        finally:
            driver.destroy_node()

    def test_hold_release_is_republished_if_first_delivery_is_lost(self):
        n = self.node
        n.send_motion_segment(); self.tick()
        self.responses[-1].set_result(Mock(accepted=False))
        n.start_segment = Mock(); self.tick(2)
        count = n.hold_pub.publish.call_count
        self.tick(1)
        self.assertGreater(n.hold_pub.publish.call_count,count)
        self.assertFalse(json.loads(n.hold_pub.publish.call_args.args[0].data)['held'])

    def test_hold_receiver_rejects_stale_malformed_and_reordered_release(self):
        from wildos_nav2_adapter.execution_hold import ExecutionHoldReceiver
        receiver = ExecutionHoldReceiver()
        def message(revision=1,held=True,wall=None,**extra):
            return String(data=json.dumps(dict(owner='one',revision=revision,held=held,
                                               wall_time=time.time() if wall is None else wall,**extra)))
        self.assertTrue(receiver.receive(message()))
        for value in (String(data='bad json'),message(2,False,time.time()-2.),
                      message(2,False,time.time()+.1),message(0,False),message(True,False)):
            self.assertFalse(receiver.receive(value))
            self.assertTrue(receiver.held)
        self.assertTrue(receiver.receive(message(2,False)))
        self.assertFalse(receiver.receive(message(1,True)))
        self.assertFalse(receiver.held)

    def test_ideal_driver_uses_same_hold_without_changing_motion_profile(self):
        import ast
        import threading
        from pathlib import Path
        import test_kinematic_profile_status as profile
        from wildos_nav2_adapter.execution_hold import ExecutionHoldReceiver
        from wildos_nav2_adapter.drive_math import bounded_cmd_velocity
        from geometry_msgs.msg import Twist
        source = Path(__file__).parents[1]/'wildos_nav2_adapter/kinematic_vehicle_control.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='KinematicVehicleControl')
        methods = [n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in
                   ('on_execution_hold','on_command','tick')]
        namespace = dict(profile.namespace,bounded_cmd_velocity=bounded_cmd_velocity)
        exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),str(source),'exec'),namespace)
        driver = S(command_lock=threading.Lock(),command=(1.2,.3),command_at=self.wall,
            enabled=True,execution_hold=ExecutionHoldReceiver(),carrier_model='ideal_point',
            minimum_turning_radius=0.,max_angular_rate=.8,rpc_worker=Mock(),
            last_state_received_at=self.wall,last_state_stamp_ns=123,measured_velocity=[0.,0.,0.],
            native=Mock(),control=Mock(),get_clock=self.node.get_clock)
        driver.rpc_worker.snapshot.return_value = dict(data={},telemetry_ok=True,
            telemetry_age_sec=0.,rpc_wall_time=time.time(),actuator_ok=True,error=None)
        driver.on_execution_hold = lambda msg: namespace['on_execution_hold'](driver,msg)
        self.node.hold_pub.publish.side_effect = driver.on_execution_hold
        self.node.send_motion_segment(); self.tick()
        command = Twist(); command.linear.x = 1.2
        namespace['on_command'](driver,command)
        namespace['tick'](driver)
        self.assertEqual(driver.command,(0.,0.))
        driver.rpc_worker.set_command.assert_called_with(0.,0.)
        self.assertEqual(driver.carrier_model,'ideal_point')
        self.assertFalse(json.loads(driver.control.publish.call_args.args[0].data)['actuating'])

    def test_confirmation_fence_change_starts_distinct_clear_intent(self):
        n = self.node; responses = self.service()
        old_serial = n.graph_reset_current_serial
        n.confirmation_not_before_ns += 1
        n.retry_graph_reset()
        self.assertNotEqual(n.graph_reset_current_serial,old_serial)
        self.assertEqual(n.graph_reset_attempts,1)
        responses[0].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        responses[-1].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

    def test_watchdogs_use_steady_clock_even_if_ros_clock_stops(self):
        from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
        from rclpy.node import Node
        from rclpy.clock import Clock,ClockType
        with patch('wildos_nav2_adapter.graph_path_bridge.Clock',wraps=Clock) as clock, \
                patch.object(Node,'create_timer',return_value=S()) as timer:
            bridge = GraphPathBridge()
        try:
            monitors = [c for c in timer.call_args_list if c.args[1] in
                        (bridge.monitor_inputs,bridge.retry_graph_reset)]
            self.assertEqual(len(monitors),2)
            for call in monitors:
                self.assertIs(call.kwargs['clock'],bridge.transaction_clock)
            self.assertEqual(clock.call_args.kwargs['clock_type'],ClockType.STEADY_TIME)
        finally:
            bridge.destroy_node()

    def test_duplicate_terminal_cannot_clear_next_request(self):
        n = self.node
        n.send_motion_segment()
        _,result = self.accept()
        epoch,serial = n.epoch,n.active_follow_serial
        n.start_segment = Mock()
        n.odom.pose.pose.position.x = 6.
        result.set_result(S(status=4))
        n.motion_queue = [route((6.,0.),(12.,0.))]
        n.send_motion_segment()
        n.on_follow_result(result,epoch,(6.,0.),serial)
        self.assertTrue(n.following)
        self.assertEqual(len(n.motion_queue),1)
        self.assertIsNone(n.active_follow_serial)

    def test_missing_terminal_response_times_out_without_visual_success(self):
        n = self.node
        n.send_motion_segment()
        handle,_ = self.accept()
        n.odom.pose.pose.position.x = 6.
        self.tick()
        self.assertFalse(n.following)
        self.assertEqual(n.last_status['reason'], 'follow_result_timeout')
        handle.cancel_goal_async.assert_called()
        self.assertFalse(n.object_found)

    def test_replacement_ack_gap_does_not_resume_old_action(self):
        n = self.node
        n.send_motion_segment()
        _,result = self.accept()
        n.send_motion_segment(replacement=[drivable_route((0.,0.),(12.,0.))])
        n.odom.pose.pose.position.x = 6.
        result.set_result(S(status=4))
        n.start_segment = Mock()
        self.tick()
        self.assertFalse(n.replacement_pending)
        self.assertFalse(n.following)
        self.assertIsNone(n.deferred_follow_result)
        self.assertEqual(n.last_status['reason'],'follow_replacement_timeout')
        n.start_segment.assert_not_called()

    def test_clear_service_response_timeout_retries_with_distinct_requests(self):
        n = self.node
        responses = []
        def dispatch(_):
            future = Future(); responses.append(future); return future
        n.graph_reset_client.service_is_ready.return_value = True
        n.graph_reset_client.call_async.side_effect = dispatch
        n.query_reset_pending = True
        n.retry_graph_reset()
        self.tick(7)
        self.assertGreaterEqual(len(responses),2)
        self.assertTrue(n.query_reset_pending)
        self.assertTrue(n.query_reset_inflight)
        responses[0].set_result(S(success=True))
        self.assertTrue(n.query_reset_pending)
        self.assertTrue(n.query_reset_inflight)
        responses[-1].set_result(S(success=True))
        self.assertFalse(n.query_reset_pending)

