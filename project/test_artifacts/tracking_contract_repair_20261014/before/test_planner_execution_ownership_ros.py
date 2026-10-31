"""Planner transactions must not retire unrelated accepted motion actions."""
from concurrent.futures import Future
from types import SimpleNamespace as S
import time
import unittest
from unittest.mock import Mock
from std_msgs.msg import String
import test_execution_mechanism_ros as mechanisms
from test_graph_path_bridge_routing import route


class PlannerOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.fixture = mechanisms.ExecutionMechanismTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.real_start_segment = self.node.start_segment
        self.node.query = self.node.expected = None
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.responses = []
        def dispatch(_goal):
            future = Future()
            self.responses.append(future)
            return future
        self.node.plan_client.send_goal_async.side_effect = dispatch

    def tearDown(self):
        self.fixture.tearDown()

    def request(self):
        self.node.candidates = [(12., 0., 0., 12.)]
        self.node.candidate_index = 0
        self.node.try_candidate()
        return self.responses[-1]

    def timeout(self):
        self.node.plan_request_started_at = time.monotonic()-self.node.compute_path_timeout-.1
        self.node.recover_timed_out_plan()

    def active_motion(self):
        self.node.motion_queue = [route((0., 0.), (6., 0.))]
        self.node.following = True
        self.node.follow_handle = Mock()
        self.node.active_follow_serial = self.node.follow_request_serial = 7
        self.node.extending_path = True
        self.node.start_segment = Mock()

    def finish_motion(self, status=4):
        self.node.odom.pose.pose.position.x = 6.
        self.node.on_follow_result(Mock(result=Mock(return_value=S(status=status))),
                                  self.node.epoch, (6., 0.), 7)

    def test_extension_timeout_preserves_active_action_and_success(self):
        self.active_motion()
        self.request()
        epoch, handle = self.node.epoch, self.node.follow_handle
        self.timeout()
        self.assertEqual(self.node.epoch, epoch)
        self.assertIs(self.node.follow_handle, handle)
        handle.cancel_goal_async.assert_not_called()
        self.assertFalse(self.node.extending_path)
        self.finish_motion()
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.motion_queue, [])
        self.node.start_segment.assert_called_once_with()

    def test_success_before_extension_timeout_resumes_handoff_once(self):
        self.active_motion()
        self.request()
        self.finish_motion()
        self.node.start_segment.assert_not_called()
        self.timeout()
        self.assertFalse(self.node.extending_path)
        self.assertFalse(self.node.pending)
        self.node.start_segment.assert_called_once_with()

    def test_new_route_during_extension_timeout_keeps_action_ownership(self):
        self.active_motion()
        self.request()
        newest = route((0., 0.), (60., 0.))
        self.node.latest_route = newest
        self.timeout()
        self.node.start_segment.assert_not_called()
        self.finish_motion()
        self.assertIs(self.node.route, newest)
        self.node.start_segment.assert_called_once_with()

    def test_success_before_ordinary_extension_rejection_resumes_handoff(self):
        self.active_motion()
        self.request()
        self.finish_motion()
        self.node.candidate_failed('empty_path')
        self.assertFalse(self.node.extending_path)
        self.node.start_segment.assert_called_once_with()

    def test_abort_after_timeout_is_processed(self):
        self.active_motion()
        self.request()
        self.timeout()
        self.node.follow_failed = Mock()
        self.finish_motion(6)
        self.assertFalse(self.node.following)
        self.node.follow_failed.assert_called_once_with('follow_aborted', action_status=6)

    def test_completed_action_timeout_dispatches_fresh_foreground_plan(self):
        self.active_motion()
        old = self.request()
        self.finish_motion()
        self.node.start_segment = self.real_start_segment
        self.timeout()
        self.assertEqual(len(self.responses), 2)
        self.assertTrue(self.node.pending)
        self.assertFalse(self.node.extending_path)
        self.assertFalse(self.node.following)
        fresh_serial = self.node.active_plan_serial
        late = Mock(accepted=True)
        old.set_result(late)
        late.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.node.active_plan_serial, fresh_serial)

    def test_completed_action_timeout_does_not_plan_with_stale_pose(self):
        self.active_motion()
        self.request()
        self.finish_motion()
        self.node.start_segment = self.real_start_segment
        self.node.odom_received_at = time.monotonic()-self.node.odom_timeout-.1
        self.timeout()
        self.assertEqual(self.node.last_status['status'], 'stale_odom')
        self.assertEqual(len(self.responses), 1)
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertFalse(self.node.pending)
        self.assertFalse(self.node.extending_path)

    def test_completed_action_timeout_does_not_plan_after_actuator_fault(self):
        self.active_motion()
        self.request()
        self.finish_motion()
        self.node.start_segment = self.real_start_segment
        self.node.native_status['actuator_ok'] = False
        self.timeout()
        self.assertEqual(self.node.last_status['status'], 'execution_unavailable')
        self.assertEqual(len(self.responses), 1)
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertFalse(self.node.following)
        self.assertFalse(self.node.pending)
        self.assertFalse(self.node.object_found)

    def test_extension_failure_does_not_compete_with_pending_replacement(self):
        self.active_motion()
        self.request()
        self.node.replacement_pending = True
        self.finish_motion()
        self.timeout()
        self.assertTrue(self.node.replacement_pending)
        self.assertIsNotNone(self.node.deferred_follow_result)
        self.node.start_segment.assert_not_called()

    def test_late_acceptance_cannot_claim_reused_candidate_index(self):
        old = self.request()
        self.timeout()
        current = self.request()
        handle = Mock(accepted=True)
        old.set_result(handle)
        handle.cancel_goal_async.assert_called_once_with()
        handle.get_result_async.assert_not_called()
        self.assertFalse(current.done())
        self.assertTrue(self.node.pending)

    def test_late_response_exception_does_not_fail_new_request(self):
        old = self.request()
        self.timeout()
        self.request()
        self.node.candidate_failed = Mock()
        old.set_exception(RuntimeError('late response'))
        self.node.candidate_failed.assert_not_called()

    def test_late_result_cannot_replace_new_request_at_same_index(self):
        old = self.request()
        result = Future()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        old.set_result(handle)
        self.timeout()
        self.request()
        self.node.candidate_failed = Mock()
        result.set_exception(RuntimeError('late result'))
        self.node.candidate_failed.assert_not_called()
        self.assertTrue(self.node.pending)

    def test_terminal_result_is_consumed_once(self):
        response = self.request()
        result = Future()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        response.set_result(handle)
        serial = self.node.active_plan_serial
        self.node.execute_checked_path = Mock()
        self.node.validate_path = Mock(return_value=None)
        result.set_result(S(status=4, result=S(path=route((0., 0.), (12., 0.)))))
        self.node.on_plan_result(result, self.node.epoch, 0, serial)
        self.node.execute_checked_path.assert_called_once()

    def test_duplicate_acceptance_registers_one_result_callback(self):
        response = self.request()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = Future()
        response.set_result(handle)
        self.node.on_plan_accepted(response, self.node.epoch, 0, self.node.active_plan_serial)
        handle.get_result_async.assert_called_once_with()
        handle.cancel_goal_async.assert_not_called()

    def test_query_reset_still_retires_planner_and_motion_callbacks(self):
        self.active_motion()
        old = self.request()
        epoch = self.node.epoch
        self.node.on_query(String(data='new target'))
        handle = Mock(accepted=True)
        old.set_result(handle)
        handle.cancel_goal_async.assert_called_once_with()
        future = Mock()
        self.node.on_follow_result(future, epoch, (6., 0.), 7)
        future.result.assert_not_called()
        self.assertFalse(self.node.following)
