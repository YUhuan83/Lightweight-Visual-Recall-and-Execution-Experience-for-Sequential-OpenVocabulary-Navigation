"""Execution may not complete or learn after the physical actuator fails."""
from concurrent.futures import Future
import json
import time
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
from std_msgs.msg import String
import test_execution_mechanism_ros as fixtures
from test_graph_path_bridge_routing import route
from wildos_nav2_adapter.execution_feedback import native_command_braking


class ExecutionTransportHealthTests(unittest.TestCase):
    def setUp(self):
        fixtures.ExecutionMechanismTests.setUp(self)

    def tearDown(self):
        fixtures.ExecutionMechanismTests.tearDown(self)

    def native(self, **changes):
        value = dict(wall_time=time.time(), speed=0., actuator_ok=True,
                     telemetry_ok=True, telemetry_age_sec=0.,
                     control_acknowledged=True, actual={'brake':1.}, sent={'brake':1.})
        value.update(changes)
        self.node.on_native_status(String(data=json.dumps(value)))

    def test_failed_actuator_at_endpoint_cannot_report_route_done(self):
        fixtures.ExecutionMechanismTests.move(self, 40.)
        self.native(actuator_ok=False, control_acknowledged=False)
        self.node.start_segment()
        self.assertEqual(self.node.last_status['status'], 'execution_unavailable')
        self.assertFalse(self.node.object_found)
        self.assertNotEqual(self.node.last_status['status'], 'route_done')

    def test_late_success_after_fault_cannot_complete_or_learn_or_change_cusp(self):
        for sections in (1, 2):
            with self.subTest(sections=sections):
                self.node.motion_queue = [route((0.,0.), (40.,0.))]*sections
                self.node.following = True
                handle = self.node.follow_handle = Mock()
                self.node.finish_execution_feedback = Mock()
                self.native(actuator_ok=False, control_acknowledged=False)
                fixtures.ExecutionMechanismTests.move(self, 40.)
                result = Future(); result.set_result(S(status=4))
                self.node.on_follow_result(result, self.node.epoch, (40.,0.))
                self.assertEqual(self.node.last_status['status'], 'execution_unavailable')
                self.assertFalse(self.node.motion_queue)
                self.assertFalse(self.node.following)
                self.assertFalse(any(call.args and call.args[0] is True
                                     for call in self.node.finish_execution_feedback.call_args_list))
                handle.cancel_goal_async.assert_called_once()

    def test_duplicate_native_status_does_not_renew_receipt_age(self):
        with patch('time.time', return_value=1000.), patch('time.monotonic', return_value=100.):
            self.native()
        received_at = self.node.native_status_at
        with patch('time.time', return_value=1000.3), patch('time.monotonic', return_value=100.3):
            self.node.on_native_status(String(data=json.dumps(self.node.native_status)))
        self.assertEqual(self.node.native_status_at, received_at)

    def test_unacknowledged_brake_intent_is_not_delivered_braking_evidence(self):
        self.assertFalse(native_command_braking(dict(sent={'brake':1.}, actuator_ok=False,
                                                    control_acknowledged=False)))
        self.assertTrue(native_command_braking(dict(sent={'brake':1.}, actuator_ok=True,
                                                   control_acknowledged=True)))

    def test_missing_unacknowledged_and_cumulatively_aged_status_block_dispatch(self):
        for change in ('missing', 'unacknowledged', 'aged', 'invalid'):
            with self.subTest(change=change):
                self.node.motion_queue = [route((0.,0.), (12.,0.))]
                if change == 'missing':
                    self.node.native_status = self.node.native_status_at = None
                else:
                    self.native(control_acknowledged=change!='unacknowledged',
                                telemetry_age_sec=float('nan') if change=='invalid' else .3)
                    if change=='aged':
                        self.node.native_status_at = time.monotonic()-.25
                self.node.send_motion_segment()
                self.node.follow_client.send_goal_async.assert_not_called()
                self.assertEqual(self.node.last_status['status'], 'execution_unavailable')
                self.assertEqual(self.node.expected, (40.,0.))
                self.assertEqual(self.node.recovery_attempts, 0)

    def test_monitor_invalidates_old_result_and_preserves_target_then_recovers(self):
        self.native()
        self.node.following = True; handle = self.node.follow_handle = Mock()
        self.node.motion_queue = [route((0.,0.), (12.,0.))]
        epoch = self.node.epoch
        self.node.native_status_at = time.monotonic()-1.
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once()
        future = Future(); future.set_result(S(status=4))
        self.node.on_follow_result(future, epoch, (12.,0.))
        self.assertEqual(self.node.last_status['status'], 'execution_unavailable')
        self.assertEqual(self.node.expected, (40.,0.))
        self.node.start_segment = Mock()
        self.node.plan_client.wait_for_server = Mock(return_value=True)
        self.native()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once_with()
        self.assertEqual(self.node.recovery_attempts, 0)

    def test_delayed_acceptance_is_cancelled_if_transport_aged_during_request(self):
        self.native()
        self.node.native_status_at = time.monotonic()-1.
        handle = Mock(); handle.accepted = True
        future = Future(); future.set_result(handle)
        self.node.on_follow_accepted(future, self.node.epoch, (12.,0.))
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.node.last_status['status'], 'execution_unavailable')
        self.assertFalse(self.node.following)

    def test_read_only_and_ideal_profiles_keep_their_execution_contracts(self):
        self.node.native_status = self.node.native_status_at = None
        self.node.follow_enabled = False
        self.assertIsNone(self.node.execution_transport_issue())
        self.node.follow_enabled = True; self.node.carrier_model = 'ideal_point'
        self.assertIsNone(self.node.execution_transport_issue())

    def test_transport_recovery_keeps_existing_geometric_backoff_without_counting_failure(self):
        self.node.blocked_endpoint = (40.,0.)
        self.node.blocked_until = time.monotonic()+10.
        self.node.recovery_attempts = 1
        self.node.last_status = dict(status='execution_unavailable')
        self.node.start_segment = Mock()
        self.node.plan_client.wait_for_server = Mock(return_value=True)
        self.native()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()
        self.node.blocked_until = 0.
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once_with()
        self.assertEqual(self.node.recovery_attempts, 1)


if __name__ == '__main__':
    unittest.main()
