"""A map wait ends when planning is dispatched, including background planning."""
from concurrent.futures import Future
from copy import deepcopy
import time,unittest
from types import SimpleNamespace as S
from unittest.mock import Mock
import test_execution_mechanism_ros as mechanisms

class PlanningPhaseTests(unittest.TestCase):
    setUp=mechanisms.ExecutionMechanismTests.setUp
    tearDown=mechanisms.ExecutionMechanismTests.tearDown

    def ready(self):
        n=self.node
        n.candidates=[(15.,0.,0.,0.)];n.candidate_index=0
        n.grid_arrived_at=time.monotonic()
        n.plan_client=Mock();n.plan_client.send_goal_async.return_value=Future()
        n.status('waiting_for_costmap_update',candidate_index=0)
        return n

    def test_dispatch_replaces_waiting_label_and_exposes_transaction(self):
        n=self.ready();n.try_candidate()
        self.assertEqual(n.last_status['status'],'planning')
        self.assertEqual(n.last_status['request_serial'],n.active_plan_serial)
        self.assertEqual(n.last_status['candidate_index'],0)
        self.assertFalse(n.last_status['background'])
        self.assertTrue(n.pending);self.assertFalse(n.candidate_waiting_for_map)

    def test_stale_receipt_waits_then_new_map_dispatches(self):
        n=self.ready();n.grid_arrived_at=time.monotonic()-1.;n.try_candidate()
        self.assertEqual(n.last_status['status'],'waiting_for_costmap_update')
        n.plan_client.send_goal_async.assert_not_called()
        self.now+=10_000_000
        grid=deepcopy(n.grid);grid.header.stamp=n.get_clock().now().to_msg()
        n.on_map(grid)
        n.plan_client.send_goal_async.assert_called_once()
        self.assertEqual(n.last_status['status'],'planning')

    def test_background_dispatch_retains_accepted_motion_owner(self):
        n=self.ready();n.following=True;n.follow_handle=Mock();n.active_follow_serial=19
        handle=n.follow_handle;n.extending_path=True
        n.try_candidate()
        self.assertTrue(n.last_status['background'])
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle)
        self.assertEqual(n.active_follow_serial,19)

    def test_immediate_rejection_cannot_be_overwritten_by_late_planning_label(self):
        n=self.ready();future=Future();future.set_result(S(accepted=False))
        n.plan_client.send_goal_async.return_value=future
        n.try_candidate()
        self.assertNotEqual(n.last_status['status'],'planning')
        self.assertFalse(n.pending)

if __name__=='__main__':unittest.main()
