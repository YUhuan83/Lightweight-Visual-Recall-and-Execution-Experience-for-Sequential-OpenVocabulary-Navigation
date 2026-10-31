"""Rolling motion must not bypass shared idle retry admission."""
from concurrent.futures import Future
from unittest.mock import Mock
import unittest
import test_continuous_exploration_ros as fixture
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge


class ContinuousRetryAdmissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.ContinuousExplorationTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.ContinuousExplorationTests.tearDownClass()
    def setUp(self):
        self.f=fixture.ContinuousExplorationTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        n=self.n=self.f.moving()
        n.following=False;n.follow_handle=n.follow_active=n.active_follow_serial=None
        n.motion_queue=[];n.pending=False
        n.sync_recovery_progress(n.route)
        self.incoming=self.f.route((12.,0.),'frontier-A')
    def block(self):
        self.n.block_current_route(backoff=4.)
        self.n.last_status=dict(status='follow_failed')
        self.n.start_segment=Mock()
    def test_base_preserves_unexpired_backoff_for_same_route(self):
        self.block();GraphPathBridge.on_route(self.n,self.incoming)
        self.n.start_segment.assert_not_called()
    def test_continuous_preserves_unexpired_backoff_for_same_route(self):
        self.block();self.n.on_route(self.incoming)
        self.n.start_segment.assert_not_called()
    def test_base_preserves_exhausted_retry_budget(self):
        self.block();self.n.recovery_attempts=3
        GraphPathBridge.on_route(self.n,self.incoming)
        self.n.start_segment.assert_not_called()
    def test_continuous_preserves_exhausted_retry_budget(self):
        self.block();self.n.recovery_attempts=3
        self.n.on_route(self.incoming)
        self.n.start_segment.assert_not_called()
    def planner(self):
        n=self.n;n.plan_client=Mock()
        n.plan_client.send_goal_async.return_value=Future()
        return n
    def test_idle_new_intent_does_not_cancel_and_send_its_own_plan_twice(self):
        n=self.planner()
        n.on_route(self.f.route((20.,0.),'frontier-B'))
        self.assertEqual(n.plan_client.send_goal_async.call_count,1)
        self.assertTrue(n.pending)
        self.assertEqual(n._planned_intent,n.intent_key(n.route))
    def test_superseded_idle_compute_reenters_shared_admission(self):
        n=self.planner();old=Mock();n.plan_handle=old
        n.pending=True;n.active_plan_serial=99
        n.on_route(self.f.route((20.,0.),'frontier-B'))
        old.cancel_goal_async.assert_called_once()
        self.assertEqual(n.plan_client.send_goal_async.call_count,1)
        self.assertTrue(n.pending)
        self.assertEqual(n._planned_intent,n.intent_key(n.route))
    def test_new_owner_can_legitimately_clear_old_owner_backoff(self):
        n=self.planner();n.block_current_route(backoff=4.)
        n.recovery_attempts=3;n.last_status=dict(status='follow_failed')
        n.on_route(self.f.route((20.,0.),'frontier-B'))
        self.assertEqual(n.plan_client.send_goal_async.call_count,1)
        self.assertEqual(n.recovery_attempts,0)
        self.assertTrue(n.pending)
    def test_invalid_route_does_not_replace_current_intent(self):
        self.block();old=self.n._desired_intent
        route=self.f.route((20.,0.),'frontier-B');route.header.frame_id='wrong'
        self.n.on_route(route)
        self.n.start_segment.assert_not_called()
        self.assertEqual(self.n._desired_intent,old)
