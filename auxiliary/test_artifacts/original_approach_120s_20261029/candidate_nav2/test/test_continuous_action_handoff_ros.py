"""A submitted FollowPath is not an acknowledged old motion owner."""
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as S
from unittest.mock import Mock
import test_continuous_exploration_ros as fixture


class ContinuousActionHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): fixture.ContinuousExplorationTests.setUpClass()
    @classmethod
    def tearDownClass(cls): fixture.ContinuousExplorationTests.tearDownClass()
    def setUp(self):
        self.fixture=fixture.ContinuousExplorationTests(); self.fixture.setUp()
        self.node=self.fixture.moving(); self.addCleanup(self.node.destroy_node)
        self.node.follow_failed=Mock()
        self.node.full_path_execution=True
        self.node.follow_active['path_stamp_ns']=17
    def route(self): return self.fixture.route((20.,0.),'frontier-B')
    def test_new_submission_after_old_terminal_is_not_failed_as_retired_motion(self):
        n=self.node; n.follow_active=n.follow_handle=n.active_follow_serial=None
        request=n.follow_request=dict(serial=8,started_at=0.,replacement=False)
        epoch=n.epoch
        n.on_route(self.route())
        n.follow_failed.assert_not_called()
        self.assertIs(n.follow_request,request); self.assertEqual(n.epoch,epoch)
        self.assertTrue(n.following)
    def test_ack_for_replacement_does_not_make_stale_motion_metadata_its_owner(self):
        n=self.node; n.active_follow_serial=8
        n.follow_active=dict(serial=8,path_stamp_ns=1)
        n.on_route(self.route())
        n.follow_failed.assert_not_called()
    def test_real_old_acknowledged_motion_still_requires_current_direction_progress(self):
        n=self.node; n._controller_path=None
        n.on_route(self.route())
        n.follow_failed.assert_called_once_with('retired_controller_progress_unavailable')
    def test_old_completion_preserves_graph_route_owned_by_pending_plan(self):
        n=self.node;old=n.route;old_endpoint=n.route_endpoint
        n.latest_route=self.route();n.latest_route_complete=True
        n.pending=True;n.odom.pose.pose.position.x=12.
        result=Future();result.set_result(S(status=4))
        n.on_follow_result(result,n.epoch,(12.,0.),7)
        n.follow_failed.assert_not_called()
        self.assertIs(n.route,old);self.assertEqual(n.route_endpoint,old_endpoint)
        self.assertTrue(n.pending);self.assertFalse(n.following)
    def test_old_completion_adopts_latest_route_when_no_plan_is_pending(self):
        n=self.node;n.latest_route=self.route();latest=n.latest_route
        n.latest_route_complete=True;n.pending=False
        n.odom.pose.pose.position.x=12.;n.start_segment=Mock()
        result=Future();result.set_result(S(status=4))
        n.on_follow_result(result,n.epoch,(12.,0.),7)
        self.assertIs(n.route,latest);self.assertEqual(n.route_endpoint,(20.,0.))
        n.start_segment.assert_called_once_with()
