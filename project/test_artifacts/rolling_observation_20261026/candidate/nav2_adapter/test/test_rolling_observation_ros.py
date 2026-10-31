import unittest
from unittest.mock import Mock,patch
import test_visible_approach_ros as visible_fixtures
import test_graph_path_bridge_routing as route_fixtures
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge

class RollingObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):visible_fixtures.VisibleApproachTests.setUpClass()
    @classmethod
    def tearDownClass(cls):visible_fixtures.VisibleApproachTests.tearDownClass()
    def setUp(self):
        self.f=visible_fixtures.VisibleApproachTests();self.f.setUp()
        self.addCleanup(self.f.tearDown)
    def moving(self):
        n=self.f.prepare_moving_visible_goal();n.full_path_execution=True
        return n
    def test_two_metre_new_observation_can_prepare_next_leg_early(self):
        n=self.moving();n.visible_hint['point']=[10.,0.]
        epoch=n.epoch;handle=n.follow_handle
        self.assertTrue(n.advance_visible_goal())
        self.assertEqual(n.expected,(10.,0.));self.assertEqual(n.epoch,epoch)
        handle.cancel_goal_async.assert_not_called()
    def test_fresh_hint_after_completed_leg_keeps_the_observation_session(self):
        n=self.moving();n.following=False;n.follow_handle=None
        n.odom.pose.pose.position.x=8.;n.visible_hint['point']=[11.,0.]
        n.reset=Mock();n.request_graph_reset=Mock();epoch=n.epoch
        n.finish_visible_approach()
        n.reset.assert_not_called();n.request_graph_reset.assert_not_called()
        self.assertEqual(n.expected,(11.,0.));self.assertEqual(n.epoch,epoch)
        self.assertTrue(n.visible_approach_active);self.assertFalse(n.object_found)
    def test_stale_hint_cannot_refresh_or_extend_observation(self):
        n=self.moving();n.visible_hint['point']=[10.,0.]
        n.visible_hint['observation_stamp_ns']-=3_000_000_000
        self.assertFalse(n.advance_visible_goal());self.assertEqual(n.expected,(8.,0.))
    def test_pending_or_confirmed_route_keeps_its_owner(self):
        for attribute,value in (('pending',True),('query_reset_pending',True),
                                ('active_object_confirmation',{'instance_id':'confirmed'})):
            n=self.moving();n.visible_hint['point']=[10.,0.];setattr(n,attribute,value)
            self.assertFalse(n.advance_visible_goal());self.assertEqual(n.expected,(8.,0.))
            setattr(n,attribute,None if attribute=='active_object_confirmation' else False)
    def test_legacy_mode_keeps_its_existing_update_threshold(self):
        n=self.moving();n.full_path_execution=False;n.visible_hint['point']=[10.,0.]
        self.assertFalse(n.advance_visible_goal())
    def test_completion_cannot_refresh_backwards_or_before_action_acceptance(self):
        n=self.moving();n.following=False;n.follow_handle=None
        n.odom.pose.pose.position.x=8.;n.visible_hint['point']=[5.,0.]
        self.assertFalse(n.advance_visible_goal(completed=True))
        n.visible_hint['point']=[11.,0.];n.follow_request={'serial':99}
        self.assertFalse(n.advance_visible_goal(completed=True))

class FastRecoveryProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):visible_fixtures.VisibleApproachTests.setUpClass()
    @classmethod
    def tearDownClass(cls):visible_fixtures.VisibleApproachTests.tearDownClass()
    def setUp(self):
        with patch.object(route_fixtures,'GraphPathBridge',ContinuousGraphPathBridge):
            route_fixtures.GraphBridgeRoutingTests.setUp(self)
        self.addCleanup(self.node.destroy_node)
        n=self.node;n.full_path_execution=True;n.follow_enabled=True
        n.candidates=[(6.,0.,0.,6.),(4.,0.,0.,4.),(2.,0.,0.,2.)]
        n.candidate_index=0;n.try_candidate=Mock()
        n.active_plan_start=(0.,0.);n.active_plan_failure_context=None
    def test_first_empty_stopped_route_tries_short_probe_before_other_far_goals(self):
        n=self.node;n.candidate_failed('empty_path')
        self.assertEqual(n.candidates[n.candidate_index][:2],(2.,0.))
        self.assertEqual(sum(c[:3]==(2.,0.,0.) for c in n.candidates[1:]),1)
    def test_healthy_following_does_not_receive_a_stopped_probe(self):
        n=self.node;n.following=True;n.extending_path=True
        n.reserve_short_route_probe=Mock(wraps=n.reserve_short_route_probe)
        n.candidate_failed('empty_path')
        self.assertEqual(n.candidates[1][:2],(4.,0.))
    def test_transport_failure_does_not_probe_as_geometry_failure(self):
        n=self.node;n.candidate_failed('planner_error:transport')
        self.assertEqual(n.candidates[1][:2],(4.,0.))
    def test_confirmed_or_memory_goal_is_not_replaced_by_a_probe(self):
        n=self.node;n.expected=(6.,0.);n.candidate_failed('empty_path')
        self.assertEqual(n.candidates[1][:2],(4.,0.))

if __name__=='__main__':unittest.main()
