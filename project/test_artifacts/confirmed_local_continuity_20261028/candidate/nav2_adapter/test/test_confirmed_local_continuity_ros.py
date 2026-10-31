"""A known object keeps its identity through rolling observation waypoints."""
import unittest
import json
from copy import deepcopy
from types import SimpleNamespace as S
from unittest.mock import Mock
import test_confirmed_graph_handoff_ros as graph_tests


class ConfirmedLocalContinuityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):graph_tests.ConfirmedGraphHandoffTests.setUpClass()
    @classmethod
    def tearDownClass(cls):graph_tests.ConfirmedGraphHandoffTests.tearDownClass()
    def setUp(self):
        self.f=graph_tests.ConfirmedGraphHandoffTests();self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.b=self.f.f.fixture.bridge
        self.b.expected=self.b.confirmed_route_goal=(8.,0.)
        self.b.route_complete=True
        self.b.active_object_confirmation=dict(query=self.b.query,instance_id='A',
            object_position=[40.,0.,2.],standoff=[8.,0.],standoff_clamped=True,
            standoff_intermediate=True)

    def test_clamped_confirmed_waypoint_is_not_final_observation(self):
        self.b.await_object_observation()
        self.assertIsNone(self.b.observation_wait_until)
        self.assertEqual(self.b.expected,(8.,0.))
        self.assertEqual(self.b.active_object_confirmation['instance_id'],'A')
        self.assertFalse(self.b.object_found)

    def test_queued_next_approach_precedes_partial_route_wait(self):
        self.b.route_complete=False
        self.b.consume_confirmed_approach=Mock(return_value=True)
        self.b.await_object_observation()
        self.b.consume_confirmed_approach.assert_called_once_with()
        self.assertIsNone(self.b.observation_wait_until)

    def test_queued_next_approach_precedes_clamped_waypoint_wait(self):
        self.b.consume_confirmed_approach=Mock(return_value=True)
        self.b.await_object_observation()
        self.b.consume_confirmed_approach.assert_called_once_with()
        self.assertIsNone(self.b.observation_wait_until)

    def test_final_unclamped_goal_still_waits_for_fresh_arrival_evidence(self):
        self.b.active_object_confirmation['standoff_clamped']=False
        self.b.active_object_confirmation['standoff_intermediate']=False
        self.b.await_object_observation()
        self.assertIsNotNone(self.b.observation_wait_until)
        self.assertFalse(self.b.object_found)

    def test_body_clearance_adjusted_final_goal_is_not_a_temporary_waypoint(self):
        self.b.active_object_confirmation['standoff_intermediate']=False
        self.b.await_object_observation()
        self.assertIsNotNone(self.b.observation_wait_until)
        self.assertFalse(self.b.object_found)

    def test_unconfirmed_waypoint_does_not_acquire_confirmed_retention(self):
        self.b.confirmed_route_goal=None
        self.b.active_object_confirmation=None
        self.b.await_object_observation()
        self.assertIsNotNone(self.b.observation_wait_until)
        self.assertFalse(self.b.object_found)

    def test_stale_clamped_metadata_for_another_waypoint_cannot_suppress_final_wait(self):
        self.b.active_object_confirmation['standoff']=[4.,0.]
        self.b.await_object_observation()
        self.assertIsNotNone(self.b.observation_wait_until)
        self.assertFalse(self.b.object_found)

    def test_recorded_local_confirmed_goal_uses_only_identified_confirmation(self):
        meta,target=self.f.f.snapshot();n=self.f.n
        original=n.get_parameter
        n.get_parameter=lambda name:S(value=False) if name=='graph_destination_approach' else original(name)
        n.pending_confirmation=(target,deepcopy(meta['confirmation']))
        n.retry_confirmation(force=True)
        data=json.loads(n.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(data['approach_goal_ready'])
        self.assertTrue(data['standoff_clamped'])
        self.assertTrue(data['standoff_intermediate'])
        self.assertTrue(data['instance_id'])
        n.publisher.publish.assert_not_called()


if __name__=='__main__':unittest.main()
