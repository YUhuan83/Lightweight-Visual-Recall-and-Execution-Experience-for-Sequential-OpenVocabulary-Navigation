"""Confirmed target ownership survives partial-route observation boundaries."""
import unittest
from types import SimpleNamespace as S
import test_confirmed_progress_ros as progress


class ConfirmedGraphHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):progress.ConfirmedProgressTests.setUpClass()
    @classmethod
    def tearDownClass(cls):progress.ConfirmedProgressTests.tearDownClass()
    def setUp(self):
        self.f=progress.ConfirmedProgressTests();self.f.setUp();self.addCleanup(self.f.tearDown)
        self.n=self.f.node;original=self.n.get_parameter
        self.n.get_parameter=lambda name:S(value=True) if name=='graph_destination_approach' else original(name)
    def test_far_confirmed_route_has_one_owner_not_a_racing_unlabelled_pose(self):
        self.f.pending((40.,0.,2.));self.n.retry_confirmation(force=True)
        self.n.publisher.publish.assert_not_called()
        self.n.confirmation_pub.publish.assert_called_once()
    def test_partial_frontier_end_is_not_a_final_target_observation_wait(self):
        b=self.f.fixture.bridge;b.expected=b.confirmed_route_goal=(30.,0.);b.route_complete=False
        b.active_object_confirmation=dict(query=b.query,object_position=[40.,0.],instance_id='A')
        b.await_object_observation()
        self.assertIsNone(b.observation_wait_until)
        self.assertEqual(b.expected,(30.,0.));self.assertEqual(b.active_object_confirmation['instance_id'],'A')
        self.assertFalse(b.object_found)
    def test_complete_target_route_still_requires_fresh_final_observation(self):
        b=self.f.fixture.bridge;b.expected=b.confirmed_route_goal=(30.,0.);b.route_complete=True
        b.await_object_observation()
        self.assertIsNotNone(b.observation_wait_until);self.assertFalse(b.object_found)
    def test_ordinary_unconfirmed_waypoint_keeps_existing_observation_contract(self):
        b=self.f.fixture.bridge;b.expected=(8.,0.);b.route_complete=False
        b.await_object_observation();self.assertIsNotNone(b.observation_wait_until)
