"""Body-relative arrival across perception, navigation and evidence matching."""
import json
import math
import time
import unittest
from unittest.mock import Mock, patch
from std_msgs.msg import String
from wildos_nav2_adapter import arrival_criteria as criteria
import test_execution_mechanism_ros as execution_fixture
import test_goal_confirmation_ros as perception_fixture


class BodyArrivalTests(unittest.TestCase):
    def setUp(self):
        self.fixture=execution_fixture.ExecutionMechanismTests();self.fixture.setUp()
        self.bridge=self.fixture.node;self.bridge.query='house'
        self.bridge.route_confirmed_target=Mock(return_value=False)
        self.perception_fixture=perception_fixture.GoalConfirmationTests();self.perception_fixture.setUp()
        self.perception=self.perception_fixture.node

    def tearDown(self):
        self.perception_fixture.tearDown();self.fixture.tearDown()

    def confirmation(self,point=(5.4,0.,2.),standoff=None,**changes):
        data=json.loads(self.fixture.confirmation('house',list(point),[0.,0.,0.],standoff).data)
        data.update(position_confirmed=True,inliers=4,observations=4,instance_id='house-a')
        data.update(changes)
        return String(data=json.dumps(data))

    def observe(self,point,count=4):
        self.perception.publish_standoff=Mock(return_value='no_reachable_standoff')
        for _ in range(count):self.perception_fixture.observe(point)
        self.fixture.now=self.perception_fixture.now
        self.bridge.odom_received_at=time.monotonic()
        return self.perception.confirmation_pub.publish.call_args

    def test_front_boundary_is_two_metres_from_body_without_standoff(self):
        self.bridge.on_object_confirmation(self.confirmation())
        self.assertTrue(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['arrival_basis'],'vehicle_body_clearance')
        self.assertAlmostEqual(self.bridge.last_status['object_distance_m'],2.)
        self.assertAlmostEqual(self.bridge.last_status['reference_point_distance_m'],5.4)
        self.assertEqual(self.bridge.last_status['object_arrival_tolerance_m'],2.)

    def test_reaching_a_different_standoff_is_unnecessary(self):
        self.bridge.on_object_confirmation(self.confirmation(standoff=[0.,5.]))
        self.assertTrue(self.bridge.object_found)

    def test_outside_body_boundary_cannot_succeed_at_standoff(self):
        self.bridge.on_object_confirmation(self.confirmation(point=(5.401,0.,2.),standoff=[0.,0.]))
        self.assertFalse(self.bridge.object_found)

    def test_map_clamping_cannot_expand_body_arrival_distance(self):
        self.bridge.on_object_confirmation(self.confirmation(point=(5.6,0.,2.),standoff=[0.,0.],
            standoff_clamped=True,standoff_object_distance_m=6.))
        self.assertFalse(self.bridge.object_found)

    def test_rotated_body_uses_world_target_and_vehicle_heading(self):
        self.bridge.odom.pose.pose.orientation.z=math.sin(math.pi/4)
        self.bridge.odom.pose.pose.orientation.w=math.cos(math.pi/4)
        self.bridge.on_object_confirmation(self.confirmation(point=(0.,5.4,2.)))
        self.assertTrue(self.bridge.object_found)
        self.assertAlmostEqual(self.bridge.last_status['object_distance_m'],2.)

    def test_side_and_rear_use_their_own_body_extents(self):
        distance=getattr(criteria,'vehicle_body_distance',None)
        self.assertIsNotNone(distance)
        self.assertAlmostEqual(distance((0.,0.),0.,(0.,3.05),'ackermann'),2.)
        self.assertAlmostEqual(distance((0.,0.),0.,(-3.05,0.),'ackermann'),2.)
        self.assertAlmostEqual(distance((0.,0.),0.,(4.4,2.05),'ackermann'),math.sqrt(2))
        self.assertAlmostEqual(distance((0.,0.),0.,(3.,0.),'ideal_point'),3.)

    def test_geometric_confirmation_survives_missing_approach_pose(self):
        call=self.observe((5.4,0.,2.))
        self.assertIsNotNone(call)
        data=json.loads(call.args[0].data)
        self.assertTrue(data['position_confirmed'])
        self.assertTrue(data['search_confirmed'])
        self.assertFalse(data['approach_goal_ready'])
        self.assertIsNone(data['standoff'])
        self.perception.publish_standoff.assert_not_called()
        self.bridge.on_object_confirmation(call.args[0])
        self.assertTrue(self.bridge.object_found)

    def test_far_confirmed_target_is_not_hidden_by_missing_approach_pose(self):
        call=self.observe((20.,0.,2.))
        data=json.loads(call.args[0].data)
        self.assertTrue(data['position_confirmed'])
        self.assertTrue(data['search_confirmed'])
        self.assertFalse(data['approach_goal_ready'])
        self.bridge.on_object_confirmation(call.args[0])
        self.assertFalse(self.bridge.object_found)

    def test_only_three_localizations_cannot_finish(self):
        self.assertIsNone(self.observe((5.,0.,2.),count=3))
        self.assertFalse(self.bridge.object_found)

    def test_duplicate_localization_does_not_supply_consensus(self):
        self.perception_fixture.observe((5.,0.,2.))
        for _ in range(4):
            self.perception_fixture.observe((5.,0.,2.),stamp=self.perception_fixture.now)
        self.perception.confirmation_pub.publish.assert_not_called()

    def test_stale_confirmation_and_query_mismatch_cannot_finish(self):
        self.bridge.on_object_confirmation(self.confirmation(observation_stamp_ns=self.fixture.now-6_000_000_000))
        self.bridge.on_object_confirmation(self.confirmation(query='gas station'))
        self.assertFalse(self.bridge.object_found)

    def test_stale_vehicle_pose_cannot_finish(self):
        self.bridge.odom_received_at=time.monotonic()-2.
        self.bridge.on_object_confirmation(self.confirmation())
        self.assertFalse(self.bridge.object_found)

    def test_carried_geometry_refresh_cannot_become_arrival(self):
        self.bridge.active_object_confirmation=dict(query='house',object_position=[5.,0.],
            observation_stamp_ns=self.fixture.now,instance_id='house-a')
        self.bridge.on_object_confirmation(self.confirmation(point=(5.,0.,2.),approach_refresh=True,
            approach_update_stamp_ns=self.fixture.now,base_observation_stamp_ns=self.fixture.now))
        self.assertFalse(self.bridge.object_found)

    def test_ambiguous_depth_cannot_finish(self):
        self.bridge.on_object_confirmation(self.confirmation(quality=dict(confidence=.9,association_validated=False)))
        self.assertFalse(self.bridge.object_found)

    def test_evidence_matcher_recomputes_body_distance_and_requires_live_mask(self):
        message=self.confirmation();self.bridge.on_object_confirmation(message)
        self.assertTrue(self.bridge.object_found)
        status=dict(self.bridge.last_status);confirmation=json.loads(message.data)
        self.assertTrue(criteria.arrival_status_matches(status,'house',confirmation))
        c=dict(confirmation,received_at=10.)
        event=dict(status=status,received_at=10.)
        mask=dict(query='house',observation_stamp_ns=confirmation['observation_stamp_ns'],
            at=10.,count=3,pixels=30)
        match=lambda masks:criteria.matched_arrival_evidence([event],[c],masks,'house',
            self.bridge.query_started_ns,self.fixture.now,10.,2)
        self.assertIsNone(match([]))
        self.assertIsNotNone(match([mask]))
        for change in (dict(object_distance_m=1.),dict(object_arrival_tolerance_m=2.1),
                       dict(vehicle_heading=math.pi),dict(vehicle_position=[-1.,0.])):
            self.assertFalse(criteria.arrival_status_matches(dict(status,**change),'house',confirmation))

    def test_memory_without_live_confirmation_cannot_finish(self):
        self.bridge.memory_active=dict(id='old-view',object_position=[5.,0.])
        self.assertIsNone(criteria.matched_arrival_evidence([],[],[],'house',
            self.bridge.query_started_ns,self.fixture.now,10.,2))
        self.assertFalse(self.bridge.object_found)

    def test_arrival_cancels_active_follow_action_and_is_terminal(self):
        handle=Mock()
        self.bridge.following=True;self.bridge.follow_handle=handle
        self.bridge.on_object_confirmation(self.confirmation())
        self.assertTrue(self.bridge.object_found)
        handle.cancel_goal_async.assert_called_once()
        epoch=self.bridge.epoch
        self.bridge.on_object_confirmation(self.confirmation(point=(30.,0.,2.)))
        self.assertEqual(self.bridge.epoch,epoch)
        self.assertTrue(self.bridge.object_found)

    def test_continuous_strategy_uses_the_same_arrival_terminal(self):
        from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
        with patch.object(execution_fixture,'GraphPathBridge',ContinuousGraphPathBridge):
            fixture=execution_fixture.ExecutionMechanismTests();fixture.setUp()
        try:
            node=fixture.node;node.query='house'
            node._desired_intent=('old-frontier',)
            node.on_object_confirmation(self.confirmation())
            self.assertTrue(node.object_found)
            self.assertIsNone(node._desired_intent)
            self.assertEqual(node.last_status['arrival_basis'],'vehicle_body_clearance')
        finally:
            fixture.tearDown()


if __name__=='__main__':unittest.main()
