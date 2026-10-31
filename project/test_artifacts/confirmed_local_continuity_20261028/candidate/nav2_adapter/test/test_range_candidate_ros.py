"""Measured current range guides observation; consensus still owns confirmation."""
import json
import math
import unittest
from unittest.mock import Mock
from std_msgs.msg import String
from test_visible_approach_ros import VisibleApproachTests as Harness


class RangeCandidateTests(unittest.TestCase):
    setUpClass = classmethod(Harness.setUpClass.__func__)
    tearDownClass = classmethod(Harness.tearDownClass.__func__)
    setUp = Harness.setUp
    tearDown = Harness.tearDown
    stamp = Harness.stamp
    query = Harness.query
    mask = Harness.mask
    detect = Harness.detect

    def prepare(self):
        self.perception.observation_minimum_turning_radius = 12.
        self.perception.vehicle_observation_pose = Mock(return_value=((5., 0.), 0.))
        self.detect()
        self.perception.visible_step_published_ns = 0
        self.perception.visible_approach_pub.publish.reset_mock()
        self.perception.vehicle_observation_pose.reset_mock()

    def localization(self, **changes):
        value = dict(query=self.perception.active_query, frame='spot1/odom',
            observation_stamp_ns=self.now, localization_valid=True, source='lidar',
            position=[10., 0., 3.], observer_position=[0., 0., 0.],
            observer_orientation=[0., 0., 0., 1.],
            quality=dict(association_validated=True, confidence=.65))
        value.update(changes)
        return String(data=json.dumps(value))

    def test_supported_current_range_guides_observation_before_consensus(self):
        self.prepare()
        self.perception.on_localization(self.localization())
        self.perception.vehicle_observation_pose.assert_called_once()
        args = self.perception.vehicle_observation_pose.call_args.args
        self.assertEqual(args[1], (10., 0.))
        self.assertGreater(args[2], 4.)
        payload = json.loads(self.perception.visible_approach_pub.publish.call_args.args[0].data)
        self.assertEqual(payload['evidence'], 'range_candidate')
        self.assertTrue(payload['target_range_known'])
        self.assertFalse(payload['position_confirmed'])
        self.assertFalse(payload['search_confirmed'])
        self.perception.confirmation_pub.publish.assert_not_called()

    def test_old_mask_range_does_not_bind_a_new_mask(self):
        self.prepare()
        self.perception.on_localization(self.localization())
        self.now += 100_000_000
        self.perception.on_visible_target(self.mask())
        self.perception.visible_step_published_ns = 0
        self.perception.vehicle_observation_pose.reset_mock()
        self.perception.publish_visible_step()
        self.perception.vehicle_observation_pose.assert_called_once_with(0.)
        self.assertEqual(json.loads(self.perception.visible_approach_pub.publish.call_args.args[0].data)['evidence'], 'bearing_only')

    def test_invalid_association_and_other_query_cannot_supply_range(self):
        for changes in (dict(quality=dict(association_validated=False, confidence=.9)),
                        dict(query='gas station')):
            self.query('gas station' if self.perception.active_query!='gas station' else 'green residential house')
            self.prepare()
            changes = dict(changes)
            if 'query' in changes: changes['query']='not the active query'
            self.perception.on_localization(self.localization(**changes))
            self.perception.visible_approach_pub.publish.assert_not_called()

    def test_candidate_expiry_preserves_mask_freshness_gate(self):
        self.prepare()
        self.perception.on_localization(self.localization())
        self.now += 2_000_000_001
        self.perception.visible_approach_pub.publish.reset_mock()
        self.perception.visible_step_published_ns = 0
        self.perception.publish_visible_step()
        self.perception.visible_approach_pub.publish.assert_not_called()

    def test_duplicate_range_does_not_publish_again(self):
        self.prepare()
        message=self.localization()
        self.perception.on_localization(message)
        calls=self.perception.visible_approach_pub.publish.call_count
        self.perception.on_localization(message)
        self.assertEqual(self.perception.visible_approach_pub.publish.call_count,calls)
        self.assertEqual(len(self.perception.observations),1)

    def test_bridge_accepts_candidate_as_observation_never_arrival(self):
        self.prepare()
        data=dict(query=self.bridge.query,frame='spot1/odom',observation_stamp_ns=self.now,
            point=[5.,0.],bearing=0.,navigation_heading=0.,evidence='range_candidate',
            target_range_known=True,object_position=[10.,0.,3.],localization_source='lidar',
            position_confirmed=False,search_confirmed=False)
        self.bridge.on_visible_approach(String(data=json.dumps(data)))
        self.assertTrue(self.bridge.visible_approach_active)
        self.assertEqual(self.bridge.expected,(5.,0.))
        self.assertFalse(self.bridge.object_found)
        self.assertIsNone(self.bridge.active_object_confirmation)
        self.assertTrue(self.bridge.last_status['target_range_known'])

    def test_bridge_rejects_malformed_and_confirmed_candidate(self):
        self.prepare()
        data=dict(query=self.bridge.query,frame='spot1/odom',observation_stamp_ns=self.now,
            point=[5.,0.],bearing=0.,evidence='range_candidate',target_range_known=True,
            object_position=[10.,0.,3.],localization_source='lidar',
            position_confirmed=False,search_confirmed=False)
        for changes in (dict(object_position=[10.,0.]),dict(object_position=[float('nan'),0.,3.]),
                        dict(search_confirmed=True),dict(localization_source='unknown'),
                        dict(observation_stamp_ns=self.now-2_000_000_001)):
            self.bridge.on_visible_approach(String(data=json.dumps({**data,**changes})))
            self.assertFalse(self.bridge.visible_approach_active)


del Harness
