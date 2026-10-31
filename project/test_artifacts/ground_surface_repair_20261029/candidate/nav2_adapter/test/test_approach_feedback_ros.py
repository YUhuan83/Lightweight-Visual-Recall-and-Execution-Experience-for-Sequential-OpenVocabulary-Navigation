"""Accepted waypoint failure must not be compared to a newer unaccepted proposal."""
from copy import deepcopy
import json
import unittest
from unittest.mock import Mock
from std_msgs.msg import String
import test_surface_association_ros as fixture


class ApproachFeedbackTests(unittest.TestCase):
    setUp = fixture.SurfaceAssociationTests.setUp
    tearDown = fixture.SurfaceAssociationTests.tearDown
    advance = fixture.SurfaceAssociationTests.advance
    localize = fixture.SurfaceAssociationTests.localize

    def lock(self):
        for _ in range(5):
            self.localize()
        n, b = self.goal, self.bridge
        anchor = deepcopy(n.confirmed_instance_anchor)
        self.old_goal = (12., 0.)
        n.last_goal = (16., 0.)  # New safe intermediate proposal, not accepted yet.
        n.active_approach = (Mock(), deepcopy(anchor))
        b.active_object_confirmation = deepcopy(anchor)
        b.expected = b.confirmed_route_goal = self.old_goal
        b.confirmed_route_object = tuple(anchor['object_position'][:2])
        b.failed_confirmed_goals = []
        b.status_pub = Mock()
        n.publish_standoff = Mock(return_value=None)
        n.publish_approach_refresh = Mock()
        return anchor

    def failure(self):
        self.assertTrue(self.bridge.fail_confirmed_approach('no_vehicle_path'))
        return self.bridge.status_pub.publish.call_args.args[0]

    def test_real_failure_publisher_carries_the_selected_instance(self):
        anchor = self.lock()
        data = json.loads(self.failure().data)
        self.assertEqual(data.get('instance_id'), anchor['instance_id'])
        self.assertEqual(data['goal'], list(self.old_goal))

    def test_new_proposal_does_not_hide_failure_of_accepted_waypoint(self):
        self.lock()
        self.goal.on_navigation_plan_status(self.failure())
        self.goal.publish_standoff.assert_called_once()
        self.goal.publish_approach_refresh.assert_called_once()
        self.assertEqual(self.goal.rejected_observation_poses[-1][0], self.old_goal)
        self.assertNotEqual(self.goal.last_goal, self.old_goal)

    def test_other_instance_cannot_blacklist_current_proposal(self):
        self.lock()
        data = json.loads(self.failure().data)
        data.update(instance_id='old-other-instance', goal=list(self.goal.last_goal))
        self.goal.on_navigation_plan_status(String(data=json.dumps(data)))
        self.goal.publish_standoff.assert_not_called()
        self.assertFalse(self.goal.rejected_observation_poses)

    def test_duplicate_and_expired_failure_do_not_recompute(self):
        self.lock()
        message = self.failure()
        self.goal.on_navigation_plan_status(message)
        self.assertEqual(self.goal.publish_standoff.call_count, 1)
        self.goal.on_navigation_plan_status(message)
        self.now += 3_000_000_000
        self.goal.on_navigation_plan_status(message)
        self.assertEqual(self.goal.publish_standoff.call_count, 1)

    def test_legacy_unidentified_failure_requires_matching_proposal(self):
        self.lock()
        data = json.loads(self.failure().data)
        data.pop('instance_id', None)
        self.goal.on_navigation_plan_status(String(data=json.dumps(data)))
        self.goal.publish_standoff.assert_not_called()
        data['goal'] = list(self.goal.last_goal)
        self.goal.on_navigation_plan_status(String(data=json.dumps(data)))
        self.goal.publish_standoff.assert_called_once()


if __name__ == '__main__':
    unittest.main()
