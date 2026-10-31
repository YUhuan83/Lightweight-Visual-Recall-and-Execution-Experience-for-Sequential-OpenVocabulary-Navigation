"""Recovery preserves pose/yaw identity rather than reusing another endpoint's yaw."""
import math,unittest
from unittest.mock import Mock
import test_confirmed_recovery_ros as fixture

class RecoveryHeadingTests(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ConfirmedRecoveryTests();self.f.setUp();self.n=self.f.node
        self.n.confirmed_route_heading=.9
    def tearDown(self):self.f.tearDown()
    def test_deferred_intermediate_keeps_its_own_navigation_heading(self):
        n=self.n
        n.deferred_confirmed_approach=dict(self.f.data,standoff=[14.,4.],
            standoff_clamped=True,standoff_navigation_heading=.3)
        self.f.exhaust_plans();n.monitor_observation()
        q=n.waypoint_pub.publish.call_args.args[0].pose.orientation
        self.assertAlmostEqual(math.atan2(2*q.w*q.z,1-2*q.z*q.z),.3)
        self.assertAlmostEqual(n.exploration_observation_heading(),.3)
        self.assertEqual(n.confirmed_route_goal,(14.,4.))
        self.assertFalse(n.object_found)
    def test_new_radial_view_cannot_inherit_failed_endpoints_heading(self):
        n=self.n
        self.f.exhaust_plans();n.monitor_observation()
        self.assertIsNone(n.confirmed_route_heading)
        q=n.waypoint_pub.publish.call_args.args[0].pose.orientation
        heading=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
        self.assertAlmostEqual(n.exploration_observation_heading(),heading)
        self.assertFalse(n.object_found)
    def test_rejected_intermediate_does_not_leak_its_yaw_to_fallback(self):
        n=self.n
        n.deferred_confirmed_approach=dict(self.f.data,standoff=[14.,4.],
            standoff_clamped=True,standoff_navigation_heading=.3)
        n.pose_blockers=Mock(side_effect=lambda _g,p,_h:['occupied'] if p==(14.,4.) else [])
        self.f.exhaust_plans();n.monitor_observation()
        self.assertNotEqual(n.expected,(14.,4.))
        self.assertIsNone(n.confirmed_route_heading)
        q=n.waypoint_pub.publish.call_args.args[0].pose.orientation
        self.assertAlmostEqual(n.exploration_observation_heading(),math.atan2(2*q.w*q.z,1-2*q.z*q.z))
    def test_no_new_pose_does_not_mutate_old_pose_heading(self):
        n=self.n;n.pose_blockers=Mock(return_value=['occupied'])
        self.assertFalse(n.start_live_observation())
        self.assertEqual(n.confirmed_route_heading,.9)
        n.waypoint_pub.publish.assert_not_called()
