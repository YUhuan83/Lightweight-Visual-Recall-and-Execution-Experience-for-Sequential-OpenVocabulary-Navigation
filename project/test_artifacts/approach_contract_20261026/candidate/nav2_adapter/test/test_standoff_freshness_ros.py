"""Stale vehicle map is a rejected pose, not an exception in the approach producer."""
import unittest
import test_confirmed_vehicle_ros as fixture


class StandoffFreshnessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.ConfirmedVehicleTests.setUpClass()

    @classmethod
    def tearDownClass(cls):
        fixture.ConfirmedVehicleTests.tearDownClass()

    def setUp(self):
        self.f = fixture.ConfirmedVehicleTests()
        self.f.setUp()

    def tearDown(self):
        self.f.tearDown()

    def test_stale_vehicle_grid_rejects_explicit_heading_without_crashing(self):
        target, _ = self.f.recorded()
        n = self.f.n
        n.vehicle_grid.header.stamp.sec -= 4
        try:
            reason = n.publish_standoff(target)
        except TypeError as error:
            self.fail('Pose-filter heading contract violated: '+str(error))
        self.assertEqual(reason, 'no_reachable_standoff')
        self.assertIsNone(n.last_goal)
        n.publisher.publish.assert_not_called()

    def test_fresh_map_recovery_preserves_the_recorded_vehicle_pose(self):
        target, _ = self.f.recorded()
        n = self.f.n
        stamp = n.vehicle_grid.header.stamp.sec
        n.vehicle_grid.header.stamp.sec -= 4
        try:
            n.publish_standoff(target)
        except TypeError as error:
            self.fail('Pose-filter heading contract violated: '+str(error))
        n.vehicle_grid.header.stamp.sec = stamp
        self.assertIsNone(n.publish_standoff(target))
        self.assertIsNotNone(n.last_goal)
        self.assertIsNotNone(n.last_standoff_navigation_heading)
        self.assertTrue(n.last_standoff_clamped)
