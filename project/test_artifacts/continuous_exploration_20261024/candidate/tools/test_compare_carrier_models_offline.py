"""Validate comparison arithmetic and scope; these tests do not fix navigation."""
import math
import unittest
from compare_carrier_models_offline import compare, point_step, point_witness


class CarrierComparisonTests(unittest.TestCase):
    def test_point_rotation_does_not_translate_navigation_origin(self):
        position, heading = point_step((1., 2.), .4, 0., .8, .02)
        self.assertEqual(position, (1., 2.))
        self.assertAlmostEqual(heading, .416)

    def test_point_straight_and_curved_equations(self):
        position, heading = point_step((0., 0.), math.pi/2, 1.2, 0., .1)
        self.assertAlmostEqual(position[0], 0.)
        self.assertAlmostEqual(position[1], .12)
        position, heading = point_step((0., 0.), 0., 1., .5, .1)
        self.assertAlmostEqual(position[0], 2*math.sin(.05))
        self.assertAlmostEqual(position[1], 2*(1-math.cos(.05)))

    def test_witness_covers_both_directions_and_zero_distance(self):
        for direction in (-1, 1):
            for endpoint in ((3., -2.), (0., 0.)):
                result = point_witness((0., 0.), .9, endpoint, -.4, direction=direction)
                self.assertLess(result['endpoint_distance_m'], 1e-10)
                self.assertLess(abs(result['heading_error_rad']), 1e-10)

    def test_same_recorded_poses_distinguish_point_from_bounded_feedback(self):
        result = compare()
        self.assertEqual(len(result['samples']), 3)
        for sample in result['samples']:
            self.assertLess(sample['ideal_point_witness']['endpoint_distance_m'], 1e-9)
            self.assertLess(abs(sample['ideal_point_witness']['heading_error_rad']), 1e-9)
            self.assertFalse(sample['bounded_bicycle_replay']['reached'])
            self.assertTrue(sample['bounded_bicycle_replay']['direction_guard'])
        self.assertIn('not full navigation acceptance', result['scope'])


if __name__ == '__main__':
    unittest.main()
