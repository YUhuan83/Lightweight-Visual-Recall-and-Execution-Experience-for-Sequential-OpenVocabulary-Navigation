"""Pure geometry checks for the first motion section (no ROS required).

Motivated by trial_20261021_183339: the exploration goal was ~25 m south, but
the first dispatched sections drove north for about 75 s. A soft cost can
influence ranking among admitted Ackermann alternatives without banning reverse.
"""
import unittest

from wildos_nav2_adapter.motion_path import (first_section_away, first_section_retreat_cost,
                                             motion_sections)


def line(start, end, step=0.25):
    length = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
    count = max(1, int(length / step))
    return [(start[0] + (end[0] - start[0]) * i / count,
             start[1] + (end[1] - start[1]) * i / count) for i in range(count + 1)]


class FirstSectionDirectionTests(unittest.TestCase):
    def test_north_opening_toward_a_southern_goal_is_reported(self):
        # 183339: goal (727.5,-26.5); first section from (734.5,-12) north to
        # (735.2,-3.2), then the suffix turns south.
        points = line((734.5, -12.0), (735.2, -3.2))
        points += line((735.2, -3.2), (732.0, -18.0))[1:]
        self.assertEqual(len(motion_sections(points)), 2)
        self.assertTrue(first_section_away(points, (727.5, -26.5)))

    def test_section_toward_the_goal_is_not_reported(self):
        points = line((734.5, -12.0), (731.0, -18.0))
        points += line((731.0, -18.0), (729.0, -24.0))[1:]
        self.assertFalse(first_section_away(points, (727.5, -26.5)))

    def test_short_backup_inside_the_threshold_is_not_reported(self):
        # A legitimate three-point turn may back up ~1 m before turning.
        points = line((734.5, -12.0), (734.5, -11.0))
        points += line((734.5, -11.0), (730.0, -20.0))[1:]
        self.assertFalse(first_section_away(points, (727.5, -26.5)))
        self.assertTrue(first_section_away(points, (727.5, -26.5), 0.5))

    def test_disabled_or_degenerate_inputs_are_false(self):
        points = line((734.5, -12.0), (735.2, -3.2))
        self.assertFalse(first_section_away(points, (727.5, -26.5), 0))
        self.assertFalse(first_section_away([(1.0, 1.0)], (0.0, 0.0)))
        self.assertFalse(first_section_away(points, (0.0,)))

    def test_first_section_retreat_cost_is_soft_and_only_measures_first_section(self):
        points = [(0., 0.), (-1., 0.), (-2., 0.), (-1., 0.), (0., 0.), (2., 0.)]
        self.assertEqual(first_section_retreat_cost(points, (5., 0.)), 2.)
        self.assertEqual(first_section_retreat_cost([(0., 0.), (1., 0.)], (5., 0.)), 0.)
        self.assertEqual(first_section_retreat_cost([(0., 0.)], (5., 0.)), 0.)


if __name__ == "__main__":
    unittest.main()
