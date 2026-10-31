import math
import unittest

from wildos_nav2_adapter.motion_path import maneuver_reached, motion_sections


class MotionSectionTests(unittest.TestCase):
    def test_reverse_forward_sections_share_cusp_but_not_later_branch(self):
        points = [(0., 0.), (-1., 0.), (-2., 0.), (-1., 0.), (0., 0.), (1., 0.)]
        self.assertEqual(motion_sections(points), [(0, 2), (2, 5)])
        # An actual pose closer to a later branch still cannot prune the cusp
        # when that branch is absent from the first FollowPath goal.
        first = points[:3]
        position = (-0.6, 0.)
        nearest = min(range(len(first)), key=lambda i: math.dist(first[i], position))
        self.assertLessEqual(nearest, 2)
        self.assertEqual(first[-1], (-2., 0.))

    def test_repeated_cusp_pose_does_not_hide_direction_change(self):
        points = [(0., 0.), (-1., 0.), (-1., 0.), (0., 0.)]
        self.assertEqual(motion_sections(points), [(0, 2), (2, 3)])

    def test_multiple_cusps_preserve_all_poses_in_order(self):
        points = [(0., 0.), (-2., 0.), (3., 0.), (1., 0.)]
        self.assertEqual(motion_sections(points), [(0, 1), (1, 2), (2, 3)])

    def test_smooth_curve_is_not_split(self):
        points = [(8 * math.cos(t), 8 * math.sin(t)) for t in (0., .1, .2, .3)]
        self.assertEqual(motion_sections(points), [(0, 3)])

    def test_short_paths_have_no_executable_section(self):
        self.assertEqual(motion_sections([]), [])
        self.assertEqual(motion_sections([(0., 0.)]), [])

    def test_success_before_cusp_does_not_allow_next_direction(self):
        self.assertFalse(maneuver_reached((-1., 0.), 0., (-2., 0.), 0.))
        self.assertFalse(maneuver_reached((-1.75, 0.), 0., (-2., 0.), 0.))

    def test_cusp_completion_requires_heading_and_position(self):
        self.assertFalse(maneuver_reached((-2., 0.), 1., (-2., 0.), 0.))
        self.assertTrue(maneuver_reached((-1.9, 0.), .1, (-2., 0.), 0.))
        self.assertTrue(maneuver_reached((-2., 0.), math.pi, (-2., 0.), -math.pi))
        self.assertFalse(maneuver_reached((math.nan, 0.), 0., (-2., 0.), 0.))


if __name__ == '__main__':
    unittest.main()
