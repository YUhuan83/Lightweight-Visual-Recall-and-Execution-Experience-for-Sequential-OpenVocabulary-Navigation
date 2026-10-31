import math
import unittest

from wildos_nav2_adapter.motion_path import (
    MANEUVER_COMPLETION_POSITION_TOLERANCE_M, maneuver_reached, motion_sections,
    initial_maneuver_already_reached, maneuver_join, diagnose_maneuver_join)


class MotionSectionTests(unittest.TestCase):
    def test_settled_cusp_join_checks_direction_and_steering(self):
        for direction in (1., -1.):
            points = [(0., 0.), (direction*6., 0.)]
            join = maneuver_join(points, [0., 0.], (.2, .05), .1, 8.)
            self.assertIsNotNone(join)
            self.assertAlmostEqual(join[0][0], .2)
            self.assertAlmostEqual(join[-1][1], 0.)
        self.assertIsNone(maneuver_join([(0.,0.),(6.,0.)], [0.,0.], (0.,.29), .34, 8.))
        self.assertIsNone(maneuver_join([(0.,0.),(6.,0.)], [0.,0.], (.61,0.), 0., 8.))

    def test_accepted_cusp_disk_can_join_next_feasible_branch(self):
        for gap in (.56, .58, .59):
            self.assertIsNotNone(maneuver_join(
                [(0., 0.), (6., 0.)], [0., 0.], (-gap, 0.), .02, 8.))

    def test_cusp_join_reports_which_geometry_check_rejected_the_arc(self):
        join, reason = diagnose_maneuver_join(
            [(0., 0.), (6., 0.)], [0., 0.], (0., .59), 0., 8.)
        self.assertIsNone(join)
        self.assertEqual(reason, "join_curvature_exceeds_turning_limit")
        join, reason = diagnose_maneuver_join(
            [(0., 0.), (6., 0.)], [0., 0.], (.61, 0.), 0., 8.)
        self.assertIsNone(join)
        self.assertEqual(reason, "cusp_position_outside_tolerance")

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
        self.assertFalse(maneuver_reached((-1.65, 0.), 0., (-2., 0.), 0.))

    def test_cusp_completion_requires_heading_and_position(self):
        self.assertFalse(maneuver_reached((-2., 0.), 1., (-2., 0.), 0.))
        self.assertTrue(maneuver_reached((-1.9, 0.), .1, (-2., 0.), 0.))
        self.assertTrue(maneuver_reached((-2., 0.), math.pi, (-2., 0.), -math.pi))
        self.assertFalse(maneuver_reached((math.nan, 0.), 0., (-2., 0.), 0.))

    def test_recorded_cusp_offsets_can_complete_without_accepting_a_large_gap(self):
        for gap in (.114,.164,.23):
            self.assertTrue(maneuver_reached((gap,0.),math.radians(13.),(0.,0.),0.))
        self.assertFalse(maneuver_reached((.31,0.),0.,(0.,0.),0.))

    def test_ackermann_endpoint_tolerance_accepts_a_valid_near_cusp_pose(self):
        self.assertEqual(MANEUVER_COMPLETION_POSITION_TOLERANCE_M, .6)
        self.assertTrue(maneuver_reached(
            (.54, 0.), .02, (0., 0.), 0.,
            position_tolerance=MANEUVER_COMPLETION_POSITION_TOLERANCE_M,
            heading_tolerance=.35))
        self.assertFalse(maneuver_reached(
            (.61, 0.), .02, (0., 0.), 0.,
            position_tolerance=MANEUVER_COMPLETION_POSITION_TOLERANCE_M,
            heading_tolerance=.35))

    def test_short_initial_cusp_inside_arrival_disk_cannot_repeat_without_motion(self):
        points = [(0., 0.), (-.356, 0.), (3., 0.)]
        headings = [0., 0., 0.]
        self.assertTrue(initial_maneuver_already_reached(points, headings, (-.126, 0.), .112))
        self.assertFalse(initial_maneuver_already_reached(points, headings, (0., 0.), 0.))
        self.assertFalse(initial_maneuver_already_reached(points, headings, (-.126, 0.), .36))
        self.assertFalse(initial_maneuver_already_reached(points[:2], headings[:2], (-.126, 0.), 0.))


if __name__ == '__main__':
    unittest.main()
