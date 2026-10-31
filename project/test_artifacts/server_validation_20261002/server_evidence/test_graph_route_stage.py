import math
import unittest
from types import SimpleNamespace as NS

from wildos_nav2_adapter.graph_route import (
    cell_cost, footprint_blockers, freshness_issue, route_candidates,
    validate_vehicle_path,
)


def grid(width=120, height=80, resolution=0.25):
    origin = NS(position=NS(x=-5.0, y=-5.0), orientation=NS(x=0., y=0., z=0., w=1.))
    return NS(info=NS(width=width, height=height, resolution=resolution,
                      origin=origin), data=[0] * (width * height))


def set_cell(map_msg, x, y, cost):
    i = math.floor((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
    j = math.floor((y - map_msg.info.origin.position.y) / map_msg.info.resolution)
    map_msg.data[j * map_msg.info.width + i] = cost


class GraphRouteTests(unittest.TestCase):
    def test_planning_inputs_must_be_recent(self):
        self.assertIsNone(freshness_issue(10., 9.5, 9., 1., 3.))
        self.assertEqual(freshness_issue(10., None, 9., 1., 3.),
                         ("stale_odom", None))
        issue = freshness_issue(10., 8.8, 9., 1., 3.)
        self.assertEqual(issue[0], "stale_odom")
        self.assertAlmostEqual(issue[1], 1.2)
        self.assertEqual(freshness_issue(10., 9.5, 6., 1., 3.),
                         ("stale_costmap", 4.))

    def test_candidates_follow_graph_and_skip_obstructed_goal(self):
        occupancy = grid()
        set_cell(occupancy, 10.0, 0.0, 100)
        candidates = route_candidates((0., 0.), [(0., 0.), (18., 0.)], occupancy,
                                      maximum=16., spacing=2.)
        self.assertEqual([int(c[3]) for c in candidates], [16, 14, 12, 8, 6, 4])
        self.assertTrue(all(abs(c[2]) < 1e-6 for c in candidates))

    def test_completed_route_prefix_is_not_planned_backwards(self):
        candidates = route_candidates((12., 0.), [(0., 0.), (10., 0.), (24., 0.)], grid())
        self.assertTrue(candidates)
        self.assertTrue(all(c[0] > 12. for c in candidates))
        self.assertEqual(candidates[0][:2], (24., 0.))

    def test_last_short_approach_has_a_candidate(self):
        candidates = route_candidates((0., 0.), [(0., 0.), (1.2, 0.)], grid())
        self.assertEqual(candidates[0][:2], (1.2, 0.))

    def test_endpoint_beyond_horizon_is_not_added_at_a_segment_boundary(self):
        candidates = route_candidates((0., 0.), [(0., 0.), (8., 0.), (20., 0.)],
                                      grid(), maximum=8.)
        self.assertTrue(all(c[0] <= 8. for c in candidates))

    def test_result_start_and_observation_heading_are_checked(self):
        args = ([(0., 0.), (6., 0.)], [0., 0.], [(0., 0.), (6., 0.)], grid(), (6., 0.))
        self.assertEqual(validate_vehicle_path(*args, start=(2., 0.)), "wrong_start")
        self.assertEqual(validate_vehicle_path(*args, target_heading=1.5), "wrong_heading")

    def test_path_rejects_obstacle_between_nav2_poses(self):
        occupancy = grid()
        set_cell(occupancy, 5., 0., 100)
        reason = validate_vehicle_path([(0., 0.), (10., 0.)], [0., 0.],
                                       [(0., 0.), (10., 0.)], occupancy, (10., 0.))
        self.assertEqual(reason, "collision_cell")

    def test_path_checks_vehicle_footprint_between_sparse_poses(self):
        occupancy = grid()
        set_cell(occupancy, 3., 0.75, 100)
        reason = validate_vehicle_path([(0., 0.), (6., 0.)], [0., 0.],
                                       [(0., 0.), (6., 0.)], occupancy, (6., 0.))
        self.assertEqual(reason, "footprint_collision")

    def test_vehicle_body_can_be_blocked_while_center_is_free(self):
        occupancy = grid()
        set_cell(occupancy, 2.1, 0.8, 100)
        self.assertEqual(cell_cost(occupancy, 0., 0.), 0)
        self.assertTrue(footprint_blockers(occupancy, (0., 0.), 0.))

    def test_inscribed_inflation_is_not_applied_twice_to_vehicle_body(self):
        occupancy = grid()
        set_cell(occupancy, 2.1, 0.8, 99)
        self.assertEqual(footprint_blockers(occupancy, (0., 0.), 0.), [])
        self.assertIsNone(validate_vehicle_path(
            [(0., 0.), (6., 0.)], [0., 0.], [(0., 0.), (6., 0.)],
            occupancy, (6., 0.)))

    def test_path_center_still_rejects_inscribed_inflation(self):
        occupancy = grid()
        set_cell(occupancy, 3., 0., 99)
        self.assertEqual(validate_vehicle_path(
            [(0., 0.), (6., 0.)], [0., 0.], [(0., 0.), (6., 0.)],
            occupancy, (6., 0.)), "collision_cell")

    def test_path_rejects_unknown_and_off_route(self):
        occupancy = grid()
        for x in [n / 4 for n in range(4, 33)]:
            set_cell(occupancy, x, 0., -1)
        self.assertEqual(validate_vehicle_path([(0., 0.), (8., 0.)], [0., 0.],
                         [(0., 0.), (8., 0.)], occupancy, (8., 0.)), "unknown_costmap")
        self.assertEqual(validate_vehicle_path([(0., 10.), (8., 10.)], [0., 0.],
                         [(0., 0.), (8., 0.)], occupancy, (8., 10.)), "outside_graph_corridor")

    def test_path_rejects_sharp_turn_but_accepts_straight_path(self):
        occupancy = grid()
        points = [(0., 0.), (1., 0.), (2., 0.), (3., 0.), (4., 0.)]
        self.assertIsNone(validate_vehicle_path(points, [0.] * 5,
                          [(0., 0.), (4., 0.)], occupancy, (4., 0.)))
        self.assertEqual(validate_vehicle_path(points, [0., 0., 1., 1., 1.],
                         [(0., 0.), (4., 0.)], occupancy, (4., 0.)), "excess_curvature")

    def test_curvature_limit_matches_vehicle_turning_radius(self):
        occupancy = grid()
        points = [(0., 0.), (6., 0.)]
        route = [(0., 0.), (6., 0.)]
        headings = [0., 1.0]
        self.assertIsNone(validate_vehicle_path(
            points, headings, route, occupancy, (6., 0.),
            minimum_turning_radius=5.0))
        self.assertEqual(validate_vehicle_path(
            points, headings, route, occupancy, (6., 0.),
            minimum_turning_radius=8.0), "excess_curvature")

    def test_sampled_circle_uses_arc_length_for_curvature(self):
        occupancy = grid()
        headings = [math.radians(5 * i) for i in range(7)]
        for radius, expected in [(8.0, None), (7.99, "excess_curvature")]:
            points = [(radius * math.sin(h), radius * (1 - math.cos(h)))
                      for h in headings]
            self.assertEqual(validate_vehicle_path(
                points, headings, points, occupancy, points[-1],
                minimum_turning_radius=8.0), expected)

    def test_rotated_costmap_origin(self):
        occupancy = grid(width=4, height=4, resolution=1.0)
        occupancy.info.origin.position.x = 0.
        occupancy.info.origin.position.y = 0.
        occupancy.info.origin.orientation.z = math.sin(math.pi / 4)
        occupancy.info.origin.orientation.w = math.cos(math.pi / 4)
        occupancy.data[1] = 100
        self.assertEqual(cell_cost(occupancy, -.5, 1.5), 100)

    def test_quantized_reverse_arc_keeps_the_actual_turning_radius_limit(self):
        occupancy = grid()
        bin_angle = math.radians(5.)
        continuous = [bin_angle*i/2 for i in range(17)]
        headings = [bin_angle*math.ceil(i/2) for i in range(17)]
        for radius, expected in ((8., None), (7.99, 'excess_curvature')):
            points = [(8-radius*math.sin(h), 8-radius*(1-math.cos(h))) for h in continuous]
            self.assertEqual(validate_vehicle_path(points, headings, points, occupancy, points[-1],
                             heading_quantization_bins=72), expected)

    def test_reverse_cusp_is_not_a_body_rotation(self):
        points = [(5.,0.), (4.,0.), (3.,0.), (2.,0.), (3.,0.), (4.,0.), (5.,0.), (6.,0.)]
        self.assertIsNone(validate_vehicle_path(points, [0.]*len(points), points, grid(), points[-1],
                          heading_quantization_bins=72))


if __name__ == "__main__":
    unittest.main()
