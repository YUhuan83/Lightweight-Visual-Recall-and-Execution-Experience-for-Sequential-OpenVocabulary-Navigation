import math
import csv
import json
from pathlib import Path
import unittest
from types import SimpleNamespace as NS

from wildos_nav2_adapter.graph_route import (
    cell_cost, footprint_blockers, point_blockers, freshness_issue, route_candidates,
    validate_vehicle_path, validate_ideal_path, validate_ideal_local_prefix,
    forward_arc_arrival_heading,
)
from wildos_nav2_adapter.input_quality import measurement_age


def grid(width=120, height=80, resolution=0.25):
    origin = NS(position=NS(x=-5.0, y=-5.0), orientation=NS(x=0., y=0., z=0., w=1.))
    return NS(info=NS(width=width, height=height, resolution=resolution,
                      origin=origin), data=[0] * (width * height))


def set_cell(map_msg, x, y, cost):
    i = math.floor((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
    j = math.floor((y - map_msg.info.origin.position.y) / map_msg.info.resolution)
    map_msg.data[j * map_msg.info.width + i] = cost


class GraphRouteTests(unittest.TestCase):
    def test_forward_arc_arrival_heading_matches_circle_and_rotation(self):
        for rotation in (0., .7, -2.8):
            for sign in (-1., 1.):
                radius, turn = 13., sign*.4
                start = (3., -5.)
                local = (radius*math.sin(abs(turn)), sign*radius*(1-math.cos(turn)))
                target = (start[0]+math.cos(rotation)*local[0]-math.sin(rotation)*local[1],
                          start[1]+math.sin(rotation)*local[0]+math.cos(rotation)*local[1])
                heading = forward_arc_arrival_heading(start, rotation, target, 8.)
                self.assertIsNotNone(heading)
                self.assertAlmostEqual(math.remainder(heading-rotation-turn, 2*math.pi), 0.)

    def test_forward_arc_arrival_heading_preserves_radius_and_position_envelope(self):
        for start, heading, target, radius in (
                ((0.,0.),0.,(2.,1.),8.), ((0.,0.),0.,(-3.,0.),8.),
                ((0.,0.),0.,(0.,3.),8.), ((0.,0.),0.,(.5,0.),8.),
                ((0.,0.),float('nan'),(6.,1.),8.), ((0.,0.),0.,(6.,1.),0.)):
            with self.subTest(start=start, target=target, heading=heading):
                self.assertIsNone(forward_arc_arrival_heading(start, heading, target, radius))
        self.assertEqual(forward_arc_arrival_heading((0.,0.),0.,(6.,0.),8.),0.)

    def test_same_raw_smooth_fixtures_as_actual_cpp_contract(self):
        occupancy=grid(width=400,height=400)
        occupancy.info.origin.position.x=660.;occupancy.info.origin.position.y=-85.
        directory=Path(__file__).resolve().parents[2]/'ackermann_smac/test/fixtures'
        for index in (0,1,3):
            for kind in ('raw','smooth'):
                with (directory/f'full_trial_case{index}_{kind}.csv').open() as source:
                    rows=list(csv.DictReader(source))
                points=[(float(r['x']),float(r['y'])) for r in rows]
                heads=[float(r['yaw']) for r in rows]
                for radius in (8.,12.):
                    with self.subTest(index=index,kind=kind,radius=radius):
                        self.assertEqual(validate_vehicle_path(points,heads,points,occupancy,
                            points[-1],minimum_turning_radius=radius,heading_quantization_bins=72),
                            None if kind=='raw' else 'excess_curvature')

    def test_whole_path_cannot_hide_recorded_invalid_cusp_sections(self):
        from wildos_nav2_adapter.motion_path import motion_sections
        fixture=json.loads((Path(__file__).parent/'fixtures/town_smoothing_cusp_20261014.json').read_text())
        occupancy=grid(width=400,height=400)
        occupancy.info.origin.position.x=660.;occupancy.info.origin.position.y=-85.
        for case in fixture['cases']:
            points,heads=case['points'],case['headings']
            with self.subTest(stamp=case['stamp_ns']):
                whole=validate_vehicle_path(points,heads,points,occupancy,points[-1],
                                            heading_quantization_bins=72)
                section_reasons=[validate_vehicle_path(points[a:b+1],heads[a:b+1],points,
                    occupancy,points[b],heading_quantization_bins=72) for a,b in motion_sections(points)]
                self.assertIn('excess_curvature',section_reasons)
                self.assertEqual(whole,'excess_curvature')

    def test_cusp_cannot_cancel_a_terminal_heading_violation(self):
        points=[(0.,0.),(1.,0.),(2.,0.),(3.,0.),(2.,0.),(1.,0.),(0.,0.)]
        heads=[0.,0.,0.,.2,0.,0.,0.]
        self.assertEqual(validate_vehicle_path(points,heads,points,grid(),points[-1]),'excess_curvature')

    def test_recorded_rpp_collision_matches_padded_raster_boundary(self):
        case = json.loads((Path(__file__).parent/'fixtures/town_rpp_padded_collision.json').read_text())
        occupancy = grid(width=case['shape'][1], height=case['shape'][0],
                         resolution=case['resolution'])
        occupancy.info.origin.position.x, occupancy.info.origin.position.y = case['origin']
        for x, y in case['lethal_cells']:
            occupancy.data[y*occupancy.info.width+x] = 100
        self.assertEqual(footprint_blockers(occupancy, case['position'], case['heading'], padding=0.), [])
        self.assertTrue(footprint_blockers(occupancy, case['position'], case['heading']))
        for x, y in case['lethal_cells']:
            occupancy.data[y*occupancy.info.width+x] = 99
        self.assertEqual(footprint_blockers(occupancy, case['position'], case['heading']), [])

    def test_real_smac_cusp_connector_does_not_invent_a_sharp_section(self):
        from wildos_nav2_adapter.motion_path import motion_sections
        case=json.loads((Path(__file__).parent/'fixtures/town_analytic_join.json').read_text())
        points,headings=case['points'],case['headings']
        occupancy=grid(width=400,height=400,resolution=.25)
        occupancy.info.origin.position.x=690.;occupancy.info.origin.position.y=-70.
        for first,last in [(0,len(points)-1),*motion_sections(points)]:
            p=points[first:last+1];h=headings[first:last+1]
            self.assertIsNone(validate_vehicle_path(p,h,p,occupancy,p[-1],start=p[0],heading_quantization_bins=72))
        # Obstacle detection still applies at the very connector we excluded
        # from the unreliable circle fit.
        set_cell(occupancy,*points[9],100)
        self.assertEqual(validate_vehicle_path(points,headings,points,occupancy,points[-1],
                                               heading_quantization_bins=72),'collision_cell')

    def test_rear_axle_footprint_covers_long_front_and_short_rear(self):
        occupancy = grid()
        set_cell(occupancy, 3.2, .8, 100)
        self.assertTrue(footprint_blockers(occupancy, (0.,0.), 0.))
        occupancy = grid()
        set_cell(occupancy, -1.8, .8, 100)
        self.assertEqual(footprint_blockers(occupancy, (0.,0.), 0.), [])
        # Rotating the same vehicle must rotate its offset footprint too.
        occupancy = grid()
        set_cell(occupancy, -.8, 3.2, 100)
        self.assertTrue(footprint_blockers(occupancy, (0.,0.), math.pi/2))

    def test_planning_inputs_must_be_recent(self):
        self.assertIsNone(freshness_issue(10., 9.5, 9., 1., 3.))
        self.assertEqual(freshness_issue(10., None, 9., 1., 3.),
                         ("stale_odom", None))
        issue = freshness_issue(10., 8.8, 9., 1., 3.)
        self.assertEqual(issue[0], "stale_odom")
        self.assertAlmostEqual(issue[1], 1.2)
        self.assertEqual(freshness_issue(10., 9.5, 6., 1., 3.),
                         ("stale_costmap", 4.))

    def test_beamng_clock_lead_is_accepted_only_with_explicit_tolerance(self):
        stamp = lambda ns: NS(header=NS(stamp=NS(
            sec=ns // 10**9, nanosec=ns % 10**9)))
        now = 10 * 10**9
        odom = stamp(now + 400_000_000)
        self.assertIsNone(measurement_age(odom, now, 1.0))
        self.assertEqual(measurement_age(odom, now, 1.0, future_tolerance_sec=0.5), 0.0)
        self.assertIsNone(measurement_age(
            stamp(now + 510_000_000), now, 1.0, future_tolerance_sec=0.5))

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
                         [(0., 0.), (8., 0.)], occupancy, (8., 0.), max_unknown_fraction=.2), "unknown_costmap")
        self.assertEqual(validate_vehicle_path([(0., 10.), (8., 10.)], [0., 0.],
                         [(0., 0.), (8., 0.)], occupancy, (8., 10.), corridor=9.), "outside_graph_corridor")

    def test_unknown_policy_matches_smac_and_maneuver_space_matches_vehicle_radius(self):
        occupancy = grid(width=200, height=200)
        occupancy.data = [-1] * len(occupancy.data)
        self.assertIsNone(validate_vehicle_path([(0., 0.), (8., 0.)], [0., 0.],
                         [(0., 0.), (8., 0.)], occupancy, (8., 0.)))
        self.assertTrue(route_candidates((0., 0.), [(0., 0.), (8., 0.)], occupancy))
        occupancy.data = [0] * len(occupancy.data)
        angles = [i * math.pi / 80 for i in range(81)]
        # An 8 m Ackermann turn ends 16 m away from a coarse graph chord.
        points = [(8.*math.sin(a), 8.*(1-math.cos(a))) for a in angles]
        coarse_route = [(0., 0.), (24., 0.)]
        self.assertIsNone(validate_vehicle_path(points, angles, coarse_route, occupancy, points[-1]))
        self.assertEqual(validate_vehicle_path(points, angles, coarse_route, occupancy,
                                              points[-1], corridor=9.), 'outside_graph_corridor')

    def test_path_rejects_sharp_turn_but_accepts_straight_path(self):
        occupancy = grid()
        points = [(0., 0.), (1., 0.), (2., 0.), (3., 0.), (4., 0.)]
        self.assertIsNone(validate_vehicle_path(points, [0.] * 5,
                          [(0., 0.), (4., 0.)], occupancy, (4., 0.)))
        self.assertEqual(validate_vehicle_path(points, [0., 0., 1., 1., 1.],
                         [(0., 0.), (4., 0.)], occupancy, (4., 0.)), "excess_curvature")

    def test_ideal_point_carrier_has_no_turn_radius_or_vehicle_footprint(self):
        occupancy = grid()
        set_cell(occupancy, 2.1, 0.8, 100)  # Inside the old car body, away from path centre.
        points = [(0., 0.), (1., 0.), (1., 1.), (2., 1.)]
        headings = [0., 0., math.pi/2, math.pi]
        self.assertEqual(point_blockers(occupancy, (0., 0.)), [])
        self.assertIsNone(validate_ideal_path(
            points, headings, [(0., 0.), (2., 1.)], occupancy, points[-1],
            start=points[0], target_heading=math.pi))
        self.assertIsNone(validate_ideal_local_prefix(
            points, occupancy, points[0], maximum=2.0))
        set_cell(occupancy, 0.1, 0.1, 100)
        self.assertTrue(point_blockers(occupancy, (0., 0.)))
        self.assertEqual(validate_ideal_path(
            points, headings, [(0., 0.), (2., 1.)], occupancy, points[-1],
            start=points[0]), 'collision_cell')

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
