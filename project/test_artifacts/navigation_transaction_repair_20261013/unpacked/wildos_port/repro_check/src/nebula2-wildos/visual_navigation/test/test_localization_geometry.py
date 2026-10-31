import unittest
from pathlib import Path
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth, triangulate_bearings, ground_like_surface


class LocalizationGeometryTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]])
        self.mask = np.ones((100, 100), dtype=bool)

    def surface(self, depth, u0=40, v0=40, size=5):
        return np.array([[(u-50)*depth/100, (v-50)*depth/100, depth]
                         for u in range(u0, u0+size*2, 2) for v in range(v0, v0+size*2, 2)])

    def test_supported_road_depth_is_not_an_elevated_traffic_light(self):
        # Optical camera: right, down, forward; world up is optical -Y.
        rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        ground = np.array([[x, 1.5, z] for x in np.linspace(-2., 2., 16)
                           for z in np.linspace(8., 10.5, 40)])
        quality = foreground_depth(ground, self.k, self.mask)
        self.assertIsNotNone(quality)
        self.assertTrue(ground_like_surface(quality, rotation))

    def test_elevated_surface_and_vertical_pole_are_not_ground(self):
        rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        elevated = np.array([[x, -3., z] for x in np.linspace(-2., 2., 16)
                             for z in np.linspace(12., 15., 40)])
        quality = foreground_depth(elevated, self.k, self.mask)
        self.assertIsNotNone(quality)
        self.assertFalse(ground_like_surface(quality, rotation))
        quality = foreground_depth(self.surface(20., size=8), self.k, self.mask)
        self.assertFalse(ground_like_surface(quality, rotation))

    def test_recorded_narrow_road_strip_cannot_confirm_traffic_light(self):
        # Live trial 25: 36 supported returns passed the depth gate but the
        # second spatial span was only 4 mm. It was still horizontal road.
        quality = dict(position=np.array([0., 0., -1.5]),
                       plane_normal=np.array([.0201644, .0049958, .9997842]),
                       plane_residual=.0004854, plane_span=.0042102)
        self.assertTrue(ground_like_surface(quality, np.eye(3)))
        quality['position'][2] = 3.
        self.assertFalse(ground_like_surface(quality, np.eye(3)))

    def test_foreground_and_background_do_not_make_a_median_between_surfaces(self):
        points = np.concatenate([self.surface(8), self.surface(30, 60, 60)])
        result = foreground_depth(points, self.k, self.mask)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['position'][2], 8.)
        self.assertEqual(result['points'], 25)
        self.assertFalse(result['association_validated'])
        self.assertEqual(len(result['depth_candidates']), 2)

    def test_isolated_near_returns_cannot_override_supported_object(self):
        points = np.concatenate([self.surface(12), self.surface(2, 65, 65, 2)])
        result = foreground_depth(points, self.k, self.mask)
        self.assertAlmostEqual(result['position'][2], 12.)

    def test_coherent_sparse_returns_report_lower_confidence(self):
        result = foreground_depth(self.surface(8, size=3), self.k, self.mask, dense_minimum=20)
        self.assertIsNotNone(result)
        self.assertEqual(result['points'], 9)
        self.assertLess(result['confidence'], .9)
        self.assertGreaterEqual(result['confidence'], .5)


    def test_occluded_background_returns_at_same_pixel_do_not_vote(self):
        result = foreground_depth(np.concatenate([self.surface(8), self.surface(20)]), self.k, self.mask)
        self.assertEqual(result['points'], 25)
        self.assertAlmostEqual(result['position'][2], 8.)

    def test_recorded_station_foreground_is_not_vetoed_by_background_density(self):
        fixture = Path(__file__).parent/'fixtures/station_localization_projection.npz'
        with np.load(fixture) as sample:
            result = foreground_depth(sample['points_cam'], sample['k'], sample['mask'])
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result['points'], 80)
        self.assertTrue(12. < result['position'][2] < 16.)
        self.assertLess(result['support'], .4)
        self.assertGreaterEqual(result['confidence'], .5)
        self.assertTrue(result['association_validated'])

    def test_bad_intrinsics_empty_mask_and_too_few_returns_rejected(self):
        self.assertIsNone(foreground_depth(self.surface(8), np.zeros((3, 3)), self.mask))
        self.assertIsNone(foreground_depth(self.surface(8), self.k, np.zeros_like(self.mask)))
        self.assertIsNone(foreground_depth(self.surface(8, size=2), self.k, self.mask))

    def test_recorded_dense_foreground_survives_modest_background_variation(self):
        fixture = Path(__file__).parent/'fixtures/station_localization_dense_background.npz'
        with np.load(fixture) as sample:
            result = foreground_depth(sample['points_cam'], sample['k'], sample['mask'])
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result['points'],80)
        self.assertTrue(12. < result['position'][2] < 16.)
        self.assertTrue(.2 <= result['support'] < .25)
        self.assertTrue(result['association_validated'])

    def test_minority_near_surface_below_dense_support_gate_is_rejected(self):
        points = np.concatenate([self.surface(3,10,10,5), self.surface(20,40,40,12)])
        result = foreground_depth(points,self.k,self.mask)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['position'][2],20.)

    def test_disconnected_sparse_depth_noise_cannot_localize(self):
        points = self.surface(8)
        points[:, 2] = np.arange(2., 27.) * 3.
        self.assertIsNone(foreground_depth(points, self.k, self.mask))

    def test_independent_bearings_recover_target(self):
        target = np.array([10., 2., 3.])
        views = [dict(origin=o, ray=target-np.array(o)) for o in ([0., 0., 0.], [2., 0., 0.], [2., 1., 0.])]
        result = triangulate_bearings(views)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['position'], target, atol=1e-7)

    def test_same_pose_and_parallel_rays_cannot_confirm_distance(self):
        self.assertIsNone(triangulate_bearings([dict(origin=[0, 0, 0], ray=[1, 0, 0])]*3))
        self.assertIsNone(triangulate_bearings([dict(origin=[0, i, 0], ray=[1, 0, 0]) for i in range(3)]))

    def test_inconsistent_view_and_target_behind_cameras_rejected(self):
        self.assertIsNone(triangulate_bearings([dict(origin=[0, 0, 0], ray=[-1, 1, 0]),
                                               dict(origin=[2, 0, 0], ray=[-1, -1, 0])]))
        target = np.array([10., 2., 3.])
        views = [dict(origin=o, ray=target-np.array(o)) for o in ([0., 0., 0.], [2., 0., 0.])]
        views.append(dict(origin=[3., 0., 0.], ray=[1., -1., 0.]))
        self.assertIsNone(triangulate_bearings(views))
