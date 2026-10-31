"""Independent LiDAR ranges locate points even when a plane normal is unobservable."""
from pathlib import Path
import unittest
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import (
    foreground_depth, ground_like_surface)


class ScanlineDepthTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]])
        self.mask = np.ones((100, 100), dtype=bool)

    def scanline(self, depth=8., vertical=False):
        pixels = [(50, v) for v in range(40, 56, 2)] if vertical else [
            (u, 50) for u in range(40, 56, 2)]
        return np.array([[(u-50)*depth/100, (v-50)*depth/100, depth] for u, v in pixels])

    def test_recorded_supported_scanline_provides_measured_position(self):
        with np.load(Path(__file__).parent/'fixtures/scanline_depth_v46.npz') as fixture:
            points, k, mask = fixture['points_cam'], fixture['k'], fixture['mask']
        result = foreground_depth(points, k, mask)
        self.assertIsNotNone(result)
        self.assertTrue(result['association_validated'])
        self.assertGreaterEqual(result['points'], 8)
        self.assertIsInstance(result['plane_normal_reliable'], bool)
        self.assertLess(np.linalg.norm(points-result['position'], axis=1).min(), 1e-8)
        pixel = k@result['position']
        u, v = np.floor(pixel[:2]/pixel[2]).astype(int)
        self.assertTrue(mask[v, u])
        self.assertTrue(np.all(np.isfinite(result['covariance'])))

    def test_horizontal_independent_returns_locate_without_claiming_plane(self):
        result = foreground_depth(self.scanline(), self.k, self.mask)
        self.assertIsNotNone(result)
        self.assertEqual(result['points'], 8)
        self.assertAlmostEqual(result['position'][2], 8.)
        self.assertFalse(result['plane_normal_reliable'])

    def test_vertical_independent_returns_locate_without_claiming_plane(self):
        result = foreground_depth(self.scanline(vertical=True), self.k, self.mask)
        self.assertIsNotNone(result)
        self.assertEqual(result['points'], 8)
        self.assertFalse(result['plane_normal_reliable'])

    def test_repeated_returns_at_one_pixel_do_not_count_as_independent(self):
        points = np.repeat([[0., 0., 8.]], 50, axis=0)
        self.assertIsNone(foreground_depth(points, self.k, self.mask))

    def test_multiple_scanline_surfaces_remain_ambiguous(self):
        far = self.scanline(20., vertical=True)
        far[:, 0] += 4.  # Separate image columns: no occluded shared pixel.
        points = np.concatenate([self.scanline(8.), far])
        result = foreground_depth(points, self.k, self.mask)
        self.assertIsNotNone(result)
        self.assertFalse(result['association_validated'])
        self.assertEqual(len(result['depth_candidates']), 2)

    def test_unobservable_normal_cannot_clear_a_below_camera_ground_return(self):
        quality = dict(position=np.array([10., 0., -1.5]),
            plane_normal=np.array([1., 0., 0.]), plane_residual=0., plane_span=0.,
            plane_normal_reliable=False)
        self.assertTrue(ground_like_surface(quality, np.eye(3)))
        quality['position'][2] = 3.
        self.assertFalse(ground_like_surface(quality, np.eye(3)))

    def test_full_surface_retains_reliable_plane_and_geometric_support(self):
        points = np.array([[x, y, 8.] for x in np.linspace(-1., 1., 8)
                           for y in np.linspace(-1., 1., 8)])
        result = foreground_depth(points, self.k, self.mask)
        self.assertIsNotNone(result)
        self.assertIs(result.get('plane_normal_reliable'), True)
        self.assertTrue(result['association_validated'])


if __name__ == '__main__':
    unittest.main()
