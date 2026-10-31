import unittest

import numpy as np

from object_search import triangulation as T


def camera(width=640, height=480, focal=500.0, cx=320.0, cy=240.0):
    return np.array([[focal, 0.0, cx], [0.0, focal, cy], [0.0, 0.0, 1.0]])


class RayTest(unittest.TestCase):
    def test_point_on_ray_has_zero_distance(self):
        ray = T.Ray(origin=[0, 0, 0], direction=[0, 0, 1])
        d = ray.distance_to_points(np.array([[0.0, 0.0, 5.0], [1.0, 0.0, 5.0]]))
        self.assertAlmostEqual(d[0], 0.0, places=9)
        self.assertAlmostEqual(d[1], 1.0, places=9)

    def test_point_behind_origin_is_infinite(self):
        ray = T.Ray(origin=[0, 0, 0], direction=[0, 0, 1])
        d = ray.distance_to_points(np.array([[0.0, 0.0, -3.0]]))
        self.assertTrue(np.isinf(d[0]))

    def test_bbox_ray_points_at_box_centre(self):
        K = camera()
        ray = T.ray_from_bbox([300, 220, 340, 260], K, np.eye(3), np.zeros(3))
        self.assertTrue(np.allclose(ray.origin, [0, 0, 0]))
        self.assertAlmostEqual(ray.direction[0], 0.0, places=9)
        self.assertTrue(ray.direction[2] > 0)


class InRangeTest(unittest.TestCase):
    def test_median_depth_recovers_target(self):
        K = camera()
        target = np.array([0.0, 0.0, 5.0])
        rng = np.random.default_rng(0)
        cloud = target + rng.normal(0, 0.05, size=(80, 3))
        mask = np.zeros((480, 640), dtype=bool)
        mask[230:250, 310:330] = True
        position = T.localize_in_range(cloud, mask, K, np.eye(3), np.zeros(3), min_points=10)
        self.assertIsNotNone(position)
        self.assertLess(np.linalg.norm(position - target), 0.2)

    def test_returns_none_without_enough_points(self):
        K = camera()
        cloud = np.array([[0.0, 0.0, 5.0]])
        mask = np.zeros((480, 640), dtype=bool)
        mask[239, 319] = True
        self.assertIsNone(T.localize_in_range(cloud, mask, K, np.eye(3), np.zeros(3), min_points=10))


class ParticleTriangulationTest(unittest.TestCase):
    def test_two_view_triangulation_recovers_target(self):
        K = camera()
        target = np.array([0.0, 0.0, 5.0])
        # View A at origin looking +Z; View B shifted +X, same orientation.
        view_a = T.View(mask=None, K=K, R_wc=np.eye(3), t_wc=np.zeros(3))
        view_b = T.View(mask=None, K=K, R_wc=np.eye(3), t_wc=np.array([2.0, 0.0, 0.0]))
        # Masks cover the projected target in each view.
        pix_a = K @ target
        pix_a = pix_a[:2] / pix_a[2]
        pix_b = K @ (target - view_b.t_wc)
        pix_b = pix_b[:2] / pix_b[2]
        view_a.mask = np.zeros((480, 640), dtype=bool)
        view_a.mask[int(pix_a[1]) - 3:int(pix_a[1]) + 4, int(pix_a[0]) - 3:int(pix_a[0]) + 4] = True
        view_b.mask = np.zeros((480, 640), dtype=bool)
        view_b.mask[int(pix_b[1]) - 3:int(pix_b[1]) + 4, int(pix_b[0]) - 3:int(pix_b[0]) + 4] = True
        position, method = T.localize([view_a, view_b], cloud_world=None, num_particles=20000,
                                      depth_range=(3.0, 7.0), seed=0)
        self.assertEqual(method, "particle_triangulation")
        self.assertLess(np.linalg.norm(position - target), 0.6)

    def test_localize_prefers_lidar_when_in_range(self):
        K = camera()
        target = np.array([0.0, 0.0, 5.0])
        rng = np.random.default_rng(1)
        cloud = target + rng.normal(0, 0.03, size=(100, 3))
        mask = np.zeros((480, 640), dtype=bool)
        mask[230:250, 310:330] = True
        view = T.View(mask=mask, K=K, R_wc=np.eye(3), t_wc=np.zeros(3))
        position, method = T.localize([view], cloud_world=cloud, min_lidar_points=10)
        self.assertEqual(method, "lidar_median_depth")
        self.assertLess(np.linalg.norm(position - target), 0.2)


def three_views(target, offsets=(-2.0, 0.0, 2.0)):
    K = camera()
    views = []
    for tx in offsets:
        t_wc = np.array([tx, 0.0, 0.0])
        pixel = K @ (target - t_wc)
        pixel = pixel[:2] / pixel[2]
        mask = np.zeros((480, 640), dtype=bool)
        mask[int(pixel[1]) - 3:int(pixel[1]) + 4, int(pixel[0]) - 3:int(pixel[0]) + 4] = True
        views.append(T.View(mask=mask, K=K, R_wc=np.eye(3), t_wc=t_wc))
    return views


class HypothesisTest(unittest.TestCase):
    def test_wide_prior_is_biased(self):
        target = np.array([0.0, 0.0, 5.0])
        position, _ = T.localize(three_views(target), cloud_world=None, num_particles=20000,
                                 depth_range=(1.0, 100.0), seed=0)
        self.assertGreater(np.linalg.norm(position - target), 5.0)

    def test_depth_hypothesis_converges(self):
        target = np.array([0.0, 0.0, 5.0])
        hypothesis = {"location": target.tolist()}
        position, _ = T.localize(three_views(target), cloud_world=None, num_particles=20000,
                                 depth_range=(1.0, 100.0), seed=0,
                                 depth_hypothesis=hypothesis, distribution="gaussian_hyp",
                                 hypothesis_std=1.0)
        self.assertLess(np.linalg.norm(position - target), 0.3)

    def test_uniform_hypothesis_and_minimum_method(self):
        target = np.array([0.0, 0.0, 5.0])
        hypothesis = {"location": target.tolist()}
        position, _ = T.localize(three_views(target), cloud_world=None, num_particles=20000,
                                 depth_range=(1.0, 100.0), seed=0,
                                 depth_hypothesis=hypothesis, distribution="uniform_hyp",
                                 hypothesis_std=1.5, method="minimum")
        self.assertLess(np.linalg.norm(position - target), 0.5)

    def test_odom_drift_changes_pose(self):
        rng = np.random.default_rng(0)
        R_wc, t_wc = T.add_odom_drift(np.eye(3), np.zeros(3), rng, t_drift_std=1.0, r_drift_std_deg=2.0)
        self.assertFalse(np.allclose(R_wc, np.eye(3)))
        self.assertGreater(np.linalg.norm(t_wc), 0.0)



if __name__ == "__main__":
    unittest.main()
