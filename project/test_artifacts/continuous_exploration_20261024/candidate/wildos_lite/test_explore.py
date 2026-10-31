import unittest

import numpy as np

from wildos_lite import explore


class FrontierConfTests(unittest.TestCase):
    def test_thresholds_zero_weak_and_nontraversable(self):
        f = np.array([[0.1, 0.9], [0.95, 0.7]], dtype=float)
        t = np.array([[0.5, 0.5], [0.9, 0.82]], dtype=float)
        conf = explore.frontier_conf(f, t)
        self.assertEqual(conf[0, 0], 0.0)  # weak, also non-traversable
        self.assertEqual(conf[0, 1], 0.0)  # strong frontier but trav below 0.8
        self.assertAlmostEqual(conf[1, 0], 0.95)
        self.assertAlmostEqual(conf[1, 1], 0.7)  # below frontier 0.6? no (0.7), kept

    def test_none_traversability_leaves_frontier(self):
        f = np.full((4, 4), 0.9)
        conf = explore.frontier_conf(f, None)
        np.testing.assert_allclose(conf, 0.9)


class HeadingSelectionTests(unittest.TestCase):
    def test_explores_toward_right_frontier(self):
        h, w = 64, 96
        f = np.zeros((h, w))
        f[24:40, 75:88] = 0.9
        t = np.full((h, w), 0.85)
        heading, score, per_bin = explore.best_exploration_heading(
            explore.frontier_conf(f, t), t, num_bins=16)
        self.assertGreater(np.degrees(heading), 0.0)  # right of centre
        self.assertGreater(score, 0.8)

    def test_explores_toward_left_frontier(self):
        h, w = 64, 96
        f = np.zeros((h, w))
        f[24:40, 10:22] = 0.9
        t = np.full((h, w), 0.85)
        heading, _, _ = explore.best_exploration_heading(
            explore.frontier_conf(f, t), t, num_bins=16)
        self.assertLess(np.degrees(heading), 0.0)

    def test_empty_frontier_returns_forward(self):
        f = np.zeros((64, 96))
        t = np.full((64, 96), 0.9)
        heading, score, _ = explore.best_exploration_heading(
            explore.frontier_conf(f, t), t, num_bins=16)
        self.assertAlmostEqual(np.degrees(heading), 0.0, places=4)
        self.assertAlmostEqual(score, 0.0, places=4)

    def test_heading_bins_cover_full_circle(self):
        headings = explore.angular_headings(8)
        self.assertEqual(headings.shape, (8, 2))
        norms = np.linalg.norm(headings, axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-6)


if __name__ == "__main__":
    unittest.main()