"""Synthetic arrays ONLY: these tests do not validate model inference."""
import unittest
import numpy as np
from query import postprocess


class SyntheticPostprocessTests(unittest.TestCase):
    def test_threshold_strict_and_no_minmax(self):
        mask, kept, regions = postprocess(np.full((5, 5), 0.09), 0.09, 1)
        self.assertEqual(int(mask.sum()), 0)
        self.assertEqual(regions, [])

    def test_boxes_scores_and_area(self):
        a = np.zeros((8, 9), dtype=np.float32)
        a[1:3, 2:5] = 0.3
        a[6, 8] = 0.9
        mask, kept, regions = postprocess(a, 0.09, 2)
        self.assertEqual(int(mask.sum()), 7)
        self.assertEqual(int(kept.sum()), 6)
        self.assertEqual(regions[0]["box_xyxy"], [2, 1, 5, 3])
        self.assertAlmostEqual(regions[0]["mean_cosine"], 0.3)

    def test_eight_connectivity(self):
        _, _, regions = postprocess(np.eye(3), 0.5, 1)
        self.assertEqual(len(regions), 1)
        self.assertEqual(regions[0]["area_pixels"], 3)

    def test_negative_scores_not_rescaled(self):
        _, _, regions = postprocess(np.array([[-0.8, -0.1]]), 0.09, 1)
        self.assertEqual(regions, [])

    def test_invalid(self):
        for a in (np.array([[np.nan]]), np.zeros((1, 2, 3)), np.zeros((0, 0))):
            with self.assertRaises(ValueError):
                postprocess(a, 0.09, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
