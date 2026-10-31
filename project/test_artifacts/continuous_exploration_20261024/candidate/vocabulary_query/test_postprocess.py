"""Synthetic arrays ONLY: these tests do not validate model inference."""
import unittest
import numpy as np
from PIL import Image
from query import postprocess, heatmap_colors


class HeatmapColorTests(unittest.TestCase):
    def test_relative_uses_data_range(self):
        a = np.linspace(0.01, 0.12, 25, dtype=np.float32).reshape(5, 5)
        vmin, vmax, image = heatmap_colors(a, "relative")
        self.assertAlmostEqual(vmin, 0.01)
        self.assertAlmostEqual(vmax, 0.12)
        colors = np.asarray(image.convert("RGB")).reshape(-1, 3)
        self.assertGreater(len(np.unique(colors, axis=0)), 3)

    def test_fixed_and_invalid(self):
        _, _, flat = heatmap_colors(np.full((2, 2), 0.5, dtype=np.float32), "fixed")
        self.assertEqual(np.asarray(flat.convert("RGB")).reshape(-1, 3)[0].tolist(),
                         np.asarray(flat.convert("RGB")).reshape(-1, 3)[-1].tolist())
        for mode, a in (("relative", np.full((2, 2), 0.3, dtype=np.float32)),
                        ("relative", np.array([[np.nan]], dtype=np.float32))):
            with self.assertRaises(ValueError):
                heatmap_colors(a, mode)
        with self.assertRaises(ValueError):
            heatmap_colors(np.zeros((1, 1), dtype=np.float32), "bogus")

    def test_smooth_keeps_size_and_scores(self):
        a = np.linspace(0.01, 0.12, 25, dtype=np.float32).reshape(5, 5)
        _, _, rough = heatmap_colors(a, "relative")
        _, _, smooth = heatmap_colors(a, "relative", smooth=True)
        self.assertEqual(rough.size, smooth.size)
        with open(__import__("query").__file__, encoding="utf-8") as f:
            self.assertIn("--smooth", f.read())

    def test_region_overlay_mode_argument(self):
        import query
        with open(query.__file__, encoding="utf-8") as stream:
            self.assertIn("--region-overlay", stream.read())


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
