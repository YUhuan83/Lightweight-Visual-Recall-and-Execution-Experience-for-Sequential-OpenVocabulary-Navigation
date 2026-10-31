import numpy as np
import unittest

from visual_navigation.utils.object_search_scoring import (
    bias_frontiers_on_traversable_mask,
)


class TestObjectSearchScoring(unittest.TestCase):
    def test_object_mask_cannot_make_nontraversable_pixels_attractive(self):
        frontier_scores = np.array([[[[0.6, 0.1, 0.2]]]], dtype=np.float32)
        traversability = np.array([[[[0.2, 0.79, 0.9]]]], dtype=np.float32)
        object_mask = np.ones_like(frontier_scores, dtype=np.uint8)
        original_frontiers = frontier_scores.copy()
        original_traversability = traversability.copy()

        result = bias_frontiers_on_traversable_mask(
            frontier_scores,
            traversability,
            object_mask,
            object_frontier_score=0.9,
            traversability_threshold=0.8,
        )

        np.testing.assert_allclose(result, [[[[0.6, 0.1, 0.9]]]])
        np.testing.assert_array_equal(frontier_scores, original_frontiers)
        np.testing.assert_array_equal(traversability, original_traversability)

    def test_object_mask_with_invalid_shapes_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "shapes must match"):
            bias_frontiers_on_traversable_mask(
                np.zeros((2, 2)),
                np.zeros((2, 2)),
                np.zeros((2, 1)),
                object_frontier_score=0.9,
                traversability_threshold=0.8,
            )
