"""Recorded false-building and real-house regressions for feature support."""
import unittest
from pathlib import Path
import numpy as np
from visual_navigation.utils.semantic_mask import supported_mask_components


class SemanticMaskTests(unittest.TestCase):
    def test_recorded_red_building_false_positive_is_removed(self):
        mask = np.load(Path(__file__).parent/'data/semantic_false_building.npy')
        self.assertEqual(int(mask.sum()), 3)
        self.assertFalse(supported_mask_components(mask, 5).any())

    def test_recorded_green_house_remains_and_isolated_noise_is_removed(self):
        mask = np.load(Path(__file__).parent/'data/semantic_true_house.npy')
        self.assertEqual(int(mask.sum()), 13)
        checked = supported_mask_components(mask, 5)
        self.assertEqual(int(checked.sum()), 11)
        self.assertTrue(checked[0, 0, 14, 21])

    def test_disconnected_noise_cannot_pool_support(self):
        mask = np.zeros((1, 1, 10, 10), dtype=bool)
        mask[0, 0, 0, :3] = True
        mask[0, 0, 8, :3] = True
        self.assertFalse(supported_mask_components(mask, 5).any())

    def test_support_cannot_cross_cameras_or_queries(self):
        mask = np.ones((2, 2, 1, 3), dtype=bool)
        self.assertFalse(supported_mask_components(mask, 5).any())

    def test_invalid_minimum_is_rejected(self):
        for minimum in (-1, 1.5, True):
            with self.assertRaises(ValueError):
                supported_mask_components(np.ones((1, 1, 2, 2)), minimum)
