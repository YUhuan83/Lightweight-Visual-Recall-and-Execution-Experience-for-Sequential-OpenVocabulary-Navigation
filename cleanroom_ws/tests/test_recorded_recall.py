"""Replay the two real eligible records from trial 20261102_104721."""
from pathlib import Path
import unittest
import numpy as np
from visual_navigation.wildos.passive_landmark_index import PassiveLandmarkIndex


class RecordedRecallTests(unittest.TestCase):
    def test_separated_gas_station_records_recall_original_view(self):
        with np.load(Path(__file__).parent / "data/passive_gas_recall.npz") as data:
            store = PassiveLandmarkIndex(capacity=2)
            keys = ("features", "position", "observer", "key", "stamp",
                    "last_seen", "count", "id", "support", "fov")
            store.arrays = {key: data[key].copy() for key in keys}
            store.count, store.frame = 2, "spot1/odom"
            views = store.retrieve("gas station", data["text"], float(data["threshold"]),
                                   float(data["now"]), negative_text_features=data["negative"],
                                   semantic_margin=float(data["margin"]))
            self.assertEqual(len(views), 2)
            self.assertEqual(views[0]["id"], "passive:gas station:33")
            self.assertTrue(all(not view["search_confirmed"] for view in views))
            np.testing.assert_allclose(views[0]["position"], data["observer"][0, :3])
            np.testing.assert_allclose(views[0]["object_position"], data["position"][0])
            self.assertEqual(store.retrieve("unsupported", -data["text"],
                                           float(data["threshold"]), float(data["now"])), [])
