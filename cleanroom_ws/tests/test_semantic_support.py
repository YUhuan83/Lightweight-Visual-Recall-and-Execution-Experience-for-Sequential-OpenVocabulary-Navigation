"""Real-image feature regressions; run with the deployed vision Python."""

from pathlib import Path
import unittest
import numpy as np

try:
    import torch
except ImportError:
    torch = None

if torch is not None:
    from visual_navigation.wildos.passive_landmark_index import PassiveLandmarkIndex
    from visual_navigation.utils.object_search_utils import (
        competing_object_queries,
        localize_query,
    )


@unittest.skipIf(torch is None, "Run this component with the vision Python")
class SemanticSupportTests(unittest.TestCase):
    def setUp(self):
        self.fixture = np.load(Path(__file__).parent / "data/semantic_support.npz")
        self.queries = self.fixture["queries"].tolist()
        self.features = torch.from_numpy(self.fixture["text_features"])
        self.confusers = [
            "stop sign",
            "gas station sign",
            "road sign",
            "street lamp",
            "tree",
            "building",
            "sky",
        ]

    def tearDown(self):
        self.fixture.close()

    def mask(self, name, query, competitive=True):
        index = self.queries.index(query)
        negatives = competing_object_queries(query, self.confusers)
        _, mask = localize_query(
            self.features[index : index + 1],
            torch.from_numpy(self.fixture[name]),
            (640, 960),
            False,
            0.08,
            negative_text_feats=self.features[
                [self.queries.index(q) for q in negatives]
            ]
            if competitive
            else None,
            minimum_competitive_cells=5 if competitive else 0,
            minimum_component_cells=5,
        )
        return int(mask.sum())

    def test_real_false_bus_stop_mask_is_rejected(self):
        self.assertGreater(self.mask("bus_negative", "bus stop", False), 0)
        self.assertEqual(self.mask("bus_negative", "bus stop"), 0)
        self.assertEqual(self.mask("bench_negative", "bus stop shelter"), 0)

    def test_real_targets_survive_including_background_as_query(self):
        for name, query in [
            ("bus_positive", "bus stop"),
            ("station_positive", "gas station"),
            ("house_positive", "house"),
            ("road_positive", "road"),
            ("tree_positive", "tree"),
        ]:
            with self.subTest(query=query):
                baseline = self.mask(name, query, False)
                self.assertGreater(baseline, 0)
                self.assertGreater(self.mask(name, query), baseline * 0.8)
        self.assertGreater(self.mask("signal_positive", "traffic light"), 0)
        self.assertGreater(self.mask("shelter_positive", "bus stop shelter"), 0)
        # Existing colored-object discrimination is preserved exactly.
        alternatives = competing_object_queries("green house", self.confusers)
        self.assertIn("blue house", alternatives)
        self.assertNotIn("building", alternatives)

    def test_recall_uses_same_real_feature_semantics_as_live_masks(self):
        text = self.fixture["text_features"]
        target = text[self.queries.index("bus stop")]
        competitors = competing_object_queries("bus stop", self.confusers)
        negative = text[[self.queries.index(q) for q in competitors]]
        store = PassiveLandmarkIndex(capacity=32)
        points = np.array(
            [
                [x + dx, y + dy, 10]
                for x in (-1.0, 1.0)
                for y in (-1.0, 1.0)
                for dx in (-0.6, -0.3, 0, 0.3, 0.6)
                for dy in (-0.6, -0.3, 0, 0.3, 0.6)
            ]
        )
        for step, name in enumerate(("bus_negative", "bus_positive")):
            spatial = self.fixture[name][0]
            # Use an actually measured descriptor, not a synthesized class vector.
            scores = np.einsum("d,dhw->hw", target, spatial)
            if step:
                margin = scores - np.einsum("qd,dhw->qhw", negative, spatial).max(
                    axis=0
                )
                index = np.unravel_index(np.argmax(margin), margin.shape)
            else:
                index = np.unravel_index(np.argmax(scores), scores.shape)
            descriptor = spatial[:, index[0], index[1]]
            features = np.broadcast_to(
                descriptor[None, :, None, None], (1, len(target), 8, 8)
            )
            origin = np.array([step * 20.0, 0, 0])
            camera = dict(
                K=np.array([[20, 0, 20], [0, 20, 20], [0, 0, 1]]),
                R_wc=np.eye(3),
                t_wc=origin,
                width=40,
                height=40,
            )
            self.assertGreater(
                store.observe(
                    features,
                    [camera],
                    points + origin,
                    origin,
                    0,
                    (step + 1) * 1_000_000_000,
                    step + 1,
                    "spot1/odom",
                ),
                0,
            )
        raw = store.retrieve("bus stop", target, 0.08, 3)
        self.assertTrue(any(v["position"][0] == 0 for v in raw))
        matched = store.retrieve(
            "bus stop", target, 0.08, 3, negative_text_features=negative
        )
        self.assertTrue(matched)
        self.assertTrue(all(v["position"][0] == 20 for v in matched))


if __name__ == "__main__":
    unittest.main()
