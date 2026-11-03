import unittest
import numpy as np
from visual_navigation.wildos.execution_experience_store import ExecutionExperienceStore
from visual_navigation.wildos.graph_observation_store import GraphObservationStore
from visual_navigation.wildos.confirmed_viewpoint_cache import ConfirmedViewpointCache
from visual_navigation.wildos.passive_landmark_index import PassiveLandmarkIndex


class StoreTests(unittest.TestCase):
    def test_single_supported_landmark_is_an_unconfirmed_hint(self):
        from wildos_nav2_adapter.navigation import arrived

        store = PassiveLandmarkIndex(capacity=16)
        features = np.zeros((1, 2, 8, 8), dtype=np.float32)
        features[:, 0] = 1
        camera = dict(
            K=np.array([[100, 0, 20], [0, 100, 20], [0, 0, 1]]),
            R_wc=np.eye(3),
            t_wc=np.zeros(3),
            width=40,
            height=40,
        )
        points = np.array(
            [[x / 10, y / 10, 10] for x in range(1, 5) for y in range(1, 5)]
        )
        self.assertEqual(
            store.observe(features, [camera], points, [0, 0, 0],
                          0.3, 1_000_000_000, 1, "spot1/odom"), 1,
        )
        views = store.retrieve("gas station", [1, 0], 0.8, 2)
        self.assertEqual(len(views), 1)
        view = views[0]
        self.assertFalse(view["search_confirmed"])
        self.assertEqual(view["position"], [0, 0, 0])
        self.assertEqual(view["support_landmarks"], 1)
        self.assertFalse(
            arrived((0, 0), (0, 0, 10), dict(view, query="gas station"),
                    "gas station", 2_000_000_000)
        )
        store.reject(view["id"], 2, cooldown=3)
        self.assertEqual(store.retrieve("gas station", [1, 0], 0.8, 3), [])
        self.assertTrue(store.retrieve("gas station", [1, 0], 0.8, 6))
        sparse = PassiveLandmarkIndex(capacity=16)
        self.assertEqual(
            sparse.observe(features, [camera], points[:7], [0, 0, 0],
                           0.3, 1_000_000_000, 1, "spot1/odom"), 0,
        )
        self.assertEqual(sparse.retrieve("gas station", [1, 0], 0.8, 2), [])

    def test_execution_feedback_changes_retrieved_direction_prior(self):
        store = ExecutionExperienceStore(":memory:", "test")
        try:
            vector = np.array([1, 0, 0], dtype=np.float32)
            self.assertTrue(
                store.record_feedback(
                    vector, np.ones(4, dtype=np.float32), 0.0, 0.0, 4, heading_count=4
                )
            )
            scores, similarity = store.retrieve(vector, 4)
            self.assertGreater(similarity, 0.9)
            self.assertLess(scores[0], scores[1])
        finally:
            store.close()

    def test_graph_observation_stores_label_position_and_clears_map(self):
        store = GraphObservationStore(":memory:", "test")
        try:
            store.register_map("town")
            store.observe_object("town", "gas station", [10, 20, 52], confidence=0.9)
            found = store.query_objects("town", "gas station", include_stale=True)
            self.assertEqual(len(found), 1)
            self.assertEqual(found[0].label, "gas station")
            store.clear_map("town")
            self.assertEqual(store.query_objects("town", "gas station"), [])
        finally:
            store.close()

    def test_confirmed_views_are_query_specific_and_rejections_expire(self):
        store = ConfirmedViewpointCache()
        view = store.observe(
            "gas station", "spot1/odom", [0, 0, 52], 0, 10, object_position=[5, 0, 52]
        )
        self.assertEqual(len(store.candidates("gas station", 10)), 1)
        self.assertEqual(store.candidates("house", 10), [])
        store.reject("gas station", view["id"], 11, cooldown=3)
        self.assertEqual(store.candidates("gas station", 12), [])
        self.assertEqual(len(store.candidates("gas station", 15)), 1)

    def test_passive_memory_captures_without_a_query_then_retrieves(self):
        store = PassiveLandmarkIndex(capacity=16)
        features = np.zeros((1, 2, 8, 8), dtype=np.float32)
        features[:, 0] = 1
        camera = dict(
            K=np.array([[20, 0, 20], [0, 20, 20], [0, 0, 1]]),
            R_wc=np.eye(3),
            t_wc=np.zeros(3),
            width=40,
            height=40,
        )
        points = np.array(
            [
                [x + dx, y + dy, 10]
                for x in (-1.0, 1.0)
                for y in (-1.0, 1.0)
                for dx in (-0.6, -0.3, 0, 0.3, 0.6)
                for dy in (-0.6, -0.3, 0, 0.3, 0.6)
            ]
        )
        captured = store.observe(
            features,
            [camera],
            points,
            [0, 0, 0],
            0,
            1_000_000_000,
            1,
            "spot1/odom",
            min_points=8,
        )
        self.assertGreater(captured, 0)
        self.assertLessEqual(store.nbytes, 8 * 1024 * 1024)
        self.assertTrue(store.retrieve("station", [1, 0], 0.8, 2))
        self.assertEqual(store.retrieve("unrelated", [0, 1], 0.8, 2), [])
        count, memory_bytes = store.count, store.nbytes
        # A weak query match must lose to a stronger competing meaning.
        self.assertTrue(store.retrieve("weak match", [0.8, 0.6], 0.7, 2))
        self.assertEqual(
            store.retrieve(
                "weak match", [0.8, 0.6], 0.7, 2, negative_text_features=[[1, 0]]
            ),
            [],
        )
        # Retrieval policy must not delete descriptors needed by a later query.
        self.assertTrue(
            store.retrieve("station", [1, 0], 0.8, 3, negative_text_features=[[0, 1]])
        )
        self.assertEqual((store.count, store.nbytes), (count, memory_bytes))
