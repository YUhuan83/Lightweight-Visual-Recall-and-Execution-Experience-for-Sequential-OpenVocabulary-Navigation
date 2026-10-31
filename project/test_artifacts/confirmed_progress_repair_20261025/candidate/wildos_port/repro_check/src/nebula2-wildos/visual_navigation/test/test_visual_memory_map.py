import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from visual_navigation.wildos.graph_observation_store import GraphObservationStore


class VisualMemoryMapTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.tmp.name) / "memory.sqlite3")
        self.memory = GraphObservationStore(self.path, "radio-test", visual_similarity_threshold=0.7)
        self.memory.register_map("ring", "Ring", timestamp=100.0)

    def tearDown(self):
        self.memory.close()
        self.tmp.cleanup()

    def test_node_heading_aware_retrieval(self):
        emb = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        self.memory.observe_node_heading("ring", "n1", 0, 0.0, (0, 0, 0), emb, timestamp=101.0)
        self.memory.observe_node_heading("ring", "n2", 8, math.pi, (5, 0, 0), emb, timestamp=102.0)
        results = self.memory.query_node_observations("ring", emb, yaw=0.1, top_k=2)
        self.assertEqual(results[0].node_uuid, "n1")
        self.assertEqual({item.node_uuid for item in results}, {"n1", "n2"})
        self.memory.observe_node_heading(
            "ring", "n1", 0, 0.0, (0, 0, 0), emb,
            frontier_score=0.8, timestamp=103.0)
        observation = self.memory.get_node_observation("ring", "n1", 0)
        self.assertEqual(observation["observation_count"], 2)

    def test_identify_maps(self):
        emb = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        self.memory.observe_node_heading("ring", "n1", 0, 0.0, (0, 0, 0), emb, timestamp=101.0)
        self.memory.register_map("other", "Other", timestamp=100.0)
        self.memory.observe_node_heading("other", "n2", 0, 0.0, (0, 0, 0),
                                         np.array([0.0, 1.0, 0.0]), timestamp=101.0)
        result = self.memory.identify_maps(emb, yaw=0.0)
        self.assertEqual(result[0].map_id, "ring")

    def test_node_observation_update_policy(self):
        emb = np.array([1.0, 0.0], dtype=np.float32)
        self.assertFalse(GraphObservationStore.should_update_node_observation(
            "n1", 0, emb, "n1", 0, emb))
        self.assertTrue(GraphObservationStore.should_update_node_observation(
            "n1", 0, emb, "n2", 0, emb))
        self.assertTrue(GraphObservationStore.should_update_node_observation(
            "n1", 0, emb, "n1", 1, emb))

    def test_object_association_and_fusion(self):
        emb = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        object_id = self.memory.observe_object(
            "ring", "house", (10, 5, 0), emb, 0.8,
            anchor_node_uuid="n10", anchor_heading_bin=2,
            observer_pose=(7, 5, 0, 0), best_view_node_uuid="n7", best_view_heading_bin=2,
            approach_pose=(8, 5, 0, 0), approach_node_uuid="n8", approach_heading_bin=2,
            timestamp=110.0)
        same_id = self.memory.observe_object(
            "ring", "house", (10.5, 5, 0), emb, 0.9,
            anchor_node_uuid="n11", anchor_heading_bin=3,
            observer_pose=(7.5, 5, 0, 0), approach_pose=(8.5, 5, 0, 0), timestamp=111.0)
        self.assertEqual(object_id, same_id)
        objects = self.memory.query_objects("ring", "house", now=111.0)
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0].observation_count, 2)
        self.assertGreater(objects[0].position[0], 10.0)
        self.assertEqual(objects[0].anchor_node_uuid, "n10")
        self.assertEqual(objects[0].approach_node_uuid, "n8")
        self.assertEqual(objects[0].status, "CONFIRMED")
        self.assertEqual(objects[0].independent_observation_count, 2)
        self.assertEqual(objects[0].unique_node_count, 2)
        goal = self.memory.query_memory_goal("ring", "house", min_confidence=0.1, now=111.0)
        self.assertEqual(goal.object_id, object_id)
        self.memory.mark_object(object_id, reached=True, blocked=True)
        marked = self.memory.query_objects("ring", "house", now=111.0)[0]
        self.assertTrue(marked.blocked)

    def test_far_object_creates_new_identity(self):
        emb = np.array([1.0, 1.0], dtype=np.float32)
        first = self.memory.observe_object("ring", "house", (0, 0, 0), emb, timestamp=110.0)
        second = self.memory.observe_object("ring", "house", (20, 0, 0), emb, timestamp=111.0)
        self.assertNotEqual(first, second)

    def test_repeated_view_stays_tentative(self):
        emb = np.array([1.0, 0.0], dtype=np.float32)
        object_id = self.memory.observe_object(
            "ring", "door", (1, 1, 0), emb, 0.8,
            anchor_node_uuid="n1", anchor_heading_bin=2, timestamp=110.0)
        same_id = self.memory.observe_object(
            "ring", "door", (1.1, 1, 0), emb, 0.9,
            anchor_node_uuid="n1", anchor_heading_bin=2, timestamp=111.0)
        self.assertEqual(object_id, same_id)
        obj = self.memory.query_objects("ring", "door", now=111.0)[0]
        self.assertEqual(obj.status, "TENTATIVE")
        self.assertEqual(obj.observation_count, 2)
        self.assertEqual(obj.independent_observation_count, 1)
        self.assertIsNone(self.memory.query_memory_goal(
            "ring", "door", min_confidence=0.1, now=111.0))

    def test_two_view_prototypes_and_confirmed_heading_bonus(self):
        first = np.array([1.0, 0.0], dtype=np.float32)
        second = np.array([0.85, math.sqrt(1.0 - 0.85 ** 2)], dtype=np.float32)
        object_id = self.memory.observe_object(
            "ring", "flag", (2, 2, 0), first, 0.9,
            anchor_node_uuid="n1", anchor_heading_bin=4,
            best_view_node_uuid="n1", best_view_heading_bin=4, timestamp=110.0)
        same_id = self.memory.observe_object(
            "ring", "flag", (2.1, 2, 0), second, 0.9,
            anchor_node_uuid="n2", anchor_heading_bin=5, timestamp=111.0)
        self.assertEqual(object_id, same_id)
        obj = self.memory.query_objects("ring", "flag", now=111.0)[0]
        self.assertEqual(obj.status, "CONFIRMED")
        self.assertEqual(obj.prototype_count, 2)
        bonus = self.memory.confirmed_heading_bonus("ring", "flag", "n1", 16)
        self.assertGreater(bonus[4], 0.0)
        self.assertGreater(bonus[3], 0.0)
        self.assertEqual(float(bonus[10]), 0.0)

    def test_relation_query_uses_persistent_objects(self):
        for idx, x in enumerate((-10.0, 5.0, 20.0)):
            self.memory.observe_object(
                "ring", "house", (x, 5.0, 0), np.array([1, idx + 1], dtype=np.float32),
                confidence=0.9, timestamp=110.0 + idx)
        result = self.memory.resolve_object_relation(
            "ring", "左侧第三个房子", (-20, 0, 0), (1, 0, 0),
            min_confidence=0.1, now=113.0)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.position[0], 20.0)

    def test_negative_search_coverage_merges_headings(self):
        self.memory.record_search_coverage("ring", "door", "n1", [0, 2], 0.7, timestamp=120.0, heading_bin_count=4)
        self.memory.record_search_coverage("ring", "door", "n1", [1, 2], 0.9, timestamp=121.0, heading_bin_count=4)
        result = self.memory.get_search_coverage("ring", "door", "n1")
        self.assertEqual(result["heading_mask"], 0b111)
        self.assertEqual(result["observation_count"], 2)
        self.assertAlmostEqual(result["negative_evidence"], 0.9)
        self.assertAlmostEqual(self.memory.search_penalty("ring", "door", "n1", 4), 0.625)

    def test_traversal_statistics(self):
        self.memory.record_traversal("ring", "a", "b", 1, duration=2.0, observed_distance=4.0)
        self.memory.record_traversal("ring", "a", "b", 4, duration=3.0, observed_distance=0.2)
        stats = self.memory.traversal_stats("ring", "a", "b")
        self.assertEqual(stats["attempts"], 2)
        self.assertEqual(stats["successes"], 1)
        self.assertAlmostEqual(stats["success_probability"], 0.5)
        self.assertAlmostEqual(stats["mean_duration"], 2.5)
        self.memory.record_traversal(
            "ring", "a", "b", 1, start_heading_bin=2, end_heading_bin=3)
        direction_stats = self.memory.traversal_stats(
            "ring", "a", "b", start_heading_bin=2, end_heading_bin=3)
        self.assertEqual(direction_stats["attempts"], 1)
        self.assertEqual(direction_stats["successes"], 1)

    def test_dynamic_memory_decays_quickly(self):
        self.memory.observe_object(
            "ring", "vehicle", (0, 0, 0), np.array([1, 0]),
            confidence=1.0, dynamic_probability=1.0, timestamp=100.0)
        fresh = self.memory.query_objects("ring", "vehicle", min_confidence=0.2, now=100.0)
        stale = self.memory.query_objects("ring", "vehicle", min_confidence=0.2, now=140.0)
        self.assertEqual(len(fresh), 1)
        self.assertEqual(stale, [])


if __name__ == "__main__":
    unittest.main()
