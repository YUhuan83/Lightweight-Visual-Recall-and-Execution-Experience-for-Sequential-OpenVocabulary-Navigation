import unittest
import numpy as np
from visual_navigation.wildos.semantic_landmark_memory import SemanticLandmarkMemory


def scene():
    features = np.zeros((1, 2, 2, 4), dtype=np.float32)
    features[:, 0] = 1
    features[0, :, :, 1:3] = np.asarray([0., 1.])[:, None, None]
    camera = dict(K=np.array([[100., 0., 60.], [0., 100., 40.], [0., 0., 1.]]),
                  R_wc=np.eye(3), t_wc=np.array([100., 0., 0.]), width=120, height=80)
    points = np.asarray([[100+x+dx, dy, 10.] for x in (-1.6, 1.2)
                         for dx in np.linspace(-.18, .18, 4)
                         for dy in np.linspace(.1, .46, 4)])
    return features, [camera], points


class SemanticLandmarkTests(unittest.TestCase):
    def capture(self, memory, stamp=100_000_000_000, position=(100., 0., 0.), now=100., **changes):
        features, cameras, points = scene()
        return memory.observe(changes.get('features', features), cameras, changes.get('points', points),
                              position, 0., stamp, now, 'spot1/odom')

    def test_unqueried_scene_can_later_retrieve_station_region_without_confirming_it(self):
        memory = SemanticLandmarkMemory()
        self.assertGreater(self.capture(memory), 0)
        # Capture accepts no query or object label. Only later is a text target supplied.
        result = memory.retrieve('gas station', [0., 1.], .5, now=400.)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]['object_position'][0], 99.8, places=1)
        self.assertAlmostEqual(result[0]['object_position'][2], 10.)
        self.assertFalse(result[0]['search_confirmed'])
        self.assertEqual(result[0]['observation_stamp_ns'], 100_000_000_000)
        self.assertEqual(memory.retrieve('house', [1., 0.], .5, 400.), [])

    def test_repeated_pose_does_not_grow_memory_or_observation_count(self):
        memory = SemanticLandmarkMemory()
        self.capture(memory)
        before = memory.arrays['count'].copy()
        self.assertEqual(self.capture(memory, stamp=101_000_000_000, now=101.), 0)
        np.testing.assert_array_equal(before, memory.arrays['count'])
        self.assertEqual(memory.count, 2)

    def test_fixed_storage_byte_and_capacity_limits_survive_long_capture_sequence(self):
        memory = SemanticLandmarkMemory(capacity=8, max_bytes=1024)
        for n in range(100):
            features, cameras, points = scene()
            offset = 4*n
            cameras[0]['t_wc'][0] += offset
            points[:, 0] += offset
            memory.observe(features, cameras, points, [100.+offset, 0., 0.], 0.,
                           (100+n)*10**9, 100.+n, 'spot1/odom')
        self.assertLessEqual(memory.count, 8)
        self.assertLessEqual(memory.nbytes, 1024)
        self.assertEqual(memory.nbytes, sum(a.nbytes for a in memory.arrays.values()))

    def test_single_weak_patch_and_duplicate_lidar_returns_cannot_create_supported_region(self):
        memory = SemanticLandmarkMemory()
        self.capture(memory, points=scene()[2][:16])
        self.assertEqual(memory.retrieve('gas station', [0., 1.], .5, 200.), [])
        memory = SemanticLandmarkMemory()
        point = scene()[2][0]
        self.assertEqual(self.capture(memory, points=np.tile(point, (100, 1))), 0)
        self.assertEqual(memory.count, 0)

    def test_retrieval_does_not_modify_query_features_and_feedback_is_bounded(self):
        memory = SemanticLandmarkMemory(capacity=2)
        self.capture(memory)
        query = np.array([0., 2.], dtype=np.float32)
        first = memory.retrieve('gas station', query, .5, 200.)[0]
        np.testing.assert_array_equal(query, [0., 2.])
        memory.reject(first['id'], 200.)
        self.assertEqual(memory.retrieve('gas station', query, .5, 201.), [])
        self.assertTrue(memory.retrieve('gas station', query, .5, 231.))
        for n in range(20):
            memory.reject(str(n), 300.)
        self.assertLessEqual(len(memory.blocked), 2)

    def test_frame_change_and_nonfinite_features_are_rejected(self):
        memory = SemanticLandmarkMemory()
        self.capture(memory)
        with self.assertRaises(ValueError):
            memory.observe(*scene(), [100., 0., 0.], 0., 102*10**9, 102., 'other')
        features = scene()[0]
        features[0, 0, 0, 0] = np.nan
        with self.assertRaises(ValueError):
            self.capture(memory, stamp=102*10**9, position=(101., 0., 0.), now=102., features=features)


if __name__ == '__main__':
    unittest.main()
