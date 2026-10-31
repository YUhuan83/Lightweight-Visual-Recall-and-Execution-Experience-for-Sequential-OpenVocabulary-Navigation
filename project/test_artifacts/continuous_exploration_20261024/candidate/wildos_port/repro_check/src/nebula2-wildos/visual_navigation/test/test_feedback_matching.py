import unittest

from visual_navigation.wildos.feedback_matching import select_feedback_sample


class FeedbackMatchingTests(unittest.TestCase):
    def test_matches_latest_preceding_sample_from_same_node(self):
        samples = [
            {"node_uuid": "other", "ros_stamp": 10.0},
            {"node_uuid": "n1", "ros_stamp": 8.0},
            {"node_uuid": "n1", "ros_stamp": 9.9},
            {"node_uuid": "n1", "ros_stamp": 11.0},
        ]
        self.assertEqual(select_feedback_sample(samples, 10.0, "n1", 5.0), 2)
        self.assertIsNone(select_feedback_sample(samples, 10.0, "n2", 5.0))
        self.assertIsNone(select_feedback_sample(samples, 20.0, "n1", 5.0))


if __name__ == "__main__":
    unittest.main()
