import unittest

from visual_navigation.wildos.relation_query import SemanticObject, resolve_relation


class RelationQueryTests(unittest.TestCase):
    def setUp(self):
        self.objects = [
            SemanticObject("h1", "house", (-16.0, -59.0, 0.0)),
            SemanticObject("h2", "house", (16.0, -59.0, 0.0)),
            SemanticObject("h3", "house", (48.0, -59.0, 0.0)),
            SemanticObject("r1", "house", (-16.0, -101.0, 0.0)),
        ]

    def test_left_third_house(self):
        result = resolve_relation("左侧第三个房子", self.objects, (-40.0, -80.0, 0.0), (1.0, 0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertEqual(result.object_id, "h3")

    def test_right_first_house(self):
        result = resolve_relation("right first house", self.objects, (-40.0, -80.0, 0.0), (1.0, 0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertEqual(result.object_id, "r1")

    def test_missing_relation_returns_none(self):
        self.assertIsNone(resolve_relation("find a house", self.objects, (-40.0, -80.0, 0.0), (1.0, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
