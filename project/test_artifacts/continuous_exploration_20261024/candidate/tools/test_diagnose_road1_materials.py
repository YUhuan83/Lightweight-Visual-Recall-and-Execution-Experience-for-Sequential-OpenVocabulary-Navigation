import unittest

from diagnose_road1_materials import segment_distance_xy, resolve_reference, parse_json


class DiagnosticTests(unittest.TestCase):
    def test_segment_interior_ignores_z(self):
        distance, fraction, point = segment_distance_xy((5, 3, 999), (0, 0, 0), (10, 0, 0))
        self.assertEqual((distance, fraction, point), (3, .5, (5, 0)))

    def test_endpoints(self):
        self.assertEqual(segment_distance_xy((-3, 4), (0, 0), (10, 0))[0], 5)
        self.assertEqual(segment_distance_xy((13, 4), (0, 0), (10, 0))[0], 5)

    def test_degenerate_segment(self):
        self.assertEqual(segment_distance_xy((3, 4), (0, 0), (0, 0)), (5, 0, (0, 0)))

    def test_link_not_missing(self):
        result = resolve_reference('road.dds', 'levels/test/materials.json',
                                   {'levels/test/road.dds.link'})
        self.assertEqual(result['status'], 'link_entry_in_mod_target_unverified')

    def test_external_not_confirmed_missing(self):
        result = resolve_reference('/art/road.dds', 'levels/test/materials.json', set())
        self.assertEqual(result['status'], 'unresolved_in_archive_not_confirmed_missing')

    def test_ndjson_comments(self):
        self.assertEqual(list(parse_json('{"a": 1,}\n// comment\n{"b": 2}')),
                         [{'a': 1}, {'b': 2}])


if __name__ == '__main__':
    unittest.main()
