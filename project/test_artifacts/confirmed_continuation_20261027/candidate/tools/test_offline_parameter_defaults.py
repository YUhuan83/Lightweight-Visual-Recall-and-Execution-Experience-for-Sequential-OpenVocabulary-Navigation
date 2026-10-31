"""Static ROS default inspection must resolve known literals and reject unknowns."""
import ast
import unittest
from offline_navigation_check import literal_parameter_defaults


class ParameterDefaultsTests(unittest.TestCase):
    def test_literal_and_explicit_constant_are_resolved(self):
        tree = ast.parse("self.declare_parameter('frame', FRAME)\nself.declare_parameter('enable', False)")
        self.assertEqual(literal_parameter_defaults(tree, {'FRAME': 'base'}),
                         {'frame': 'base', 'enable': False})

    def test_unresolved_constant_is_an_error(self):
        with self.assertRaisesRegex(ValueError, 'Unresolved parameter default: UNKNOWN'):
            literal_parameter_defaults(ast.parse("self.declare_parameter('frame', UNKNOWN)"), {})

    def test_expressions_cannot_execute_or_be_silently_skipped(self):
        with self.assertRaises(ValueError):
            literal_parameter_defaults(ast.parse("self.declare_parameter('frame', fn())"), {})


if __name__ == '__main__':
    unittest.main()
