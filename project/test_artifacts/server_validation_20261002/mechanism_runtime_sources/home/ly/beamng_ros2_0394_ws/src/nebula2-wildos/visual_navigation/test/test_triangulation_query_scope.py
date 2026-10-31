"""Check target isolation using the actual triangulator callbacks."""
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock
from std_msgs.msg import String


class TriangulationQueryTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        tree = ast.parse(source.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ObjectMaskTriangulator')
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
                    n.name in ('on_query', 'listener_callback')]
        ns = {'String': String}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), 'exec'), ns)
        self.node = ns['ObjectMaskTriangulator']()
        self.node.views = [object()]
        self.node.prev_view_pos = object()
        self.node.triangulated_position = object()
        self.node.found_lidar_in_mask = True
        self.node.active_query = 'gas station'
        self.node.msg_buffer = Mock()
        self.node.get_logger = Mock()
        self.node.clbk_cntr = 0
        self.node.lidar_tf_frame = 'lidar'

    def test_query_change_discards_old_target_hypotheses(self):
        self.node.on_query(String(data='trash can'))
        self.assertEqual(self.node.views, [])
        self.assertIsNone(self.node.triangulated_position)
        self.assertIsNone(self.node.prev_view_pos)
        self.assertFalse(self.node.found_lidar_in_mask)
        self.node.msg_buffer.clear.assert_called_once()

    def test_late_old_target_mask_is_rejected(self):
        self.node.on_query(String(data='trash can'))
        self.node.listener_callback(Mock(query='gas station'), Mock())
        self.node.msg_buffer.add_msg.assert_not_called()
        lidar = Mock()
        lidar.header.frame_id = 'lidar'
        self.node.listener_callback(Mock(query='trash can'), lidar)
        self.node.msg_buffer.add_msg.assert_called_once()

    def test_repeated_query_preserves_current_target_views(self):
        before = list(self.node.views)
        self.node.on_query(String(data=' Gas Station '))
        self.assertEqual(self.node.views, before)
        self.node.msg_buffer.clear.assert_not_called()
