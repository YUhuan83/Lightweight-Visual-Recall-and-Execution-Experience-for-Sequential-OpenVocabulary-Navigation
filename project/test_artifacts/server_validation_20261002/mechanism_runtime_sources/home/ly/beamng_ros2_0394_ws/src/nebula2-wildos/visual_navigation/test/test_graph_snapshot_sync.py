import ast
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import Mock


class GraphSnapshotTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'visual_navigation/wildos/nav.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef)
                   and n.name == 'WildOS_Nav')
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
                    n.name in ('on_navgraph_snapshot', 'sensor_sync_callback')]
        self.now = 100.
        namespace = dict(deepcopy=deepcopy, wall_time=lambda: self.now)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), 'exec'), namespace)
        self.node = namespace['WildOS_Nav']()
        self.node.latest_navgraph = None
        self.node.latest_navgraph_at = 0.
        self.node.global_frame = 'spot1/odom'
        self.node.navigation_graph_max_age_sec = 5.
        self.node.listener_callback = Mock()

    def test_sensor_pair_can_use_fresh_graph_with_different_publication_time(self):
        graph = Mock()
        graph.header.frame_id = 'spot1/odom'
        graph.header.stamp.sec = 1
        self.node.on_navgraph_snapshot(graph)
        self.node.sensor_sync_callback(Mock(), Mock(), Mock())
        self.node.listener_callback.assert_called_once()
        self.assertIsNot(self.node.listener_callback.call_args.args[1], graph)

    def test_stale_graph_is_not_reused(self):
        graph = Mock()
        graph.header.frame_id = 'spot1/odom'
        self.node.on_navgraph_snapshot(graph)
        self.now += 6
        self.node.sensor_sync_callback(Mock())
        self.node.listener_callback.assert_not_called()

    def test_wrong_graph_frame_is_rejected(self):
        graph = Mock()
        graph.header.frame_id = 'other_map'
        self.node.on_navgraph_snapshot(graph)
        self.node.sensor_sync_callback(Mock())
        self.node.listener_callback.assert_not_called()
