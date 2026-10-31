"""Exercise the real memory callbacks without loading the vision models."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visual_navigation.wildos.confirmed_viewpoint_cache import ConfirmedViewpointCache


class ViewpointCallbackTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'visual_navigation/wildos/nav.py'
        tree = ast.parse(source.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'WildOS_Nav')
        names = {'object_confirmation_callback', '_reuse_remembered_object_goal',
                 'remembered_candidate_feedback'}
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        module = ast.Module(body=[cls], type_ignores=[])
        self.now = 100.
        namespace = dict(np=np, json=json, PoseStamped=PoseStamped, String=String,
                         wall_time=lambda: self.now)
        exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), namespace)
        self.node = namespace['WildOS_Nav']()
        self.node.text_queries = ['tree']
        self.node.global_frame = 'spot1/odom'
        self.node.object_query_started_ns = 0
        self.node.latest_navgraph = None
        self.node.num_angular_bins = 16
        self.node.visual_memory_map = None
        self.node.semantic_landmarks = None
        self.node.passive_query_candidates = {}
        self.ros_now_ns = 100_000_000_000
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=self.ros_now_ns))
        self.node.object_goal_memory = ConfirmedViewpointCache()
        self.node.remembered_candidates_publisher = Mock()
        self.node.remembered_object_goal_publisher = Mock()
        self.node.publish_legacy_remembered_goal = False
        self.node.get_logger = Mock()

    def observe(self, x, *, stamp_ns=None, query='tree', frame='spot1/odom', confirmed=True):
        data = {
            'query': query,
            'frame': frame,
            'search_confirmed': confirmed,
            'observation_stamp_ns': self.ros_now_ns if stamp_ns is None else stamp_ns,
            'object_position': [99., 0., 0.],
            'radius': 0.5,
            'inliers': 3,
            'observations': 4,
            'observer_position': [float(x), 0., 0.],
            'observer_orientation': [0., 0., 0., 1.],
        }
        self.node.object_confirmation_callback(String(data=json.dumps(data)))

    def payload(self):
        self.node._reuse_remembered_object_goal('tree')
        return json.loads(self.node.remembered_candidates_publisher.publish.call_args.args[0].data)

    def test_confirmed_observations_publish_multiple_observer_positions(self):
        self.observe(2.)
        self.observe(8.)
        payload = self.payload()
        self.assertEqual([v['position'][0] for v in payload['views']], [2., 8.])
        self.node.remembered_object_goal_publisher.publish.assert_not_called()

    def test_stale_positive_detection_does_not_add_a_view(self):
        self.ros_now_ns += 6_000_000_000
        self.observe(2., stamp_ns=100_000_000_000)
        self.assertEqual(self.payload()['views'], [])

    def test_unconfirmed_or_mismatched_query_does_not_add_a_view(self):
        self.observe(2., confirmed=False)
        self.observe(8., query='house')
        self.observe(4., frame='map')
        self.assertEqual(self.payload()['views'], [])

    def test_unreachable_feedback_preserves_other_views_and_publishes_empty_set(self):
        self.observe(2.)
        self.observe(8.)
        views = self.payload()['views']
        for index, view in enumerate(views):
            self.node.remembered_candidate_feedback(String(data=json.dumps({
                'query': 'tree', 'id': view['id'], 'outcome': 'unreachable'})))
            self.assertEqual(len(self.payload()['views']), 1 - index)
        self.now += 31.
        self.assertEqual(len(self.payload()['views']), 2)
