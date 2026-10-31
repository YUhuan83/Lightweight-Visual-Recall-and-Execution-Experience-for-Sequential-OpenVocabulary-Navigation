"""Exercise the real memory callbacks without loading the vision models."""
import ast
import json
from pathlib import Path
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visual_navigation.wildos.remembered_goal import observed_viewpoint_pose
from visual_navigation.wildos.viewpoint_memory import ViewpointMemory


class ViewpointCallbackTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'visual_navigation/wildos/nav.py'
        tree = ast.parse(source.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'WildOS_Nav')
        names = {'object_search_waypoint_callback', '_reuse_remembered_object_goal',
                 'remembered_candidate_feedback'}
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        module = ast.Module(body=[cls], type_ignores=[])
        self.now = 100.
        namespace = dict(np=np, json=json, PoseStamped=PoseStamped, String=String,
                         observed_viewpoint_pose=observed_viewpoint_pose, wall_time=lambda: self.now)
        exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), namespace)
        self.node = namespace['WildOS_Nav']()
        self.node.text_queries = ['tree']
        self.node.last_positive_mask_query = 'tree'
        self.node.last_positive_mask_at = self.now
        self.node.object_search_memory_capture_ttl = 5.
        self.node.object_goal_memory = ViewpointMemory()
        self.node.remembered_candidates_publisher = Mock()
        self.node.remembered_object_goal_publisher = Mock()
        self.node.publish_legacy_remembered_goal = False
        self.node.get_logger = Mock()

    def observe(self, x):
        odom = Odometry()
        odom.header.frame_id = 'spot1/odom'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.orientation.w = 1.
        self.node.last_positive_mask_odom = odom
        goal = PoseStamped()
        goal.header.frame_id = 'spot1/odom'
        goal.pose.position.x = 99.  # Object position is not the driving destination.
        self.node.object_search_waypoint_callback(goal)

    def payload(self):
        self.node._reuse_remembered_object_goal('tree')
        return json.loads(self.node.remembered_candidates_publisher.publish.call_args.args[0].data)

    def test_real_callbacks_publish_multiple_observer_positions(self):
        self.observe(2.)
        self.observe(8.)
        payload = self.payload()
        self.assertEqual([v['position'][0] for v in payload['views']], [2., 8.])
        self.node.remembered_object_goal_publisher.publish.assert_not_called()

    def test_stale_positive_detection_does_not_add_a_view(self):
        self.now += 6.
        self.observe(2.)
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
