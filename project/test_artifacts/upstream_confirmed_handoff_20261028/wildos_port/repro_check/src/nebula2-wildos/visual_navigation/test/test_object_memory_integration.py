"""Run the actual confirmed-object callback without importing the vision model."""
import ast
import json
import sqlite3
import time
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np

from visual_navigation.wildos.confirmed_viewpoint_cache import ConfirmedViewpointCache
from visual_navigation.wildos.graph_observation_store import GraphObservationStore

SRC = Path(__file__).parents[1]/'visual_navigation/wildos/nav.py'
tree = ast.parse(SRC.read_text(encoding='utf-8'))
method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'object_confirmation_callback')
namespace = dict(np=np, json=json, sqlite3=sqlite3, wall_time=time.time)
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SRC), 'exec'), namespace)


class ObjectMemoryIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.database = GraphObservationStore(':memory:', 'offline-test')
        self.database.register_map('town', 'town')
        self.now = 101*10**9
        node = S(pose=S(position=S(x=0., y=0., z=0.)), uuid='node-a')
        self.owner = S(text_queries=['gas station'], global_frame='spot1/odom', latest_navgraph=S(
            header=S(frame_id='spot1/odom', stamp=S(sec=101, nanosec=0)), nodes=[node]), visual_memory_map=self.database,
            object_goal_memory=ConfirmedViewpointCache(), num_angular_bins=16, memory_map_id='town',
            object_query_started_ns=100*10**9, uuid_to_str=lambda u: u,
            get_clock=lambda: S(now=lambda: S(nanoseconds=self.now)), get_logger=lambda: Mock())
        self.callback = namespace['object_confirmation_callback']

    def tearDown(self):
        self.database.close()

    def confirm(self, **changes):
        data = dict(query='gas station', frame='spot1/odom', observation_stamp_ns=self.now,
            object_position=[20., 0., 2.], observer_position=[0., 0., 0.],
            observer_orientation=[0., 0., 0., 1.], radius=.5, inliers=4, observations=5,
            search_confirmed=True, standoff=[15., 0.])
        data.update(changes)
        self.callback(self.owner, S(data=json.dumps(data)))

    def test_confirmed_region_populates_view_and_semantic_map_with_node_anchor(self):
        self.confirm()
        views = self.owner.object_goal_memory.candidates('gas station', time.time())
        self.assertEqual(len(views), 1)
        self.assertEqual(views[0]['object_position'], [20., 0., 2.])
        self.assertEqual(views[0]['position'], [0., 0., 0.])
        self.assertEqual(views[0]['anchor_node_uuid'], 'node-a')
        objects = self.database.query_objects('town', label='gas station', now=101.)
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0].object_id, views[0]['object_id'])

    def test_two_independent_views_enable_heading_prior(self):
        self.confirm()
        self.now += 10**9
        self.owner.latest_navgraph.nodes[0].uuid = 'node-b'
        self.owner.latest_navgraph.nodes[0].pose.position.x = 5.
        self.confirm(observer_position=[5., 0., 0.])
        objects = self.database.query_objects('town', label='gas station', now=102.)
        self.assertEqual(objects[0].status, 'CONFIRMED')
        bonus = self.database.confirmed_heading_bonus('town', 'gas station', 'node-b')
        self.assertGreater(float(bonus.max()), 0.)

    def test_same_label_other_instance_keeps_a_separate_view(self):
        self.confirm()
        self.now += 10**9
        self.confirm(object_position=[200., 0., 2.])
        views = self.owner.object_goal_memory.candidates('gas station', time.time())
        self.assertEqual(len(views), 2)
        self.assertNotEqual(views[0]['object_id'], views[1]['object_id'])

    def test_late_previous_generation_and_nonfinite_region_are_ignored(self):
        self.confirm(observation_stamp_ns=99*10**9)
        self.confirm(query='house')
        self.confirm(object_position=[float('nan'), 0., 0.])
        self.confirm(observer_orientation=[0., 0., 0., 0.])
        self.assertFalse(self.owner.object_goal_memory.views)

