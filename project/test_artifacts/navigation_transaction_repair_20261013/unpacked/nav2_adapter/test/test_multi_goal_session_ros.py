"""Same-process semantic session, with supplied paths and no simulated physics.

The vision confirmation/replay methods and bridge/GoalStandoff callbacks are
production code. Masks/localization and Nav2 paths are fixture inputs; this is
state-machine verification, not a perception or ROS navigation success claim.
"""
import ast
from concurrent.futures import Future
from copy import deepcopy
import json
import re
import sqlite3
import time
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from visual_navigation.wildos.viewpoint_memory import ViewpointMemory
from visual_navigation.wildos.visual_memory_map import VisualMemoryMap
from visual_navigation.wildos import visual_memory_map
from visual_navigation.wildos.semantic_landmark_memory import SemanticLandmarkMemory
from test_graph_path_bridge_routing import route

ROOT = Path(__file__).resolve().parents[2]
NAV = next(path for path in (
    ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py',
    Path(visual_memory_map.__file__).with_name('nav.py')) if path.is_file())


def memory_owner():
    tree = ast.parse(NAV.read_text(encoding='utf8'))
    methods = {'object_confirmation_callback', '_reuse_remembered_object_goal', '_query_memory_key',
               'search_query_callback', '_reuse_recent_scene_features', '_retrieve_passive_landmarks',
               '_set_object_search_competitors'}
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    namespace = dict(np=np, json=json, sqlite3=sqlite3, wall_time=time.time, String=String, re=re)
    utilities = ast.parse((NAV.parents[1]/'utils/object_search_utils.py').read_text(encoding='utf8'))
    dependencies = [n for n in utilities.body if
        (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_COLOR_WORDS' for t in n.targets))
        or (isinstance(n, ast.FunctionDef) and n.name == 'color_counterfactual_queries')]
    exec(compile(ast.Module(body=dependencies, type_ignores=[]), str(NAV), 'exec'), namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(NAV), 'exec'), namespace)
    return namespace[cls.name]()


class MultiGoalSessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100 * 10**9
        self.bridge, self.confirm = GraphPathBridge(), GoalStandoff()
        # This state-machine fixture supplies a co-located synthetic camera;
        # real mounted-rig clearance is covered by the geometry regressions.
        original_parameter = self.confirm.get_parameter
        overrides = {'standoff': 4., 'camera_offset': [0., 0., 0.],
                     'minimum_camera_target_range': .1}
        self.confirm.get_parameter = lambda name: (S(value=overrides[name])
            if name in overrides else original_parameter(name))
        self.vision = memory_owner()
        self.db = VisualMemoryMap(':memory:', 'offline-session')
        self.db.register_map('town', 'town')
        v = self.vision
        v.text_queries, v.global_frame, v.latest_navgraph = ['gas station'], 'spot1/odom', None
        v.object_goal_memory, v.visual_memory_map = ViewpointMemory(), self.db
        v.semantic_landmarks, v.passive_query_candidates = None, {}
        v.num_angular_bins, v.memory_map_id, v.object_query_started_ns = 16, 'town', self.now
        v.uuid_to_str = lambda u: u
        v.remembered_candidates_publisher = Mock()
        v.publish_legacy_remembered_goal = False
        v.search_target_cache = {'gas station': np.ones((1, 3))}
        v.object_search_competitors = ('stop sign', 'gas station sign', 'road sign', 'street lamp',
                                       'tree', 'building', 'sky')
        v.object_search_competitor_cache = {}
        v.object_search_competitor_feats = None
        v.object_search_minimum_competitive_cells = 0
        v.model = Mock()
        v.model.forward_on_text.return_value = np.ones((1, 3))
        v.last_scene_features, v.latest_odom_msg = None, None
        for name in ('frontier_uuid_to_scores', 'frontier_score_stats', 'frontier_uuid_to_scoring_distance',
                     'frontier_last_scored_at'):
            setattr(v, name, {})
        clock = S(now=lambda: S(nanoseconds=self.now, to_msg=lambda: S(sec=self.now//10**9, nanosec=self.now%10**9)))
        for node in (self.bridge, self.confirm, v):
            node.get_clock = lambda: clock
            node.get_logger = lambda: Mock()
        self.bridge.graph_reset_client = Mock()
        self.bridge.graph_reset_client.service_is_ready.return_value = True
        def clear(_request):
            f = Future()
            f.set_result(S(success=True))
            return f
        self.bridge.graph_reset_client.call_async.side_effect = clear
        self.bridge.follow_enabled = True
        self.bridge.follow_client = Mock()
        self.bridge.follow_client.wait_for_server.return_value = True
        self.bridge.follow_client.send_goal_async.side_effect = lambda goal, **_kwargs: Future()
        self.confirm.confirmation_pub.publish = Mock(side_effect=self.forward_confirmation)
        v.remembered_candidates_publisher.publish.side_effect = self.bridge.on_memory_candidates
        self.update_inputs(0.)

    def tearDown(self):
        self.bridge.destroy_node()
        self.confirm.destroy_node()
        self.db.close()

    def update_inputs(self, x):
        self.now += 100000000
        odom = Odometry()
        odom.header.frame_id = 'spot1/odom'
        odom.header.stamp = self.bridge.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = x
        self.bridge.on_odom(odom)
        self.confirm.on_odom(odom)
        grid = OccupancyGrid()
        grid.header = deepcopy(odom.header)
        grid.info.width = grid.info.height = 100
        grid.info.resolution = 1.
        grid.info.origin.position.x = grid.info.origin.position.y = -50.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 10000
        self.bridge.on_map(grid)
        self.bridge.on_local_map(deepcopy(grid))

    def forward_confirmation(self, msg):
        self.vision.object_confirmation_callback(msg)
        self.bridge.on_object_confirmation(msg)

    def query(self, label):
        self.now += 100000000
        message = String(data=label)
        self.confirm.on_query(message)
        self.bridge.on_query(message)
        self.vision.search_query_callback(message)

    def localize(self, label, object_x, observer_x):
        self.update_inputs(observer_x)
        data = dict(query=label, frame='spot1/odom', observation_stamp_ns=self.now,
            position=[object_x, 0., 2.], observer_position=[observer_x, 0., 0.],
            observer_orientation=[0., 0., 0., 1.], source='lidar', views=0,
            localization_valid=True,
            quality={'confidence': .9, 'points': 25, 'horizontal_fov': 1.4,
                     'association_validated': True})
        self.confirm.on_localization(String(data=json.dumps(data)))

    def test_unqueried_station_becomes_navigation_hint_after_house_query(self):
        class Features:
            def __init__(self, value):
                self.value = np.asarray(value, dtype=np.float32)
                self.shape = self.value.shape
            def detach(self): return self
            def float(self): return self
            def cpu(self): return self
            def numpy(self): return self.value
        v = self.vision
        v.text_queries = ['house']
        v.search_target_cache = {'house': Features([[1., 0.]]), 'gas station': Features([[0., 1.]])}
        v.mask_threshold = .5
        v.semantic_landmarks = SemanticLandmarkMemory()
        features = np.zeros((1, 2, 2, 4), dtype=np.float32)
        features[:, 0] = 1.
        features[0, :, :, 1:3] = np.array([0., 1.])[:, None, None]
        camera = dict(K=np.array([[100., 0., 60.], [0., 100., 40.], [0., 0., 1.]]),
                      R_wc=np.eye(3), t_wc=np.zeros(3), width=120, height=80)
        points = np.array([[x+dx, dy, 10.] for x in (-1.6, 1.2)
                           for dx in np.linspace(-.18, .18, 4) for dy in np.linspace(.1, .46, 4)])
        v.semantic_landmarks.observe(features, [camera], points, [0., 0., 0.], 0.,
                                     self.now, time.time(), 'spot1/odom')
        self.assertFalse(v.object_goal_memory.views)
        self.query('house')
        self.update_inputs(15.)
        self.query('gas station')
        self.assertTrue(self.bridge.memory_selecting)
        self.assertEqual(self.bridge.memory_active['evidence'], 'passive_semantic_landmarks')
        self.assertFalse(self.bridge.object_found)
        self.assertFalse(v.object_goal_memory.views)
        # A different facade inside the selected remembered region may
        # complete the query; an unrelated distant region must not do so.
        remembered_x = self.bridge.memory_active['object_position'][0]
        # This fixture bypasses action planning. Explicitly finish selection;
        # a transient candidate under comparison cannot accept arrival.
        self.bridge.memory_selecting = False
        for _ in range(5):
            self.localize('gas station', remembered_x + 2., remembered_x-2.)
        self.assertTrue(self.bridge.object_found)
        self.assertTrue(v.object_goal_memory.candidates('gas station', time.time()))

    def test_candidate_snapshot_from_previous_same_label_query_is_rejected(self):
        self.query('house')
        first_start = self.bridge.query_started_ns
        # Model a delayed same-label candidate from a prior transaction. The
        # bridge and producer each timestamp independently, so the stale
        # transaction must be outside the allowed clock scheduling skew.
        self.now += 2 * 10**9
        self.query('gas station')
        view = dict(id='passive:gas station:old', frame='spot1/odom',
                    position=[4., 0., 2.], heading=0.,
                    object_position=[10., 0., 2.], object_radius=.5,
                    evidence='passive_semantic_landmarks')
        old_snapshot = dict(query='gas station', views=[view],
                            issued_at_ns=self.now,
                            query_started_ns=first_start)
        self.bridge.on_memory_candidates(String(data=json.dumps(old_snapshot)))
        self.assertFalse(self.bridge.memory_selecting)
        self.assertIsNone(self.bridge.memory_goal)

    def test_valid_candidate_ahead_of_delayed_query_is_checked_against_new_transaction(self):
        self.query('house')
        self.now += 2 * 10**9
        self.update_inputs(0.)
        producer_query_start = self.now
        view = dict(id='passive:gas station:new', frame='spot1/odom',
                    position=[4., 0., 2.], heading=0., object_position=[10., 0., 2.],
                    object_radius=.5, evidence='passive_semantic_landmarks')
        snapshot = dict(query='gas station', views=[view],
                        issued_at_ns=self.now, query_started_ns=producer_query_start)
        # The producer's query callback can publish before the bridge's
        # subscription callback runs. Keep the complete transaction metadata
        # while buffering so it can be compared with the arriving query.
        self.bridge.on_memory_candidates(String(data=json.dumps(snapshot)))
        self.assertIn('gas station', self.bridge.memory_pending_payloads)
        message = String(data='gas station')
        self.confirm.on_query(message)
        self.bridge.on_query(message)
        self.assertTrue(self.bridge.memory_selecting)
        self.assertEqual(self.bridge.memory_active['id'], 'passive:gas station:new')

    def test_candidate_for_switching_query_survives_unrelated_out_of_order_candidate(self):
        self.query('house')
        gas_view = dict(id='passive:gas station:next', frame='spot1/odom',
                        position=[4., 0., 2.], heading=0.,
                        object_position=[10., 0., 2.], object_radius=.5,
                        evidence='passive_semantic_landmarks')
        self.now += 100_000_000
        gas_snapshot = dict(query='gas station', views=[gas_view],
                            issued_at_ns=self.now, query_started_ns=self.now)
        self.bridge.on_memory_candidates(String(data=json.dumps(gas_snapshot)))

        # A delayed snapshot from another query arrives later on the same
        # topic. It must not overwrite the candidate prepared for gas station.
        self.now += 100_000_000
        delayed_view = dict(id='passive:tree:old', frame='spot1/odom',
                            position=[8., 0., 2.], heading=0.)
        delayed_snapshot = dict(query='tree', views=[delayed_view],
                                issued_at_ns=self.now, query_started_ns=self.now)
        self.bridge.on_memory_candidates(String(data=json.dumps(delayed_snapshot)))

        self.now += 100_000_000
        message = String(data='gas station')
        self.confirm.on_query(message)
        self.bridge.on_query(message)
        self.assertTrue(self.bridge.memory_selecting)
        self.assertEqual(self.bridge.memory_active['id'], 'passive:gas station:next')

    def test_semantic_competitors_follow_the_active_query(self):
        v = self.vision
        v.search_target_cache.update({'traffic light': np.ones((1, 3)),
                                      'gas station': np.ones((1, 3))})
        self.query('traffic light')
        self.assertIsNotNone(v.object_search_competitor_feats)
        self.assertEqual(v.object_search_minimum_competitive_cells, 5)
        self.query('gas station')
        self.assertIsNone(v.object_search_competitor_feats)
        self.assertEqual(v.object_search_minimum_competitive_cells, 0)

    def test_station_house_station_reuses_same_memory_and_can_finish_at_a_new_pose(self):
        identities = tuple(map(id, (self.vision, self.db, self.vision.object_goal_memory)))
        self.query('gas station')
        for _ in range(5):
            self.localize('gas station', 10., 6.)
        self.assertTrue(self.bridge.object_found)
        self.assertTrue(self.vision.object_goal_memory.candidates('gas station', time.time()))
        self.query('house')
        self.assertFalse(self.bridge.object_found)
        for _ in range(5):
            self.localize('house', 30., 15.)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'confirmation_before_standoff')
        for _ in range(5):
            self.localize('house', 30., 20.)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'confirmation_before_standoff')
        for _ in range(5):
            self.localize('house', 30., 26.)
        self.assertTrue(self.bridge.object_found)
        # Put the synthetic vehicle at the recorded route's start pose before
        # checking same-process memory reuse below.
        self.update_inputs(20.)
        self.query('gas station')
        self.assertTrue(self.bridge.memory_selecting)
        self.assertEqual(self.bridge.memory_active['object_position'], [10., 0., 2.])
        path = route((20., 0.), (15., 0.), (8., 0.), (0., 0.))
        self.bridge.route = self.bridge.latest_route = path
        self.bridge.route_endpoint = (0., 0.)
        # Feed a supplied valid local prefix through the real memory handover.
        self.bridge.memory_full_check = False
        prefix = route((20., 0.), (15., 0.), (8., 0.))
        self.bridge.record_memory_path(prefix, [(20., 0.), (15., 0.), (8., 0.)], [0.]*3)
        self.assertFalse(self.bridge.memory_selecting)
        self.bridge.follow_client.send_goal_async.assert_called_once()
        self.assertIsNotNone(self.bridge.memory_goal)
        for _ in range(5):
            self.localize('gas station', 10., 14.)
        self.assertTrue(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'object_found')
        self.assertEqual(self.bridge.last_status['observer_position'][0], 14.)
        self.assertEqual(tuple(map(id, (self.vision, self.db, self.vision.object_goal_memory))), identities)
        self.assertGreaterEqual(len(self.db.query_objects('town', now=self.now*1e-9)), 2)

    def test_previous_a_confirmation_cannot_complete_a_new_a_after_b(self):
        self.query('gas station')
        for _ in range(5):
            self.localize('gas station', 10., 0.)
        old = self.confirm.confirmation_pub.publish.call_args.args[0]
        self.query('house')
        self.query('gas station')
        self.forward_confirmation(old)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(len(self.confirm.observations), 0)
