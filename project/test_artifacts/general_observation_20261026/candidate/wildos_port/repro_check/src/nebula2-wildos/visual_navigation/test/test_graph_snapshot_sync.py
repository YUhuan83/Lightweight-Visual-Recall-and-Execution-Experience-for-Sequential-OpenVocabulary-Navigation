import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np


class GraphSnapshotTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'visual_navigation/wildos/nav.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef)
                   and n.name == 'WildOS_Nav')
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
                    n.name in ('on_navgraph_snapshot', 'sensor_sync_callback', 'do_processing', 'publish_execution_anchors',
                               'update_navgraph_with_scores')]
        self.now = 100.
        namespace = dict(deepcopy=deepcopy, wall_time=lambda: self.now, np=np,
                         NavigationGraph=lambda: S(nodes=[],edges=[],trav_classes=[],robot_profiles=[]))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), 'exec'), namespace)
        self.node = namespace['WildOS_Nav']()
        self.node.latest_navgraph = None
        self.node.latest_navgraph_at = 0.
        self.node.global_frame = 'spot1/odom'
        self.node.navigation_graph_max_age_sec = 5.
        self.node.listener_callback = Mock()

    def test_scoring_retains_exact_mapper_stamp_for_topology_overlay(self):
        graph = S(header=S(frame_id='spot1/odom', stamp=123), trav_classes=['default'], nodes=[])
        self.node.num_cameras = 1
        self.node.traversability_class = 'default'
        self.node.remove_old_frontiers = Mock(return_value=[])
        scored, _, _ = self.node.update_navgraph_with_scores(graph, [[]], [], np.zeros(3))
        self.assertEqual(scored.header.stamp, 123)

    def test_sensor_pair_can_use_fresh_graph_with_different_publication_time(self):
        graph = Mock()
        graph.header.frame_id = 'spot1/odom'
        graph.header.stamp.sec = 1
        self.node.on_navgraph_snapshot(graph)
        self.node.sensor_sync_callback(Mock(), Mock(), Mock())
        self.node.listener_callback.assert_called_once()
        self.assertIs(self.node.listener_callback.call_args.args[1], graph)

    def test_selected_frame_uses_its_snapshot_and_scoring_cannot_mutate_received_graph(self):
        graph = S(header=S(frame_id='spot1/odom'), nodes=[S(score=.2)])
        self.node.on_navgraph_snapshot(graph)
        self.node.sensor_sync_callback(Mock(), Mock(), Mock())
        selected = self.node.listener_callback.call_args.args[1]
        newer = S(header=S(frame_id='spot1/odom'), nodes=[S(score=.8)])
        self.node.on_navgraph_snapshot(newer)
        self.node.num_cameras = 1
        self.node.using_compressed_imgs = self.node.cam_inverted = False
        self.node.br = S(imgmsg_to_cv2=Mock(return_value=np.zeros((2, 2, 3),dtype=np.uint8)))
        self.node.fetch_cam_intrinsics_extrinsics = Mock(return_value={})
        self.node.get_logger = Mock(return_value=Mock())
        def score_selected(**kwargs):
            snapshot = kwargs['navgraph']
            self.assertEqual(snapshot.nodes[0].score,.2)
            snapshot.nodes[0].score = .5
            raise RuntimeError('Stop before model inference in this callback test')
        self.node.geofrontier_to_image = S(extract_geofrontiers=Mock(side_effect=score_selected))
        self.node.do_processing({'odom':Mock(),'navgraph':selected,'cam_msgs':[Mock(),Mock()]},
                                {'world_from_cam0':Mock()})
        self.assertEqual(graph.nodes[0].score,.2)
        self.assertEqual(newer.nodes[0].score,.8)

    def test_stale_graph_is_not_reused(self):
        graph = Mock()
        graph.header.frame_id = 'spot1/odom'
        self.node.on_navgraph_snapshot(graph)
        self.now += 6
        self.node.sensor_sync_callback(Mock())
        self.node.listener_callback.assert_not_called()

    def test_execution_anchors_preserve_ids_and_poses_without_graph_payloads(self):
        class UnusedEdges(list):
            def __iter__(self):
                raise AssertionError('Execution anchors must not traverse unused graph edges')
        graph = S(header=S(frame_id='spot1/odom',stamp=123),current_node_idx=1,
                  nodes=[S(uuid=i,pose=S(x=float(i)),trav_properties=[object()],
                           heading_candidates=[object()]) for i in (1,2)],edges=UnusedEdges())
        self.node.execution_anchor_pub = Mock()
        self.node.publish_execution_anchors(graph)
        anchors = self.node.execution_anchor_pub.publish.call_args.args[0]
        self.assertEqual([(n.uuid,n.pose.x) for n in anchors.nodes],[(1,1.),(2,2.)])
        self.assertEqual(anchors.header.stamp,123)
        self.assertEqual(anchors.current_node_idx,1)
        self.assertEqual(anchors.edges,[])
        self.assertTrue(all(not hasattr(n,'heading_candidates') for n in anchors.nodes))
        self.assertEqual(len(graph.nodes[0].trav_properties),1)

    def test_wrong_graph_frame_is_rejected(self):
        graph = Mock()
        graph.header.frame_id = 'other_map'
        self.node.on_navgraph_snapshot(graph)
        self.node.sensor_sync_callback(Mock())
        self.node.listener_callback.assert_not_called()
