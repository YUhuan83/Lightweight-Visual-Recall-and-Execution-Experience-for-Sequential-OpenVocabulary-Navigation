"""Check target isolation using the actual triangulator callbacks."""
import ast
import json
from pathlib import Path
import unittest
from unittest.mock import Mock
from types import SimpleNamespace as S
from std_msgs.msg import String


class TriangulationQueryTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ObjectMaskTriangulator')
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
                    n.name in ('on_query', 'listener_callback', 'stamp_ns', 'record_sync_timing', 'prune_views')]
        ns = {'String': String, 'json': json}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), 'exec'), ns)
        self.node = ns['ObjectMaskTriangulator']()
        self.node.views = [object()]
        self.node.bearing_views = []
        self.node.query_started_ns = 0
        self.node.last_observation_ns = 0
        self.node.observation_timeout = 5.
        self.node.get_clock = lambda: S(now=lambda: S(nanoseconds=100*10**9))
        self.node.prev_view_pos = object()
        self.node.triangulated_position = object()
        self.node.found_lidar_in_mask = True
        self.node.active_query = 'gas station'
        self.node.msg_buffer = Mock()
        self.node.get_logger = Mock()
        self.node.clbk_cntr = 0
        self.node.lidar_tf_frame = 'lidar'
        self.node.num_cameras = 1
        self.node.camera_mask_max_skew_ns = 50_000_000
        self.node.sync_slop_ns = 100_000_000
        self.node.timing_pub = Mock()

    @staticmethod
    def pair(query, mask_ns=100_000_000_000, camera_ns=None, lidar_ns=None):
        camera_ns = mask_ns if camera_ns is None else camera_ns
        lidar_ns = mask_ns if lidar_ns is None else lidar_ns
        mask = S(query=query,
                 header=S(stamp=S(sec=mask_ns // 10**9, nanosec=mask_ns % 10**9)),
                 cam_infos=[S(header=S(stamp=S(sec=camera_ns // 10**9,
                                                  nanosec=camera_ns % 10**9)))])
        lidar = S(header=S(frame_id='lidar',
                           stamp=S(sec=lidar_ns // 10**9, nanosec=lidar_ns % 10**9)))
        return mask, lidar

    def test_query_change_discards_old_target_hypotheses(self):
        self.node.on_query(String(data='trash can'))
        self.assertEqual(self.node.views, [])
        self.assertIsNone(self.node.triangulated_position)
        self.assertIsNone(self.node.prev_view_pos)
        self.assertFalse(self.node.found_lidar_in_mask)
        self.node.msg_buffer.clear.assert_called_once()

    def test_late_old_target_mask_is_rejected(self):
        self.node.on_query(String(data='trash can'))
        self.node.listener_callback(*self.pair('gas station'))
        self.node.msg_buffer.add_msg.assert_not_called()
        self.node.listener_callback(*self.pair('trash can'))
        self.node.msg_buffer.add_msg.assert_called_once()

    def test_camera_capture_must_be_close_to_mask_and_transform_time(self):
        mask, lidar = self.pair('gas station', camera_ns=100_050_000_000)
        self.node.listener_callback(mask, lidar)
        self.node.msg_buffer.add_msg.assert_called_once()

        self.node.msg_buffer.reset_mock()
        mask, lidar = self.pair('gas station', camera_ns=100_051_000_000)
        self.node.listener_callback(mask, lidar)
        self.node.msg_buffer.add_msg.assert_not_called()
        payload = json.loads(self.node.timing_pub.publish.call_args[0][0].data)
        self.assertEqual(payload['reason'], 'camera_mask_timestamp_mismatch')

    def test_mask_lidar_skew_uses_the_configured_slop(self):
        mask, lidar = self.pair('gas station', lidar_ns=100_101_000_000)
        self.node.listener_callback(mask, lidar)
        self.node.msg_buffer.add_msg.assert_not_called()
        payload = json.loads(self.node.timing_pub.publish.call_args[0][0].data)
        self.assertEqual(payload['reason'], 'mask_lidar_timestamp_mismatch')

    def test_repeated_query_preserves_current_target_views(self):
        before = list(self.node.views)
        self.node.on_query(String(data=' Gas Station '))
        self.assertEqual(self.node.views, before)
        self.node.msg_buffer.clear.assert_not_called()

    def test_old_rays_cannot_be_used_as_fresh_multiview_evidence(self):
        self.node.views = ['old', 'boundary', 'fresh']
        self.node.bearing_views = [{'stamp_ns': 90*10**9},
                                   {'stamp_ns': 95*10**9}, {'stamp_ns': 99*10**9}]
        self.node.prune_views(100*10**9)
        self.assertEqual(self.node.views, ['boundary', 'fresh'])
        self.assertEqual(len(self.node.bearing_views), 2)
        self.node.prune_views(105*10**9)
        self.assertEqual(self.node.views, [])
        self.assertIsNone(self.node.prev_view_pos)

    def test_future_or_unstamped_rays_do_not_enter_current_localization(self):
        self.node.views = ['unstamped', 'future']
        self.node.bearing_views = [{}, {'stamp_ns': 101*10**9}]
        self.node.prune_views(100*10**9)
        self.assertEqual(self.node.views, [])
