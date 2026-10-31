"""Recheck the exact accepted Town memory prefix at selection/execution handoff."""
from copy import deepcopy
import json
import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as Route
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from test_graph_path_bridge_routing import configure_ackermann

ROOT = Path(__file__).resolve().parents[2]


class RecordedMemoryHandoffTests(unittest.TestCase):
    def setUp(self):
        self.node = configure_ackermann(GraphPathBridge())
        self.node.path_pub = Mock()
        self.node.waypoint_pub = Mock()
        self.node.status_pub = Mock()
        self.node.memory_selection_pub = Mock()
        self.node.query = 'gas station'
        self.fixture = np.load(ROOT/'test_artifacts/server_validation_20261002/accepted_memory_prefix_snapshot.npz')
        self.report = json.loads((ROOT/'test_artifacts/server_validation_20261002/prefix_replay.json').read_text())['accepted_prefixes'][0]
        self.node.odom = Odometry()
        self.node.odom.header.frame_id = 'spot1/odom'
        start = self.report['recorded']['start']
        self.node.odom.pose.pose.position.x, self.node.odom.pose.pose.position.y = start
        heading=self.report['recorded']['heading']
        self.node.odom.pose.pose.orientation.z=math.sin(heading/2)
        self.node.odom.pose.pose.orientation.w=math.cos(heading/2)
        self.node.odom_received_at = time.monotonic()
        self.path = self.route(self.fixture['points'], self.fixture['headings'])
        self.graph = self.route(self.fixture['route'])
        view = dict(id='recorded', frame='spot1/odom', heading=2.967,
                    position=[736.3732635771133, -12.646835812921765, 52.09688083549498])
        self.node.memory_matches = [dict(view=view, path=self.path, route=self.graph,
            heading=view['heading'], route_complete=False, cost=54., complete=False,
            target=tuple(self.fixture['points'][-1]))]

    def tearDown(self):
        self.fixture.close()
        self.node.destroy_node()

    def route(self, points, headings=None):
        path = Route()
        path.header.frame_id = 'spot1/odom'
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = 'spot1/odom'
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            heading = float(headings[index]) if headings is not None else 0.
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            path.poses.append(pose)
        return path

    def map(self, held=False):
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/odom'
        data = self.fixture['held_grid' if held else 'grid']
        grid.info.width, grid.info.height = data.shape[1], data.shape[0]
        grid.info.resolution = float(self.fixture['resolution'])
        origin = self.fixture['held_origin' if held else 'origin']
        grid.info.origin.position.x, grid.info.origin.position.y = map(float, origin)
        grid.info.origin.orientation.w = 1.
        grid.data = data.ravel().tolist()
        self.node.grid = grid
        self.node.grid_received_at = time.monotonic()

    def test_recorded_free_map_hands_selected_prefix_to_execution(self):
        self.map()
        self.node.finish_memory_selection()
        self.node.path_pub.publish.assert_called_once()
        sent = self.node.path_pub.publish.call_args.args[0]
        self.assertEqual(len(sent.poses), len(self.fixture['points']))
        self.assertEqual(self.node.last_status['status'], 'planned')
        self.assertFalse(self.node.route_complete)

    def test_later_blocked_map_rejects_same_prefix_without_erasing_object_memory(self):
        self.map(held=True)
        self.node.finish_memory_selection()
        self.node.path_pub.publish.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'path_changed_before_execution')
        self.assertEqual(self.node.last_status['reason'], 'footprint_collision')
        self.assertIsNotNone(self.node.memory_goal)


if __name__ == '__main__':
    unittest.main()

