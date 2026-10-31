"""Replay the recorded v16 blocked-start -> clear-costmap transition through v17."""
import copy
import glob
import json
import math
import sqlite3
import sys
import time
from pathlib import Path
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge, xy, yaw


TRIAL = Path('/home/ly/execution_continuity_review_20261022/v16_explore_during_reobserve/full/trial_20261023_184334')
GOAL = next(TRIAL.glob('sessions/*/goal_001'))
DATABASE = Path(glob.glob(str(GOAL / 'interfaces/*.db3'))[0])
db = sqlite3.connect(f'file:{DATABASE}?mode=ro', uri=True)


def topic_id(name):
    row = db.execute('select id from topics where name=?', (name,)).fetchone()
    return row[0] if row else None


def read(name, message_type):
    identifier = topic_id(name)
    if identifier is None:
        return []
    return [
        (bag_ns * 1e-9, deserialize_message(blob, message_type))
        for bag_ns, blob in db.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',
            (identifier,))
    ]


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def set_stamp(message, value):
    ns = int(value * 1e9)
    message.header.stamp.sec = ns // 1_000_000_000
    message.header.stamp.nanosec = ns % 1_000_000_000


def main():
    rclpy.init()
    node = GraphPathBridge()
    try:
        # Match the actual v16 field runtime recorded in the status event.
        node.carrier_model = 'ackermann'
        node.minimum_turning_radius = 8.0
        plan_rows = read('/wildos/plan_status', String)
        blocked = None
        for bag_t, message in plan_rows:
            try:
                data = json.loads(message.data)
            except (TypeError, ValueError):
                continue
            if data.get('status') == 'start_footprint_blocked':
                blocked = (data.get('stamp_ns', int(bag_t * 1e9)) * 1e-9, data)
                break
        if blocked is None:
            raise RuntimeError('v16 closed bag has no start_footprint_blocked event')
        event_t, status = blocked
        odoms = read('/spot1/odom', Odometry)
        global_maps = read('/global_costmap/costmap', OccupancyGrid)
        local_maps = read('/local_costmap/costmap', OccupancyGrid)
        if not odoms or not global_maps or not local_maps:
            raise RuntimeError('missing recorded odometry or costmap messages')

        from wildos_nav2_adapter.graph_route import footprint_blockers
        selected = None
        for global_bag_t, global_grid in global_maps:
            global_t = stamp(global_grid)
            if not event_t + .2 <= global_t <= event_t + 8.:
                continue
            odom_t, odom = min(odoms, key=lambda row: abs(stamp(row[1])-global_t))
            if abs(stamp(odom)-global_t) > .75:
                continue
            position = xy(odom.pose.pose)
            heading = yaw(odom.pose.pose)
            if footprint_blockers(global_grid, position, heading):
                continue
            near_local = [(bag_t, grid) for bag_t, grid in local_maps
                          if abs(stamp(grid)-global_t) <= .75]
            if not near_local:
                continue
            local_bag_t, local_grid = min(near_local, key=lambda row: abs(stamp(row[1])-global_t))
            if footprint_blockers(local_grid, position, heading):
                continue
            selected = (global_grid, local_grid, odom, global_t, position, heading)
            break
        if selected is None:
            raise RuntimeError('no synchronized post-block global/local map pair clears the measured footprint')

        global_grid, local_grid, odom, map_t, position, heading = selected
        now_ns = node.get_clock().now().nanoseconds
        fresh_stamp = (now_ns - 50_000_000) * 1e-9
        set_stamp(odom, fresh_stamp)
        set_stamp(global_grid, fresh_stamp)
        set_stamp(local_grid, fresh_stamp)
        if not status.get('goal') or len(status['goal']) != 2:
            raise RuntimeError(f'blocked status has no goal: {status}')
        goal = tuple(map(float, status['goal']))

        node.follow_enabled = True
        node.plan_client = Mock()
        node.plan_client.wait_for_server.return_value = True
        node.follow_client = Mock()
        node.follow_client.wait_for_server.return_value = True
        node.input_freshness_issue = Mock(return_value=None)
        node.local_freshness_issue = Mock(return_value=None)
        node.execution_transport_issue = Mock(return_value=None)
        node.status_pub = Mock()
        node.start_segment = Mock()
        node.query = status.get('query')
        node.query_generation = int(status.get('query_generation', 0))
        node.route = RosPath()
        node.route.header.frame_id = 'spot1/odom'
        node.route.header.stamp = node.get_clock().now().to_msg()
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'spot1/odom'
        start_pose.pose = copy.deepcopy(odom.pose.pose)
        end_pose = PoseStamped()
        end_pose.header.frame_id = 'spot1/odom'
        end_pose.pose.position.x, end_pose.pose.position.y = goal
        end_pose.pose.orientation = copy.deepcopy(odom.pose.pose.orientation)
        node.route.poses = [start_pose, end_pose]
        node.route_endpoint = goal
        node.blocked_endpoint = goal
        node.blocked_route = [position, goal]
        node.blocked_grid_hash = hash(tuple(global_grid.data))
        node.blocked_until = time.monotonic() + 20.
        node.last_status = dict(status, navigation_epoch=node.epoch,
                                query_generation=node.query_generation)

        node.on_odom(odom)
        node.on_local_map(local_grid)
        before = {
            'event_elapsed_sec': map_t - event_t,
            'recorded_global_receipt_delta_sec': global_bag_t - event_t,
            'recorded_local_receipt_delta_sec': local_bag_t - event_t,
            'position': list(position), 'heading': heading,
            'global_blockers': footprint_blockers(global_grid, position, heading),
            'local_blockers': footprint_blockers(local_grid, position, heading),
            'goal': list(goal), 'backoff_remaining_sec': 20.0,
        }
        node.on_map(global_grid)
        after = {
            'start_segment_calls': node.start_segment.call_count,
            'status': node.last_status.get('status'),
            'trigger': node.last_status.get('trigger'),
            'blocked_endpoint': node.blocked_endpoint,
            'blocked_until': node.blocked_until,
        }
        result = {'source_trial': str(TRIAL), 'source_event': status,
                  'recorded_clear_map': before, 'v17_callback_result': after,
                  'passed': after['start_segment_calls'] == 1
                  and after['status'] == 'blocked_start_cleared'
                  and after['trigger'] == 'fresh_costmap_footprint_clear'}
        Path('/home/ly/execution_continuity_review_20261022/v17_costmap_unblock_retry/'
             'v16_blocked_start_replay.json').write_text(
                 json.dumps(result, indent=2), encoding='utf8')
        print(json.dumps(result, indent=2))
        if not result['passed']:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        db.close()


if __name__ == '__main__':
    main()
