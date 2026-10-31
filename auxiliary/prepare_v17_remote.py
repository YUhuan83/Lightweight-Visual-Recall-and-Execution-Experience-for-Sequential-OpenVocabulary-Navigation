"""Prepare an isolated v17 server candidate from frozen v16 + one retry fix."""
from pathlib import Path
import hashlib
import json
import shutil

BASE = Path('/home/ly/execution_continuity_review_20261022')
SOURCE = BASE / 'v16_explore_during_reobserve'
TARGET = BASE / 'v17_costmap_unblock_retry'
OLD = 'execution_continuity_review_20261022/v16_explore_during_reobserve'
NEW = 'execution_continuity_review_20261022/v17_costmap_unblock_retry'


def replace_once(path, old, new):
    content = path.read_text(encoding='utf8')
    count = content.count(old)
    if count != 1:
        raise RuntimeError(f'{path}: expected one anchor match, found {count}')
    path.write_text(content.replace(old, new, 1), encoding='utf8')


def main():
    if not SOURCE.is_dir() or TARGET.exists():
        raise SystemExit(f'refusing source/target state: source={SOURCE.is_dir()} target_exists={TARGET.exists()}')
    TARGET.mkdir(parents=True)
    shutil.copytree(SOURCE / 'candidate', TARGET / 'candidate')
    full = TARGET / 'full'
    full.mkdir()
    for item in (SOURCE / 'full').iterdir():
        if item.is_file() and (item.suffix == '.py' or item.name in ('config.json', 'frozen_source_manifest.json')):
            shutil.copy2(item, full / item.name)
    (full / 'candidate').symlink_to('../candidate')
    for name in ('launch_trial.py', 'run_full.sh', 'build.sh', 'real_ros_review.py'):
        shutil.copy2(SOURCE / name, TARGET / name)
    for path in [TARGET / 'launch_trial.py', TARGET / 'run_full.sh', TARGET / 'build.sh',
                 TARGET / 'real_ros_review.py', *full.glob('*.py'), full / 'config.json']:
        content = path.read_text(encoding='utf8')
        path.write_text(content.replace(OLD, NEW), encoding='utf8')

    bridge = TARGET / 'candidate/nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
    tests = TARGET / 'candidate/nav2_adapter/test/test_graph_path_bridge_routing.py'
    replace_once(bridge,
'''            self.try_candidate()

    def input_freshness_issue(self):
''',
'''            self.try_candidate()
        self.retry_cleared_start_footprint()

    def retry_cleared_start_footprint(self):
        """Resume the same route when a fresh map clears a blocked vehicle pose.

        A blocked start normally gets a 20 s backoff. Costmaps can clear the
        vehicle footprint much sooner, so wait for fresh, mutually consistent
        inputs and let start_segment repeat its complete planning/execution
        checks instead of waiting for a timer or another GraphNav route.
        """
        status = self.last_status
        if (status.get('status') != 'start_footprint_blocked'
                or status.get('navigation_epoch') != self.epoch
                or status.get('query_generation') != self.query_generation
                or self.blocked_endpoint is None or self.route is None
                or not self.route.poses or self.route_endpoint is None
                or distance(self.route_endpoint, self.blocked_endpoint) > .5):
            return False
        if (not self.follow_enabled or self.query_reset_pending
                or self.instance_release_pending is not None
                or self.follow_quarantine or self.follow_release is not None
                or self.follow_request is not None or self.follow_active is not None
                or self.follow_hold or self.pending or self.following
                or self.recovering or self.replacement_pending or self.extending_path
                or self.frontier_choice is not None
                or self.confirmed_approach_failure is not None
                or self.execution_recovery_owns_motion()):
            return False
        if self.input_freshness_issue() is not None or self.local_freshness_issue() is not None:
            return False
        if self.execution_transport_issue() is not None:
            return False
        if (self.plan_client is None or self.follow_client is None
                or not self.plan_client.wait_for_server(timeout_sec=0.0)
                or not self.follow_client.wait_for_server(timeout_sec=0.0)):
            return False

        start = xy(self.odom.pose.pose)
        heading = yaw(self.odom.pose.pose)
        if (self.pose_blockers(self.grid, start, heading)
                or self.pose_blockers(self.local_grid, start, heading)):
            return False

        reported_goal = status.get('goal')
        current_goal = (self.expected or self.exploration_destination()
                        or xy(self.route.poses[-1].pose))
        if (not isinstance(reported_goal, (list, tuple)) or len(reported_goal) != 2
                or distance(tuple(reported_goal), current_goal) > .25):
            return False

        goal = tuple(current_goal)
        self.blocked_endpoint = self.blocked_route = self.blocked_grid_hash = None
        self.blocked_until = 0.0
        self.last_plan_start = self.last_plan_endpoint = None
        self.status('blocked_start_cleared', goal=list(goal),
                    trigger='fresh_costmap_footprint_clear')
        self.start_segment()
        return True

    def input_freshness_issue(self):
''')
    replace_once(bridge,
'''            self.status("start_footprint_blocked", goal=list(endpoint),
                        blocked_samples=blocked)
''',
'''            self.status("start_footprint_blocked", goal=list(endpoint),
                        blocked_samples=blocked, navigation_epoch=self.epoch)
''')
    test_anchor = '''        self.assertFalse(self.node.candidate_waiting_for_map)
        self.assertTrue(self.node.pending)

    def test_remembered_goal_routes_through_graph_and_ignores_other_waypoints(self):
'''
    test_insert = '''        self.assertFalse(self.node.candidate_waiting_for_map)
        self.assertTrue(self.node.pending)

    def prepare_blocked_start_retry(self):
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.blocked_endpoint = (10., 0.)
        self.node.blocked_route = [(0., 0.), (10., 0.)]
        self.node.blocked_grid_hash = hash(tuple(self.node.grid.data))
        self.node.blocked_until = time.monotonic() + 20.
        self.node.status('start_footprint_blocked', goal=[10., 0.],
                         navigation_epoch=self.node.epoch)
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.local_freshness_issue = Mock(return_value=None)
        self.node.execution_transport_issue = Mock(return_value=None)
        self.node.pose_blockers = Mock(return_value=[])
        self.node.start_segment = Mock()

    def deliver_fresh_global_map(self):
        from copy import deepcopy
        update = deepcopy(self.node.grid)
        stamp = self.node.get_clock().now().nanoseconds - 100_000_000
        update.header.stamp.sec = stamp // 10**9
        update.header.stamp.nanosec = stamp % 10**9
        self.node.on_map(update)

    def test_fresh_clear_costmap_retries_same_blocked_start_without_backoff(self):
        self.prepare_blocked_start_retry()

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_called_once_with()
        self.assertIsNone(self.node.blocked_endpoint)
        self.assertEqual(self.node.last_status['status'], 'blocked_start_cleared')
        self.assertEqual(self.node.last_status['trigger'], 'fresh_costmap_footprint_clear')

    def test_costmap_update_does_not_retry_while_either_footprint_is_blocked(self):
        self.prepare_blocked_start_retry()
        self.node.pose_blockers.return_value = [(3.38, 1.09)]

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'start_footprint_blocked')
        self.assertEqual(self.node.blocked_endpoint, (10., 0.))

    def test_costmap_update_does_not_retry_with_stale_local_map(self):
        self.prepare_blocked_start_retry()
        self.node.local_freshness_issue.return_value = ('stale_local_costmap', 2.0)

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_not_called()
        self.assertEqual(self.node.blocked_endpoint, (10., 0.))

    def test_costmap_update_does_not_resume_retired_navigation_epoch(self):
        self.prepare_blocked_start_retry()
        self.node.last_status['navigation_epoch'] -= 1

        self.deliver_fresh_global_map()

        self.node.start_segment.assert_not_called()
        self.assertEqual(self.node.blocked_endpoint, (10., 0.))

    def test_remembered_goal_routes_through_graph_and_ignores_other_waypoints(self):
'''
    replace_once(tests, test_anchor, test_insert)

    manifest_path = full / 'frozen_source_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf8'))
    changed = [
        'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
        'nav2_adapter/test/test_graph_path_bridge_routing.py',
    ]
    for name in changed:
        manifest['members'][name] = hashlib.sha256((TARGET / 'candidate' / name).read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf8')
    receipt = {
        'source': str(SOURCE), 'target': str(TARGET), 'base': 'frozen_v16',
        'changed_members': {name: manifest['members'][name] for name in changed},
        'member_count': len(manifest['members']),
    }
    (TARGET / 'candidate_delta_v17.json').write_text(
        json.dumps(receipt, indent=2), encoding='utf8')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
