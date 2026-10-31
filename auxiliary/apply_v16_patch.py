from pathlib import Path
import shutil, hashlib, json
base=Path('/home/ly/execution_continuity_review_20261022')
v15=base/'v15_short_reverse_handoff'; root=base/'v16_explore_during_reobserve'
replacements=[]
def replace_once(path, old, new):
    text=path.read_text(encoding='utf8')
    count=text.count(old)
    if count != 1:
        raise RuntimeError(f'{path}: expected one match, got {count}: {old[:90]!r}')
    path.write_text(text.replace(old,new,1),encoding='utf8')
# Rebuild from the frozen v15 candidate so unrelated local user edits cannot leak into this trial.
for rel in ('nav2_adapter/wildos_nav2_adapter/visible_approach.py',
            'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
            'nav2_adapter/test/test_visible_approach_ros.py'):
    shutil.copy2(v15/'candidate'/rel,root/'candidate'/rel)
va=root/'candidate/nav2_adapter/wildos_nav2_adapter/visible_approach.py'
gp=root/'candidate/nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
test=root/'candidate/nav2_adapter/test/test_visible_approach_ros.py'
replace_once(va, '        self.visible_reobserve_until = None\n        self.visible_goal_refresh_pending = False\n', '        self.visible_reobserve_until = None\n        self.visible_reobserve_exploration = False\n        self.visible_goal_refresh_pending = False\n')
insert='''    def check_visible_reobserve_deadline(self):
        """Stop waiting for a fresh mask while frontier exploration continues."""
        deadline = self.visible_reobserve_until
        if (not self.visible_reobserve_exploration or deadline is None
                or time.monotonic() < deadline):
            return False
        self.visible_reobserve_until = None
        self.visible_reobserve_exploration = False
        if self.object_found:
            return False
        self.status('visible_target_reobservation_timeout',
                    query=self.query, exploration_fallback_requested=True)
        return True

'''
replace_once(va, '    def advance_visible_goal(self):\n', insert+'    def advance_visible_goal(self):\n')
replace_once(va, '        reobserving = self.visible_approach_active and self.visible_reobserve_until is not None\n        if reobserving:\n            if time.monotonic() >= self.visible_reobserve_until:\n                self.release_visible_approach(\'visible_target_reobservation_timeout\')\n                return False\n', '''        self.check_visible_reobserve_deadline()
        reobserving = self.visible_reobserve_until is not None
        if reobserving and time.monotonic() >= self.visible_reobserve_until:
            self.release_visible_approach('visible_target_reobservation_timeout')
            return False
''')
replace_once(va, '        self.visible_reobserve_until = None\n        self.expected = point\n', '        self.visible_reobserve_until = None\n        self.visible_reobserve_exploration = False\n        self.expected = point\n')
replace_once(va, '''        # A short detection gap is not a completed or abandoned target search.
        # Keep ownership at the reached pose, with no path/action to repeat.
        # Only a fresh mask-derived hint may start the next checked route.
        self.expected = completed
        self.visible_approach_active = True
        self.visible_reobserve_until = time.monotonic()+REOBSERVATION_WAIT_SEC
        self.status('visible_target_reobserving', goal=list(completed),
                    target_range_known=False, requires_visual_confirmation=True,
                    wait_sec=REOBSERVATION_WAIT_SEC)
''', '''        # Do not leave the vehicle idle while waiting for the next camera mask.
        # Release the completed observation goal to GraphNav immediately; a
        # later fresh hint can still preempt exploration through the normal
        # reset barrier and checked target route.
        self.expected = None
        self.visible_approach_active = False
        self.visible_reobserve_until = time.monotonic()+REOBSERVATION_WAIT_SEC
        self.visible_reobserve_exploration = True
        self.status('visible_target_reobserving', goal=list(completed),
                    target_range_known=False, requires_visual_confirmation=True,
                    wait_sec=REOBSERVATION_WAIT_SEC,
                    exploration_fallback_requested=True)
        self.request_graph_reset()
        self.retry_graph_reset()
''')
replace_once(gp, '    def monitor_inputs(self):\n        if self.check_visible_approach_deadline():\n', '    def monitor_inputs(self):\n        if self.check_visible_reobserve_deadline():\n            return\n        if self.check_visible_approach_deadline():\n')
replace_once(gp, '                or self.visible_reobserve_until is not None):\n', '                or (self.visible_reobserve_until is not None\n                    and not self.visible_reobserve_exploration)):\n')
replace_once(gp, '        if self.visible_reobserve_until is not None:\n            self.route_diagnostic(\'visible_target_reobserving\')\n', '        if (self.visible_reobserve_until is not None\n                and not self.visible_reobserve_exploration):\n            self.route_diagnostic(\'visible_target_reobserving\')\n')
replace_once(gp, '        if self.visible_approach_active:\n            self.reset()\n            self.init_visible_approach()\n            self.expected = None\n', '        if self.visible_approach_active or self.visible_reobserve_exploration:\n            self.reset()\n            self.init_visible_approach()\n            self.expected = None\n')
replace_once(test, '''        self.assertEqual(self.bridge.expected, (8., 0.))
        self.assertTrue(self.bridge.visible_approach_active)
        self.assertFalse(self.bridge.query_reset_pending)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'visible_target_reobserving')
''', '''        self.assertIsNone(self.bridge.expected)
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertTrue(self.bridge.visible_reobserve_exploration)
        self.assertTrue(self.bridge.query_reset_pending)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'visible_target_reobserving')
        self.assertTrue(self.bridge.last_status['exploration_fallback_requested'])
''')
replace_once(test, '        data.update(observation_stamp_ns=self.now,point=[16.,0.])\n        self.bridge.on_visible_approach(String(data=json.dumps(data)))\n', '        data.update(observation_stamp_ns=self.now,point=[16.,0.])\n        self.bridge.query_reset_pending=False\n        self.bridge.on_visible_approach(String(data=json.dumps(data)))\n')
replace_once(test, '''    def test_graph_route_cannot_reexecute_completed_point_while_reobserving(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        self.bridge.route_diagnostic=Mock()
        self.bridge.start_segment=Mock()
        self.bridge.on_route(Path())
        self.bridge.route_diagnostic.assert_called_once_with('visible_target_reobserving')
        self.bridge.start_segment.assert_not_called()
''', '''    def test_graph_route_is_not_suppressed_while_exploration_reobserves(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        self.bridge.route_diagnostic=Mock()
        self.bridge.start_segment=Mock()
        route_message=Path()
        route_message.header.frame_id='spot1/odom'
        route_message.header.stamp=self.stamp()
        self.bridge.query_reset_pending=False
        self.bridge.on_route(route_message)
        self.assertEqual(self.bridge.route_diagnostic.call_args.args[0], 'empty_path')
        self.bridge.start_segment.assert_not_called()

    def test_reobservation_timeout_releases_wait_without_resetting_exploration(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        with patch('time.monotonic', return_value=100.):
            self.bridge.await_object_observation()
        self.assertTrue(self.bridge.visible_reobserve_exploration)
        self.bridge.status_pub.reset_mock()
        with patch('time.monotonic', return_value=111.):
            self.assertTrue(self.bridge.check_visible_reobserve_deadline())
        self.assertFalse(self.bridge.visible_reobserve_exploration)
        self.assertIsNone(self.bridge.visible_reobserve_until)
        self.assertTrue(self.bridge.query_reset_pending)
        status=json.loads(self.bridge.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['status'], 'visible_target_reobservation_timeout')
        self.assertTrue(status['exploration_fallback_requested'])
''')
replace_once(test, '        self.assertEqual(self.bridge.expected,(8.,0.))\n        self.assertEqual(self.bridge.epoch,epoch)\n', '        self.assertIsNone(self.bridge.expected)\n        self.assertEqual(self.bridge.epoch,epoch)\n')
manifest_path=root/'full/frozen_source_manifest.json'
manifest=json.loads(manifest_path.read_text())
changed=['nav2_adapter/wildos_nav2_adapter/visible_approach.py','nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/test/test_visible_approach_ros.py']
for name in changed:
    manifest['members'][name]=hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()
manifest_path.write_text(json.dumps(manifest,indent=2))
print(json.dumps({'restored_v15_base':changed,'patched':changed,'members':len(manifest['members'])}))
