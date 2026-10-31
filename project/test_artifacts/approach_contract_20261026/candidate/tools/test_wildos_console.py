"""Console lifecycle/ownership regression tests. No ROS, simulator or SSH."""
import json
import io
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

import wildos_console as module


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.out = Path(self.tmp.name)
        self.backend = Mock()
        self.backend.arrival.return_value = None
        self.backend.failure_image.return_value = {'available': False}
        self.now = [0.]
        self.loop = module.Experiment(self.backend, self.out, 5., lambda: self.now[0])

    def report(self, number=1):
        return json.loads((self.out/f'goal_{number:03d}/result.json').read_text(encoding='utf8'))

    def test_graphnav_library_identity_is_required_before_engine_start(self):
        import hashlib
        fake = S(get_package_share_directory=lambda _:str(self.out/'share'),
                 get_package_prefix=lambda _:str(self.out/'install'))
        library=self.out/'install/lib'
        library.mkdir(parents=True)
        config={'library_sha256':{}}
        for name in ('libwildos_ackermann_smac.so','libwildos_ackermann_rpp.so',
                     'libgraphnav_mapper_component.so','libgraphnav_planner_component.so'):
            (library/name).write_bytes(name.encode())
            config['library_sha256'][name]=hashlib.sha256(name.encode()).hexdigest()
        empty = self.out/'source'; empty.mkdir()
        with patch.dict(module.sys.modules,{'ament_index_python':S(),
                'ament_index_python.packages':fake}), patch.object(module,'ROOT',empty), \
                patch('importlib.util.find_spec',return_value=S(submodule_search_locations=[str(empty)])):
            hashes=module.verify_sources(config)
            self.assertIn(str(library/'libgraphnav_planner_component.so'),hashes)
            self.assertIn(str(library/'libgraphnav_mapper_component.so'),hashes)
            mapper = library/'libgraphnav_mapper_component.so'
            mapper.write_bytes(b'old unrelated mapper')
            with self.assertRaisesRegex(RuntimeError,r'C\+\+插件'):
                module.verify_sources(config)
            mapper.write_bytes(mapper.name.encode())
            config['library_sha256'].pop('libgraphnav_planner_component.so')
            with self.assertRaisesRegex(RuntimeError,'C\\+\\+插件'):
                module.verify_sources(config)

    def test_status_payload_query_is_logged_without_duplicate_keyword_exception(self):
        node=S(started_at=module.time.monotonic(), phase='idle', query='old', events=[],log=io.StringIO())
        result=module.record_console_event(node,'plan_status',dict(query='new',status='awaiting_graph_route'))
        self.assertEqual(result['query'],'new')
        self.assertEqual(json.loads(node.log.getvalue()),result)

    def test_actual_console_query_event_has_the_steady_deadline_origin(self):
        node=S(started_at=module.time.monotonic(),phase='interactive_search',query='green residential house',events=[],log=io.StringIO())
        with patch.object(module.time,'monotonic_ns',return_value=1234567890123):
            result=module.record_console_event(node,'query_sent',{})
        self.assertEqual(result['monotonic_ns'],1234567890123)
        self.assertEqual(json.loads(node.log.getvalue())['monotonic_ns'],1234567890123)

    def test_two_goals_reuse_backend_without_reset_and_never_count_as_acceptance(self):
        for i, query in enumerate(('green residential house', 'gas station'), 1):
            self.loop.start(query)
            self.backend.arrival.return_value = {'images': {'rgb': 'matched.png'}}
            self.loop.tick()
            report = self.report(i)
            self.assertFalse(report['passed'])
            self.assertEqual(report['state'], 'automatic_arrival_pending_visual_review')
        self.assertEqual(self.backend.begin.call_count, 2)
        self.assertEqual(self.backend.stop.call_count, 2)
        self.backend.close.assert_not_called()

    def test_busy_target_does_not_replace_current_owner(self):
        self.loop.start('one')
        with self.assertRaisesRegex(ValueError, '正在搜寻'):
            self.loop.start('two')
        self.assertEqual(self.loop.active['query'], 'one')
        self.backend.begin.assert_called_once()

    def test_query_normalization_matches_model_and_bridge(self):
        self.loop.start('  GREEN   Residential HOUSE  ')
        self.assertEqual(self.loop.active['query'], 'green residential house')

    def test_same_query_after_stop_cannot_silently_reuse_old_goal_generation(self):
        self.loop.start('tree')
        self.loop.finish('user_stopped')
        with self.assertRaisesRegex(ValueError, '不会创建新一轮'):
            self.loop.start(' TREE ')
        self.backend.begin.assert_called_once()

    def test_initial_model_query_is_rejected_without_arming(self):
        self.loop.last_query = 'door'
        with self.assertRaises(ValueError):
            self.loop.start('door')
        self.backend.begin.assert_not_called()

    def test_timeout_stops_and_retains_failure_image(self):
        self.loop.start('tree')
        self.now[0] = 6.
        self.loop.tick()
        self.assertEqual(self.report()['state'], 'timeout')
        self.backend.stop.assert_called_once()
        self.assertIn('failure_image', self.report())
        self.assertIsNone(self.loop.active)

    def test_query_handshake_failure_always_stops(self):
        self.backend.begin.side_effect = RuntimeError('missing ACK')
        with self.assertRaisesRegex(RuntimeError, 'missing ACK'):
            self.loop.start('tree')
        self.backend.stop.assert_called_once()
        self.assertEqual(self.report()['state'], 'start_failed')

    def test_interrupt_during_handshake_stops_before_propagating(self):
        self.backend.begin.side_effect = InterruptedError('user stop')
        with self.assertRaises(InterruptedError):
            self.loop.start('tree')
        self.backend.stop.assert_called_once()
        self.assertIsNone(self.loop.active)

    def test_snapshot_failure_cannot_skip_stop(self):
        self.loop.start('tree')
        self.backend.failure_image.side_effect = OSError('disk full')
        self.loop.finish('user_stopped')
        self.backend.stop.assert_called_once()
        self.assertIn('failure_image_error', self.report())

    def test_stop_failure_latches_fault_and_forbids_rearm(self):
        self.loop.start('tree')
        self.backend.stop.side_effect = RuntimeError('no fresh speed')
        with self.assertRaises(RuntimeError):
            self.loop.finish('user_stopped')
        self.assertEqual(self.report()['state'], 'unsafe_stop_or_runtime_failure')
        with self.assertRaisesRegex(RuntimeError, '禁止重新解锁'):
            self.loop.start('station')
        self.backend.begin.assert_called_once()

    def test_query_is_data_and_goal_folder_does_not_derive_from_query(self):
        query = '../../target; $(echo injection)'
        self.loop.start(query)
        self.assertEqual(self.loop.active['query'], query)
        self.assertEqual(Path(self.loop.active['output']), self.out/'goal_001')

    def test_invalid_query_cannot_start_backend(self):
        for value in ('', '  ', 'a\nb', 'x'*201, 'a\x00b'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.loop.start(value)
        self.backend.begin.assert_not_called()

    def test_missing_arrival_never_passes(self):
        self.loop.start('tree')
        for _ in range(10):
            self.loop.tick()
        self.assertEqual(self.loop.active['state'], 'searching')
        self.backend.stop.assert_not_called()

    def test_wait_cancel_does_not_evaluate_ready_predicate(self):
        event = threading.Event(); event.set()
        owned, predicate = Mock(), Mock()
        with self.assertRaises(InterruptedError):
            module.wait_ready(owned, predicate, event, 10)
        predicate.assert_not_called()

    def owner(self, identity=None):
        owned = module.OwnedProcesses(self.out, self.out)
        child = Mock(pid=123)
        child.poll.return_value = None
        identity = identity or dict(pid=123, start='100', boot='a')
        owned.children = [('stack', child, identity)]
        owned.known[123] = [identity]
        return owned, child, identity

    def test_reused_pid_is_not_signalled(self):
        owned, _, _ = self.owner()
        with patch.object(module, 'group_members', return_value=[dict(pid=123, start='200', boot='a')]), \
             patch.object(module.os, 'killpg', create=True) as kill:
            self.assertTrue(owned.close())
        kill.assert_not_called()

    def test_unproven_orphan_group_is_not_signalled(self):
        owned, _, _ = self.owner()
        with patch.object(module, 'group_members', return_value=[dict(pid=124, start='101', boot='a')]), \
             patch.object(module.os, 'killpg', create=True) as kill:
            self.assertTrue(owned.close())
        kill.assert_not_called()

    def test_owned_descendant_is_stopped_even_when_launch_leader_exited(self):
        owned, child, _ = self.owner()
        child.poll.return_value = 0
        descendant = dict(pid=124, start='101', boot='a')
        owned.known[123].append(descendant)
        with patch.object(module, 'group_members', side_effect=[[descendant], [], []]), \
             patch.object(module.signal, 'SIGKILL', 9, create=True), \
             patch.object(module.os, 'killpg', create=True) as kill:
            self.assertEqual(owned.close(), [])
        kill.assert_called_once_with(123, signal.SIGINT)

    def test_emergency_stop_rejects_pid_reuse(self):
        identity = dict(pid=123, start='100', boot='a')
        module.write_json(self.out/'console_identity.json', identity)
        with patch.object(module, 'proc_identity', return_value=dict(identity, start='101')), \
             patch.object(module.os, 'kill') as kill:
            with self.assertRaisesRegex(RuntimeError, '复用'):
                module.signal_console(self.out)
        kill.assert_not_called()

    def test_bootstrap_keeps_setup_path_as_positional_data(self):
        malicious = '/path/with space/$(touch bad);.bash'
        config = dict(setups=[malicious], python='/python', vision_python='/vision')
        with patch.object(Path, 'is_file', return_value=True), \
             patch.object(module.os, 'execvpe') as execute:
            module.bootstrap(self.out/'config.json', config)
        argv = execute.call_args.args[1]
        self.assertEqual(argv[5], malicious)
        self.assertNotIn(malicious, argv[2])

    def test_cli_help_has_no_runtime_imports(self):
        result = subprocess.run([module.sys.executable, module.__file__, '--help'], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def backend_for_query(self, status_stamp=101):
        backend = module.PhysicalBackend({}, self.out, Mock(), threading.Event())
        backend.bng = Mock()
        backend.node = Mock()
        backend.node.frames = [1]
        backend.node.state = {'odom': 1}
        backend.node.get_subscriptions_info_by_topic.return_value = [S(node_name='rosbag2_recorder')]
        backend.node.spin.side_effect = lambda seconds, done: done()
        backend.node.query_started_ns = 100
        def change(query):
            backend.node.last_status = dict(query=query, schema_version=2, stamp_ns=status_stamp)
        backend.node.change_query.side_effect = change
        backend.stop = Mock()
        backend.node.events = []
        return backend

    def test_backend_waits_for_recording_and_current_query_before_arming(self):
        backend = self.backend_for_query()
        recorder = Mock(); recorder.poll.return_value = None
        with patch.object(module.subprocess, 'Popen', return_value=recorder) as launch:
            backend.begin('station', self.out)
        for topic in ('/spot1/graphnav_planner/rejected_route_endpoint',
                      '/spot1/traversal_feedback', '/spot1/graphnav_planner/route_status',
                      '/spot1/graphnav_planner/vehicle_choice',
                      '/spot1/graphnav_planner/selection_ack',
                      # Full planner output (connector + suffix) is required to
                      # separate them when diagnosing reverse-first sections.
                      '/wildos/full_planned_path'):
            self.assertIn(topic, launch.call_args.args[0])
        self.assertEqual(backend.node.set_bool.call_args_list[-1].args,
                         ('/wildos_graph_path_bridge', 'follow_path', True))
        self.assertTrue(backend.bag is recorder)
        backend.stop.assert_called_once_with(pause=False)
        for name in ('log','native_log','sensor_timing_log'):
            getattr(backend.node,name).close()

    def test_same_query_old_status_cannot_open_control_gates(self):
        backend = self.backend_for_query(status_stamp=99)
        recorder = Mock(); recorder.poll.return_value = None
        try:
            with patch.object(module.subprocess, 'Popen', return_value=recorder):
                with self.assertRaisesRegex(RuntimeError, 'Bridge'):
                    backend.begin('station', self.out)
            backend.node.set_bool.assert_not_called()
        finally:
            for name in ('log','native_log','sensor_timing_log'):
                getattr(backend.node,name).close()

    def test_stop_signal_during_ack_cannot_open_control_gates(self):
        backend = self.backend_for_query()
        def cancel(query):
            backend.interrupted.set()
            raise InterruptedError('cancel')
        backend.node.change_query.side_effect = cancel
        try:
            with patch.object(module.subprocess, 'Popen'):
                with self.assertRaises(InterruptedError):
                    backend.begin('station', self.out)
            backend.node.set_bool.assert_not_called()
        finally:
            for name in ('log','native_log','sensor_timing_log'):
                getattr(backend.node,name).close()

    def test_gate_failure_still_attempts_sdk_stop_and_pause(self):
        backend = module.PhysicalBackend({}, self.out, Mock(), threading.Event())
        backend.node, backend.bng = Mock(), Mock()
        backend.node.stop.side_effect = RuntimeError('service down')
        fake = S(read_physics_snapshot=Mock(), physical_snapshot_errors=Mock(return_value=[]))
        with patch.dict(module.sys.modules, {'wildos_nav2_adapter.physical_vehicle': fake}), \
             patch('check_continuous_start_execution.settle', return_value=({'speed':0}, [])) as settle:
            with self.assertRaisesRegex(RuntimeError, 'control gates'):
                backend.stop()
        settle.assert_called_once()
        backend.bng.control.pause.assert_called_once()

    def run_scene(self, errors):
        import wildos_console_scene
        bng, vehicle, scenario = Mock(), Mock(), Mock()
        mode = [False]
        def set_mode(value):
            mode[0] = value == 'realistic_automatic'
        def brake(**kwargs):
            if kwargs.get('brake') and not mode[0]:
                raise RuntimeError('assisted gearbox can interpret brake as reverse')
        vehicle.set_shift_mode.side_effect = set_mode
        vehicle.control.side_effect = brake
        snapshot = dict(speed=0., position=[734.294,-11.946,52.154], mission='east_coast_usa')
        sdk = S(BeamNGpy=Mock(return_value=S(open=Mock(return_value=bng))),
                Vehicle=Mock(return_value=vehicle), Scenario=Mock(return_value=scenario))
        fake_physics = S(read_physics_snapshot=Mock(return_value=snapshot),
                         physical_snapshot_errors=Mock(return_value=errors))
        modules = {'beamngpy': sdk,
            'beamngpy.connection.prefixed_length_socket': S(PrefixedLengthSocket=object),
            'wildos_nav2_adapter.fail_closed_transport': S(install_fail_closed_reads=Mock()),
            'wildos_nav2_adapter.physical_vehicle': fake_physics}
        output = self.out/'scene.json'
        with patch.dict(module.sys.modules, modules), \
             patch.object(module.sys, 'argv', ['scene', '--home', '/engine', '--user', '/user',
                                              '--output', str(output)]), \
             patch('check_continuous_start_execution.settle', return_value=(snapshot, [])):
            result = wildos_console_scene.main()
        return result, json.loads(output.read_text()), bng, vehicle

    def test_fresh_scene_loads_once_and_releases_sdk_before_sensor_start(self):
        result, report, bng, vehicle = self.run_scene([])
        self.assertEqual(result, 0)
        self.assertTrue(report['passed'])
        self.assertEqual(report['map_loads'], 1)
        self.assertEqual(report['teleports'], 0)
        bng.scenario.load.assert_called_once()
        bng.control.pause.assert_called_once()
        vehicle.disconnect.assert_called_once()
        bng.disconnect.assert_called_once()

    def test_bad_physical_scene_is_reported_and_paused(self):
        result, report, bng, vehicle = self.run_scene(['fixed nodes detected'])
        self.assertEqual(result, 1)
        self.assertFalse(report['passed'])
        self.assertIn('fixed nodes', report['error'])
        bng.control.pause.assert_called_once()
        bng.disconnect.assert_called_once()


if __name__ == '__main__':
    unittest.main(verbosity=2)
