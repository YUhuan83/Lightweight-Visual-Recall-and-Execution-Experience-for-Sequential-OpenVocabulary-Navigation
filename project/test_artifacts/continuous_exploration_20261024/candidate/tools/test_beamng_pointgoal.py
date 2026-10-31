"""Offline behavior tests; never open a simulator connection."""
import ast
import importlib.util
from pathlib import Path
import socket
import sys
import tempfile
import types
import unittest
from unittest.mock import patch, MagicMock
import zipfile

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('pointgoal', HERE/'beamng_pointgoal.py')
pg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pg)


def road():
    x, y = np.meshgrid(np.arange(-8, 13, .15), np.arange(-7, 7, .15))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.7)))


def state(t=1.):
    return dict(pos=[100., 200., 10.], dir=[0., -1., 0.], up=[0., 0., 1.], vel=[0., 0., 0.], time=t)


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.network = patch.object(socket, 'socket', side_effect=AssertionError('Live connector forbidden'))
        self.network.start()

    def tearDown(self):
        self.network.stop()

    def test_flat_observed_goal_and_astar(self):
        grid, safe, details = pg.plan(road(), [6., 0.])
        self.assertEqual(details['path_local_xy'], [[0., 0.], [6., 0.]])
        self.assertGreater(len(details['astar_cells']), 2)
        self.assertTrue(safe[pg.cell([0, 0])])
        self.assertEqual(grid[pg.cell([15, 15])], 0)

    def test_unknown_gap_is_not_filled(self):
        p = road()
        p = p[(p[:, 0] < 3) | (p[:, 0] > 5)]
        with self.assertRaises(pg.Unsafe):
            pg.plan(p, [6., 0.])

    def test_obstacle_not_ground(self):
        p = road()
        obstacle = np.array([[4+x, y, .1] for x in np.arange(-.5, .6, .1) for y in np.arange(-7, 7, .15)])
        with self.assertRaises(pg.Unsafe):
            pg.plan(np.vstack((p, obstacle)), [6., 0.])

    def test_sparse_cloud_and_sharp_heading_fail(self):
        with self.assertRaises(pg.Unsafe):
            pg.plan(road()[::100], [6., 0.])
        with self.assertRaises(pg.Unsafe):
            pg.plan(road(), [3., 3.])

    def test_no_diagonal_corner_cut(self):
        safe = np.zeros((pg.SIZE, pg.SIZE), dtype=bool)
        safe[32, 32] = safe[33, 33] = True
        with self.assertRaises(pg.Unsafe):
            pg.astar(safe, (32, 32), (33, 33))

    def test_frame_and_round_trip(self):
        basis = pg.frame(state())
        np.testing.assert_allclose(basis[:, 0], [0, -1, 0])
        np.testing.assert_allclose(basis[:, 1], [1, 0, 0])
        local = np.array([3., 2., 1.])
        world = np.array(state()['pos']) + basis @ local
        np.testing.assert_allclose((world-state()['pos']) @ basis, local)

    def test_bracket_quality_and_rejection(self):
        s = state()
        cloud = road() @ pg.frame(s).T + s['pos']
        sensor = np.array(s['pos'])+[0, 0, 2.2]
        quality = pg.cloud_check(cloud, s, state(2), sensor, sensor)
        self.assertIn('hypothesis', quality['world_xyz'])
        for bad in (cloud+1000, np.full((200, 3), np.nan), np.zeros((0, 3))):
            with self.assertRaises(pg.Unsafe):
                pg.cloud_check(bad, s, state(2), sensor, sensor)
        with self.assertRaises(pg.Unsafe):
            pg.cloud_check(cloud, s, s, sensor, sensor)
        after = state(2)
        after['pos'][0] += .1
        with self.assertRaises(pg.Unsafe):
            pg.cloud_check(cloud, s, after, sensor, sensor)

    def test_motion_guard_reverse_speed_corridor(self):
        grid, safe, _ = pg.plan(road(), [6., 0.])
        s = state()
        origin, basis = np.array(s['pos']), pg.frame(s)
        pg.motion_guard(s, origin, basis, np.array([2., 0.]), safe, grid)
        for velocity in ([0, 1, 0], [0, -2, 0]):
            bad = state()
            bad['vel'] = velocity
            with self.assertRaises(pg.Unsafe):
                pg.motion_guard(bad, origin, basis, np.array([2., 0.]), safe, grid)
        bad = state()
        bad['pos'][0] += .4
        with self.assertRaises(pg.Unsafe):
            pg.motion_guard(bad, origin, basis, np.array([2., 0.]), safe, grid)

    def test_motion_requires_both_flags_and_evidence(self):
        args = pg.parser().parse_args(['--goal-forward', '2'])
        pg.validate_args(args)
        self.assertFalse(args.execute_motion)
        args.execute_motion = True
        with self.assertRaises(pg.Unsafe):
            pg.validate_args(args)
        args.tracker = 'beamng-ai'
        with self.assertRaises(pg.Unsafe):
            pg.validate_args(args)
        args.world_frame_verified = 'operator checked fixed curb'
        pg.validate_args(args)
        args.goal_forward = float('nan')
        with self.assertRaises(pg.Unsafe):
            pg.validate_args(args)

    def test_mocked_dry_run_retries_raw_bad_cloud_without_motion(self):
        vehicle = MagicMock()
        vehicle.vid, vehicle.model, vehicle.extensions = 'american_smoke_pickup', 'pickup', None
        vehicle.is_connected.return_value = True
        tick = [0.]
        def poll():
            tick[0] += .6
            vehicle.state = state(tick[0])
        vehicle.sensors.poll.side_effect = poll
        bng = MagicMock()
        bng.vehicles.get_current.return_value = {vehicle.vid: vehicle}
        bng.vehicles.start_connection.return_value = {'vid': vehicle.vid, 'result': 12345}
        lidar = MagicMock()
        lidar.send_ad_hoc_poll_request.return_value = 1
        lidar.is_ad_hoc_poll_request_ready.return_value = True
        lidar.get_position.return_value = [100., 200., 12.2]
        cloud = road() @ pg.frame(state()).T + state()['pos']
        lidar.collect_ad_hoc_poll_request.side_effect = [
            {'pointCloud': np.full((150, 3), np.nan)}, {'pointCloud': cloud}]
        sensors = types.ModuleType('beamngpy.sensors')
        sensors.Electrics = lambda: {}
        sensors.Lidar = MagicMock(return_value=lidar)
        module = types.ModuleType('beamngpy')
        module.BeamNGpy = MagicMock(return_value=bng)
        connection = types.ModuleType('beamngpy.connection')
        connection.Connection = MagicMock()
        real_version = pg.importlib.metadata.version
        with tempfile.TemporaryDirectory(dir=HERE) as tmp:
            args = pg.parser().parse_args(['--goal-forward', '2', '--output-root', tmp])
            with patch.dict(sys.modules, {'beamngpy': module, 'beamngpy.sensors': sensors, 'beamngpy.connection': connection}), \
                 patch.object(pg.importlib.metadata, 'version', side_effect=lambda k: '1.36' if k == 'beamngpy' else real_version(k)), \
                 patch.object(pg.time, 'sleep'):
                self.assertEqual(pg.run(args), 0)
            raw_files = list(Path(tmp).glob('*/step_*/cloud_world_raw.npy'))
            self.assertEqual(len(raw_files), 2)
            self.assertTrue(any(np.isnan(np.load(p)).all() for p in raw_files))
            self.assertEqual(len(list(Path(tmp).glob('*/step_*/plan.json'))), 1)
            vehicle.ai.set_line.assert_not_called()
            bng.scenario.load.assert_not_called()
            bng.open.assert_called_once_with(launch=False)
            for call in vehicle.control.call_args_list:
                self.assertEqual(call.kwargs['throttle'], 0)
                self.assertEqual(call.kwargs['brake'], 1)
                self.assertEqual(call.kwargs['parkingbrake'], 1)
            lidar.remove.assert_called_once()
            vehicle.disconnect.assert_called_once()


class ApiContractTests(unittest.TestCase):
    def test_actual_136_source_signatures_and_forwarding(self):
        wheel = HERE.parent/'beamng/wheelhouse-linux310/beamngpy-1.36-py3-none-any.whl'
        if wheel.exists():
            archive = zipfile.ZipFile(wheel)
            self.addCleanup(archive.close)
            read = lambda name: archive.read('beamngpy/'+name).decode()
        else:
            import beamngpy
            import importlib.metadata
            self.assertEqual(importlib.metadata.version('beamngpy'), '1.36')
            root = Path(beamngpy.__file__).parent
            read = lambda name: (root/name).read_text(encoding='utf-8')
        expected = {
            'api/beamng/vehicles.py': {'get_current': ['self', 'include_config'], 'start_connection': ['self', 'vehicle', 'extensions']},
            'vehicle/vehicle.py': {'connect': ['self', 'bng', 'tries']},
            'api/vehicle/ai.py': {'set_line': ['self', 'line', 'cling']},
            'sensors/lidar.py': {'send_ad_hoc_poll_request': ['self'], 'collect_ad_hoc_poll_request': ['self', 'request_id']}}
        for name, functions in expected.items():
            tree = ast.parse(read(name))
            defs = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
            for func, args in functions.items():
                self.assertEqual([a.arg for a in defs[func].args.args], args)
        source = read('api/vehicle/ai.py')
        self.assertIn('data["cling"] = cling', source)
        self.assertIn('data["line"] = line', source)
        self.assertIn('dict(type="SetAiLine")', source)

    def test_no_scene_or_oracle_calls_and_explicit_cling(self):
        tree = ast.parse((HERE/'beamng_pointgoal.py').read_text(encoding='utf-8'))
        forbidden = {'teleport', 'spawn', 'get_roads', 'get_road_edges', 'set_waypoint', 'drive_using_waypoints', 'set_velocity'}
        for n in ast.walk(tree):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                self.assertNotIn(n.func.attr, forbidden)
                if n.func.attr == 'set_line':
                    self.assertTrue(any(k.arg == 'cling' and isinstance(k.value, ast.Constant) and k.value.value is False for k in n.keywords))


if __name__ == '__main__':
    unittest.main(verbosity=2)
