"""Offline mocks only: no game connection or simulated runtime evidence."""
import contextlib
import io
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import cv2
import numpy as np
from PIL import Image
from preview import format_frame, lidar_xy, run_preview, save_samples, preflight
from run import adhoc, parse_args
from run import execute, DiagnosticFailure, camera_check
from fixture import geometry
from build import write_json


class PreviewTests(unittest.TestCase):
    def sample(self, lidar=False):
        reading = {'pointCloud': np.array([[10.,21.,2.], [np.nan,0,0], [1000,0,0]])} if lidar else {
            'colour': Image.new('RGB', (960,640), (255,0,0))}
        return reading, dict(host_monotonic_end=time.monotonic(), simulation_timestamp=None,
                             pose_before=dict(position=[10,20,2])), None

    def gui(self, key=ord('q'), visible=1):
        gui = Mock(wraps=cv2)
        gui.WINDOW_NORMAL = cv2.WINDOW_NORMAL
        gui.WND_PROP_VISIBLE = cv2.WND_PROP_VISIBLE
        gui.FONT_HERSHEY_SIMPLEX = cv2.FONT_HERSHEY_SIMPLEX
        gui.COLOR_RGB2BGR = cv2.COLOR_RGB2BGR
        gui.error = cv2.error
        gui.namedWindow = Mock()
        gui.destroyWindow = Mock()
        gui.imshow = Mock()
        gui.waitKey = Mock(return_value=key)
        gui.getWindowProperty = Mock(return_value=visible)
        return gui

    def test_rgb_conversion_and_uncalibrated_label(self):
        gui = Mock(wraps=cv2)
        gui.FONT_HERSHEY_SIMPLEX = cv2.FONT_HERSHEY_SIMPLEX
        gui.COLOR_RGB2BGR = cv2.COLOR_RGB2BGR
        image, counts = format_frame(gui, 'mounted', self.sample(), False, 'unresolved')
        np.testing.assert_array_equal(image[-1, -1], [0,0,255])
        self.assertEqual(counts, {})
        self.assertTrue(any('NOT CALIBRATED' in c.args[1] for c in gui.putText.call_args_list))

    def test_lidar_queried_origin_filter_and_raw_save(self):
        sample = self.sample(True)
        raw = sample[0]['pointCloud'].copy()
        xy, counts = lidar_xy(raw, [10,20,2], 'raw_world')
        np.testing.assert_array_equal(xy, [[0,1]])
        self.assertEqual(counts['total'], 3)
        self.assertEqual(counts['finite'], 2)
        self.assertEqual(counts['displayed'], 1)
        image, panels = format_frame(cv2, 'lidar', sample, False, 'unresolved')
        self.assertEqual(image.shape, (750,1280,3))
        self.assertEqual(len(panels), 2)
        with tempfile.TemporaryDirectory() as temp:
            save_samples(Path(temp), {'lidar':sample}, 'unresolved', lambda s: None)
            saved = next(Path(temp).rglob('lidar.npy'))
            np.testing.assert_array_equal(np.load(saved), raw)

    def test_close_key_timeout_and_sequential_worker(self):
        for key, visible, seconds, reason in [(ord('q'),1,0,'key'), (27,1,0,'key'),
                                             (-1,0,0,'window closed'), (-1,1,.01,'duration')]:
            state, threads = {}, []
            def poll(sensor, timeout, silent, stop):
                threads.append(threading.get_ident())
                return self.sample()[:2]
            gui = self.gui(key, visible)
            run_preview(gui, {'static':object(), 'mounted':object()}, poll, .1, seconds,
                        Path('.'), {'calibrated':False, 'checks':{}}, state, lambda s: None)
            self.assertEqual(state['exit_reason'], reason)
            self.assertFalse(state['worker_alive'])
            self.assertEqual(len(set(threads)), 1)
            self.assertNotEqual(threads[0], threading.get_ident())
            self.assertEqual(gui.destroyWindow.call_count, 2)
            if not visible:
                self.assertEqual(gui.imshow.call_count, 2)

    def test_blocked_worker_marks_cleanup_unsafe(self):
        release = threading.Event()
        entered = threading.Event()
        def poll(*args, **kwargs):
            entered.set()
            release.wait(5)
            return self.sample()[:2]
        gui = self.gui()
        gui.waitKey.side_effect = lambda _: (entered.wait(1), ord('q'))[1]
        state = {}
        try:
            run_preview(gui, {'static':object()}, poll, .01, 0, Path('.'),
                        {'calibrated':False,'checks':{}}, state, lambda s: None)
            self.assertTrue(state['worker_alive'])
        finally:
            release.set()

    def test_worker_failure_is_fatal(self):
        gui = self.gui(-1)
        with self.assertRaisesRegex(RuntimeError, 'connection lost'):
            run_preview(gui, {'static':object()}, Mock(side_effect=OSError('connection lost')), .1, 1,
                        Path('.'), {'calibrated':False,'checks':{}}, {}, lambda s: None)

    def test_adhoc_cancel_before_socket_and_silent(self):
        sensor = Mock()
        stop = threading.Event()
        stop.set()
        with contextlib.redirect_stdout(io.StringIO()) as out, self.assertRaises(InterruptedError):
            adhoc(sensor, 1, silent=True, stop=stop)
        sensor.get_position.assert_not_called()
        self.assertEqual(out.getvalue(), '')

    def test_cli_preview_explicit(self):
        base = ['--home','.', '--user','.', '--connect','--replace-current']
        self.assertFalse(parse_args(base).preview)
        a = parse_args(base+['--opencv','--mounted','--lidar','--preview'])
        self.assertEqual(a.preview_seconds, 0)
        for extra in (['--preview-seconds','1'], ['--preview','--preview-seconds','nan']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(base+extra)

    def test_headless_preflight(self):
        with patch.object(cv2, 'getBuildInformation', return_value='  GUI: NONE'), self.assertRaisesRegex(RuntimeError, 'requires GUI'):
            preflight()

    def test_fit_failure_is_diagnostic_not_transport(self):
        reading, meta, _ = self.sample()
        meta['pose_before']['direction'] = [0,-1,0]
        sensor = Mock()
        sensor.world_point_to_pixel.return_value = (0,0)
        with tempfile.TemporaryDirectory() as temp, patch('run.adhoc', return_value=(reading,meta)), \
                patch('run.coordinates', side_effect=ValueError('Invalid pose or targets behind camera')):
            with self.assertRaises(DiagnosticFailure):
                camera_check(sensor, geometry()[1], Path(temp), 'mounted', False, [0,0,1], 1)
            self.assertTrue((Path(temp)/'mounted.png').exists())
            self.assertTrue((Path(temp)/'mounted.json').exists())
        with tempfile.TemporaryDirectory() as temp, patch('run.adhoc', side_effect=OSError('socket failed')):
            with self.assertRaises(OSError):
                camera_check(sensor, geometry()[1], Path(temp), 'mounted', False, [0,0,1], 1)

    def test_execute_lifecycle_and_unsafe_cleanup(self):
        for blocked, mobile in ((False, False), (True, False), (False, True)):
            with tempfile.TemporaryDirectory() as temp, contextlib.ExitStack() as stack:
                root = Path(temp)
                write_json(root/'generated/fixture.json', geometry()[1])
                installed = root/'mods/nz_sensor_calibration.zip'
                installed.parent.mkdir()
                installed.write_bytes(b'test-only')
                bng, car, scene = Mock(), Mock(), Mock()
                car.vid = 'test-car'
                car.state = dict(pos=[0,4,1], vel=[0,0,0], up=[0,0,1])
                bng.control.queue_lua_command.side_effect = [str(root), 'true']
                bng.vehicles.start_connection.return_value = dict(vid=car.vid, result=64257)
                cameras = [Mock(name='static'), Mock(name='mounted')]
                lidar = Mock(name='lidar')
                imu = Mock(name='imu')
                events = []
                def create_camera(*args, **kwargs):
                    events.append('create')
                    return cameras[len(events)-1]
                def check(*args, **kwargs):
                    self.assertEqual(events, ['create','create','create'])
                    raise DiagnosticFailure('test fit failure')
                def preview(*args):
                    report, state = args[6:8]
                    self.assertFalse(report['calibrated'])
                    self.assertEqual(set(args[1]), {'static','mounted','lidar','imu'} if mobile else {'static','mounted','lidar'})
                    if mobile:
                        from run import periodic_poll
                        self.assertIs(args[2], periodic_poll)
                        patches['run.camera_check'].assert_not_called()
                        patches['run.lidar_check'].assert_not_called()
                        lidar_call = patches['beamngpy.sensors.Lidar'].call_args
                        self.assertIs(lidar_call.args[2], car)
                        self.assertFalse(lidar_call.kwargs['is_static'])
                        self.assertFalse(lidar_call.kwargs['is_dir_world_space'])
                        imu_call = patches['beamngpy.sensors.AdvancedIMU'].call_args
                        self.assertIs(imu_call.args[2], car)
                        self.assertEqual(imu_call.kwargs['pos'], (0,0,1))
                        self.assertEqual(imu_call.kwargs['physics_update_time'], .01)
                        self.assertEqual(imu_call.kwargs['gfx_update_time'], .2)
                        self.assertTrue(imu_call.kwargs['is_using_gravity'])
                        self.assertFalse(imu_call.kwargs['is_visualised'])
                    for sensor in [*cameras, lidar]:
                        sensor.remove.assert_not_called()
                    state['worker_alive'] = blocked
                patches = {
                    'run.ROOT':root, 'run.inspect_home':Mock(), 'run.verified_mod':Mock(return_value=installed),
                    'run.user_paths':Mock(return_value=(root,root)), 'run.scenario':Mock(return_value=(scene,car)),
                    'run.time.sleep':Mock(), 'run.importlib.metadata.version':Mock(return_value='1.36'),
                    'run.camera_check':Mock(side_effect=check), 'run.lidar_check':Mock(return_value={'passed':False}),
                    'preview.preflight':Mock(), 'preview.run_preview':Mock(side_effect=preview),
                    'beamngpy.BeamNGpy':Mock(return_value=bng), 'beamngpy.connection.Connection':Mock(),
                    'beamngpy.sensors.Camera':Mock(side_effect=create_camera),
                    'beamngpy.sensors.AdvancedIMU':Mock(return_value=imu),
                    'beamngpy.sensors.Lidar':Mock(side_effect=lambda *a, **k: (events.append('create'),lidar)[1]),
                }
                for target, value in patches.items():
                    stack.enter_context(patch(target, value))
                args = parse_args(['--home',temp,'--user',temp,'--connect','--replace-current',
                                   '--mounted','--lidar','--preview'] +
                                  (['--preview-only','--mounted-lidar','--imu'] if mobile else []))
                self.assertEqual(execute(args), 2)
                for sensor in [*cameras, lidar]:
                    self.assertEqual(sensor.remove.call_count, 0 if blocked else 1)
                self.assertEqual(bng.disconnect.call_count, 0 if blocked else 1)
                self.assertEqual(car.control.call_count, 1 if blocked else 2)
                self.assertEqual(imu.remove.call_count, int(mobile))


if __name__ == '__main__':
    unittest.main(verbosity=2)
