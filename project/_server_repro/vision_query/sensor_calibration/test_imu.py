"""Offline sensor contract tests; no simulator connection or GUI windows."""
import contextlib
import io
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from beamngpy.sensors import AdvancedIMU
from preview import format_frame, imu_measurements, sample_ready, save_samples, run_preview
from run import parse_args, periodic_poll
import test_preview


class IMUTests(unittest.TestCase):
    def records(self):
        return [dict(time=12.75, accRaw=[1.25, -2.5, 9.81],
                     angVel=dict(x=.1, y=.2, z=-.3), mass=5,
                     pos=[100, 200, 300], dirX=[1, 0, 0], dirY=[0, 1, 0], dirZ=[0, 0, 1])]

    def test_poll_list_and_empty_without_pose_api(self):
        sensor = Mock(spec=AdvancedIMU)
        for records in (self.records(), []):
            sensor.poll.return_value = records
            reading, meta = periodic_poll(sensor, 1)
            self.assertIs(reading, records)
            self.assertNotIn('pose_before', meta)
            self.assertNotIn('simulation_timestamp', meta)
            self.assertEqual(sample_ready('imu', reading), bool(records))
        self.assertEqual(sensor.method_calls, [('poll', (), {}), ('poll', (), {})])
        stop = threading.Event()
        stop.set()
        with self.assertRaises(InterruptedError):
            periodic_poll(sensor, 1, stop=stop)
        self.assertEqual(sensor.poll.call_count, 2)

    def test_finite_format_and_measurement_only_save(self):
        records = self.records()
        records.append(dict(accRaw=[float('nan'), 1, 2], angVel=[1, 2, 3]))
        records[0]['accSmooth'] = [float('inf'), 0, 0]
        sample = records, dict(host_monotonic_end=time.monotonic()), None
        gui = test_preview.PreviewTests().gui()
        image, _ = format_frame(gui, 'imu', sample, False, 'unresolved')
        self.assertEqual(image.shape, (410, 960, 3))
        text = '\n'.join(c.args[1] for c in gui.putText.call_args_list)
        for expected in ('1 / 2', '12.75', '1.25, -2.5, 9.81', '0.1, 0.2, -0.3', 'NOT CALIBRATED'):
            self.assertIn(expected, text)
        gui.cvtColor.assert_not_called()
        with tempfile.TemporaryDirectory() as temp:
            save_samples(Path(temp), {'imu': sample}, 'unresolved', lambda _: None)
            saved = json.loads(next(Path(temp).rglob('imu.json')).read_text())
            self.assertEqual(saved, imu_measurements(records))
            self.assertEqual(set(saved[0]), {'time', 'mass', 'accRaw', 'angVel'})
            self.assertEqual(records[0]['pos'], [100, 200, 300])

    def test_missing_time_and_empty_format(self):
        for records in ([], [dict(accRaw=[1, 2, 3], angVel=[4, 5, 6])]):
            gui = test_preview.PreviewTests().gui()
            format_frame(gui, 'imu', (records, {'host_monotonic_end': time.monotonic()}, None), False, 'unresolved')
            text = '\n'.join(c.args[1] for c in gui.putText.call_args_list)
            self.assertIn('unavailable' if records else 'Waiting', text)
        for reading in (None, {}, [None], [{'accRaw': [1, 2]}]):
            self.assertFalse(sample_ready('imu', reading))

    def test_empty_then_valid_single_worker(self):
        calls = []
        sensor = Mock(spec=AdvancedIMU)
        sensor.poll.side_effect = [[], self.records(), []]
        gui = test_preview.PreviewTests().gui(-1)
        def poll(*args, **kwargs):
            calls.append(threading.get_ident())
            return periodic_poll(*args, **kwargs)
        state, messages = {}, []
        run_preview(gui, {'imu': sensor}, poll, 1, .45, Path('.'),
                    {'calibrated': False, 'checks': {}}, state, messages.append)
        self.assertEqual(len(calls), 3)
        self.assertEqual(len(set(calls)), 1)
        self.assertNotEqual(calls[0], threading.get_ident())
        self.assertFalse(state['worker_alive'])
        self.assertTrue(any('resumed' in s for s in messages))

    def test_cli(self):
        base = ['--home', '.', '--user', '.', '--connect', '--replace-current']
        for extra in (['--imu'], ['--preview', '--imu'], ['--preview-only', '--imu'],
                      ['--mounted-lidar'], ['--preview', '--preview-only', '--mounted-lidar'],
                      ['--preview', '--lidar', '--mounted-lidar']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(base + extra)
        for extra in (['--imu'], ['--lidar', '--mounted-lidar'],
                      ['--mounted', '--lidar', '--mounted-lidar', '--imu', '--manual-drive']):
            args = parse_args(base + ['--preview', '--preview-only'] + extra)
            self.assertTrue(args.preview_only)
        self.assertFalse(parse_args(base + ['--lidar']).mounted_lidar)

    def test_periodic_lidar_uses_queried_world_pose(self):
        sensor = Mock()
        sensor.get_position.side_effect = [(100, 200, 3), (101, 200, 3)]
        sensor.get_direction.return_value = (1, 0, 0)
        sensor.poll.return_value = {'pointCloud': [[100, 201, 3]]}
        reading, meta = periodic_poll(sensor, 1)
        self.assertEqual(meta['pose_before']['position'], (100, 200, 3))
        self.assertEqual(meta['pose_after']['position'], (101, 200, 3))
        image, panels = format_frame(test_preview.PreviewTests().gui(), 'lidar',
                                    (reading, meta, None), False, 'unresolved')
        self.assertEqual(image.shape[1], 1280)
        self.assertEqual(panels['raw_world']['displayed'], 1)
        self.assertEqual(panels['translated_local_world_axes']['displayed'], 0)

    def test_actual_136_constructor_without_transport(self):
        bng, car = Mock(), Mock(vid='mock-car')
        with patch.object(AdvancedIMU, 'send_ack_ge') as send, \
                patch.object(AdvancedIMU, 'send_recv_ge', return_value={'data': 7}):
            sensor = AdvancedIMU('imu', bng, car, pos=(0, 0, 1), dir=(0, -1, 0), up=(0, 0, 1),
                                 is_using_gravity=True, physics_update_time=.01, gfx_update_time=.2,
                                 is_send_immediately=False, is_dir_world_space=False, is_visualised=False)
            self.assertIs(sensor.vehicle, car)
            sent = send.call_args.kwargs
            self.assertEqual(sent['pos'], (0, 0, 1))
            self.assertEqual(sent['physicsUpdateTime'], .01)
            self.assertEqual(sent['GFXUpdateTime'], .2)
            self.assertTrue(sent['isUsingGravity'])
            self.assertFalse(sent['isVisualised'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
