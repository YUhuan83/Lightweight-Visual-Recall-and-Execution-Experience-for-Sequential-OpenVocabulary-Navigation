"""Synthetic saved-evidence tests; never connect to a simulator."""
import contextlib
import io
import json
import os
import shutil
import subprocess
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

from summarize_capture import main, summarize


class SummaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding='utf-8')

    def test_missing_empty_and_malformed_are_distinct(self):
        result = summarize(self.root)
        self.assertIsNone(result['calibrated'])
        self.assertEqual(result['snapshots'], [])
        snapshot = self.root / 'preview' / 'synthetic'
        self.write(snapshot / 'imu_metadata.json', {'received_samples': 0, 'saved_samples': 0})
        missing = summarize(self.root)['snapshots'][0]['sensors']['imu']
        self.assertIsNone(missing['actual_record_count'])
        self.assertIsNone(missing['actual_empty'])
        self.write(snapshot / 'imu.json', [])
        empty = summarize(self.root)['snapshots'][0]['sensors']['imu']
        self.assertEqual(empty['actual_record_count'], 0)
        self.assertTrue(empty['actual_empty'])
        self.write(snapshot / 'imu.json', {})
        result = summarize(self.root)
        self.assertIsNone(result['snapshots'][0]['sensors']['imu']['finite_accel_gyro_count'])
        self.assertTrue(any('unexpected JSON type' in issue for issue in result['issues']))

    def test_actual_save_samples_schema_and_no_binary_reads(self):
        import numpy as np
        from PIL import Image
        from preview import save_samples
        report = dict(calibrated=False, runtime_observed=True, checks={
            'mounted': {'rgb_std': 8, 'saturated_fraction': .1, 'passed': False}},
            cleanup_errors=['synthetic cleanup failure'])
        self.write(self.root / 'runtime_report.json', report)
        meta = dict(source='synthetic periodic poll; cached frames possible',
                    host_monotonic_start=10, host_monotonic_end=11,
                    pose_before={'position': [0, 0, 2]})
        records = [dict(time=3.5, accRaw=[1, 2, 9.81], angVel={'x': .1, 'y': 0, 'z': -.1}),
                   dict(accRaw=[float('nan'), 0, 0], angVel=[0, 0, 0])]
        save_samples(self.root, {
            'mounted': ({'colour': Image.new('RGB', (2, 2))}, meta, 5),
            'lidar': ({'pointCloud': np.array([[1., 2., 3.]])}, meta, None),
            'imu': (records, meta, None)}, 'unresolved', lambda _: None)
        original = Path.open

        def json_only(path, *args, **kwargs):
            self.assertEqual(path.suffix, '.json')
            return original(path, *args, **kwargs)

        with patch.object(Path, 'open', json_only):
            result = summarize(self.root)
        self.assertEqual(result['diagnostics']['checks'], report['checks'])
        sensors = result['snapshots'][0]['sensors']
        self.assertEqual(sensors['imu']['received_samples'], 2)
        self.assertEqual(sensors['imu']['actual_record_count'], 1)
        self.assertEqual(sensors['imu']['finite_accel_gyro_count'], 1)
        self.assertEqual(sensors['imu']['latest_valid_measurement']['time'], 3.5)
        self.assertFalse(sensors['mounted']['synchronized'])
        self.assertEqual(sensors['lidar']['display_counts']['raw_world']['total'], 1)
        self.assertEqual(sensors['imu']['metadata']['source'], meta['source'])

    def test_invalid_vectors_missing_time_and_count_mismatch(self):
        snapshot = self.root / 'preview' / 'synthetic'
        self.write(snapshot / 'imu.json', [None, {'accRaw': [1, 2], 'angVel': [0, 0, 0]},
                   {'accRaw': [1, 2, 3], 'angVel': [0, 0, 0]}])
        self.write(snapshot / 'imu_metadata.json', {'saved_samples': 9})
        result = summarize(self.root)
        imu = result['snapshots'][0]['sensors']['imu']
        self.assertEqual(imu['actual_record_count'], 3)
        self.assertEqual(imu['finite_accel_gyro_count'], 1)
        self.assertNotIn('time', imu['latest_valid_measurement'])
        self.assertIsNone(imu['received_samples'])
        self.assertTrue(result['issues'])

    def test_cli_latest_stdout_and_explicit_overwrite(self):
        old = self.root / 'captures' / 'zz-old'
        new = self.root / 'captures' / 'aa-new'
        for directory, stamp in ((old, 1000), (new, 2000)):
            self.write(directory / 'runtime_report.json', {'calibrated': False})
            os.utime(directory / 'runtime_report.json', (stamp, stamp))
        with patch('summarize_capture.ROOT', self.root), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(['--latest']), 0)
        self.assertEqual(json.loads(output.getvalue())['capture'], str(new.resolve()))
        destination = self.root / 'summary.json'
        self.assertFalse(destination.exists())
        main([str(new), '--output', str(destination)])
        original = destination.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            main([str(old), '--output', str(destination)])
        self.assertEqual(destination.read_bytes(), original)
        main([str(old), '--output', str(destination), '--overwrite'])
        self.assertEqual(json.loads(destination.read_text())['capture'], str(old.resolve()))
        for args in ([str(old), '--overwrite'], [str(old), '--output', str(self.root / 'runtime_report.json')],
                     [str(old), '--output', str(self.root / 'absent' / 'summary.json')]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args)

    def test_built_source_allowlist(self):
        from build import ROOT, SOURCE_FILES
        archive = ROOT / 'dist' / 'sensor_calibration_source.zip'
        with zipfile.ZipFile(archive) as source:
            self.assertIsNone(source.testzip())
            self.assertEqual(set(source.namelist()), {'sensor_calibration/' + name for name in SOURCE_FILES})
            for name in source.namelist():
                self.assertEqual(source.read(name), (ROOT / Path(name).name).read_bytes())
                self.assertNotIn('.venv', name)
                self.assertNotIn('tech.key', name)
                self.assertIn(Path(name).suffix, ('.py', '.md', '.txt', '.ps1'))

    @unittest.skipUnless(os.name == 'nt' and shutil.which('powershell.exe'), 'Windows PowerShell required')
    def test_powershell_preflight_never_executes_runner(self):
        root = Path(__file__).resolve().parent
        script = self.root / 'start_preview.ps1'
        script.write_bytes((root / script.name).read_bytes())
        # Neither fake executable nor runner may be invoked, even with consent.
        for name in ('python.exe', 'run.py', 'home/Bin64/BeamNG.tech.x64.exe',
                     'home/content/levels/smallgrid.zip', 'home/content/vehicles/pickup.zip',
                     'user/current/mods/nz_sensor_calibration.zip', 'dist/nz_sensor_calibration.zip',
                     'generated/fixture.json', 'generated/package.json'):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('NOT EXECUTABLE', encoding='ascii')
        command = ['powershell.exe', '-NoProfile', '-NonInteractive', '-File', str(script),
                   '-Home', str(self.root / 'home'), '-User', str(self.root / 'user'),
                   '-Python', str(self.root / 'python.exe')]
        for mode in ('connect', 'launch'):
            result = subprocess.run(command + ['-WhatIf', '-ConfirmReplace', '-Sensors', 'all', '-Mode', mode],
                                    capture_output=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(b'WhatIf: preflight only', result.stdout)
            self.assertIn(b'manual-drive=False', result.stdout)
        for extra in ([], ['-WhatIf', '-Dx11']):
            result = subprocess.run(command + extra, capture_output=True, timeout=20)
            self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / 'captures').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
