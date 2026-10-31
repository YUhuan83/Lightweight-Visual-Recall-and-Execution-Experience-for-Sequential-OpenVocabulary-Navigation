import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import preflight as p


class PreflightTests(unittest.TestCase):
    def test_actual_graph(self):
        graph = p.inventory(p.DEFAULT_SOURCE)
        self.assertEqual(len(graph['packages']), 9)
        self.assertIn('graaf_vendor', graph['packages']['graphnav_planner']['source_edges'])
        self.assertIn('pose_graph_msgs', graph['packages']['graphnav_mapper']['source_edges'])
        self.assertEqual(graph['graaf']['tag'], 'v1.1.1')
        self.assertEqual(graph['graaf']['url'], 'https://github.com/bobluppes/graaf.git')

    def test_prepare_actual_source_and_refuse_reuse(self):
        before = p.snapshot(p.DEFAULT_SOURCE)
        with tempfile.TemporaryDirectory(dir=p.HERE, prefix='test-') as tmp:
            dest = Path(tmp) / 'new'
            record = p.prepare(p.DEFAULT_SOURCE, dest)
            staged = dest / 'src/nebula2-wildos'
            self.assertEqual(before, p.snapshot(p.DEFAULT_SOURCE))
            self.assertEqual(record['original_sha256'], before['sha256'])
            after = p.snapshot(staged)['files']
            self.assertEqual(len(record['added_markers']), 7)
            self.assertEqual(set(after) - set(before['files']), set(record['added_markers']))
            for name, digest in before['files'].items():
                self.assertEqual(after[name], digest)
            for directory in json.loads((p.HERE / 'namespace_patch.json').read_text())['directories']:
                self.assertTrue((staged / directory / '__init__.py').is_file())
            with self.assertRaisesRegex(ValueError, 'exists'):
                p.prepare(p.DEFAULT_SOURCE, dest)

    def test_setuptools_discovery(self):
        try:
            from setuptools import find_packages
        except ImportError:
            self.skipTest('setuptools unavailable; stdlib copy checks still run')
        original = find_packages(str(p.DEFAULT_SOURCE / 'visual_navigation'), exclude=['test'])
        self.assertNotIn('visual_navigation.wildos', original)
        with tempfile.TemporaryDirectory(dir=p.HERE, prefix='test-') as tmp:
            dest = Path(tmp) / 'new'
            p.prepare(p.DEFAULT_SOURCE, dest)
            installed = find_packages(str(dest / 'src/nebula2-wildos/visual_navigation'), exclude=['test'])
            for directory in json.loads((p.HERE / 'namespace_patch.json').read_text())['directories']:
                self.assertIn(directory.split('/', 1)[1].replace('/', '.'), installed)

    def test_exclusions_and_hash_changes(self):
        with tempfile.TemporaryDirectory(dir=p.HERE, prefix='test-') as tmp:
            root = Path(tmp)
            (root / 'keep.py').write_text('a')
            (root / '__pycache__').mkdir()
            (root / '__pycache__/cached.pyc').write_bytes(b'cache')
            (root / 'model.pt').write_bytes(b'weight')
            first = p.snapshot(root)
            self.assertEqual(set(first['files']), {'keep.py'})
            (root / 'keep.py').write_text('b')
            self.assertNotEqual(first['sha256'], p.snapshot(root)['sha256'])

    def test_outside_home_refused(self):
        with tempfile.TemporaryDirectory(dir=p.HERE, prefix='test-') as tmp:
            with patch.object(Path, 'home', return_value=Path(tmp) / 'different-home'):
                with self.assertRaisesRegex(ValueError, 'under home'):
                    p.prepare(p.DEFAULT_SOURCE, Path(tmp) / 'new')

    def test_symlink_refused(self):
        with tempfile.TemporaryDirectory(dir=p.HERE, prefix='test-') as tmp:
            root = Path(tmp)
            (root / 'original').write_text('data')
            try:
                (root / 'link').symlink_to(root / 'original')
            except OSError:
                self.skipTest('OS disallows symlink creation')
            with self.assertRaisesRegex(ValueError, 'symlink'):
                p.snapshot(root)

    def test_missing_ros_explicit_and_readonly(self):
        before = p.snapshot(p.DEFAULT_SOURCE)
        with patch.dict('os.environ', {'ROS_DISTRO': ''}), patch.object(p.shutil, 'which', return_value=None):
            report = p.preflight(p.DEFAULT_SOURCE)
        self.assertEqual(report['status'], 'BLOCKED')
        self.assertTrue(any('Missing ROS 2 Humble' in b for b in report['blockers']))
        self.assertEqual(before, p.snapshot(p.DEFAULT_SOURCE))

    def test_cli_requires_no_server_flag(self):
        result = subprocess.run([sys.executable, '-B', str(p.HERE / 'preflight.py'), '--inventory'],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--no-server-precheck', result.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
