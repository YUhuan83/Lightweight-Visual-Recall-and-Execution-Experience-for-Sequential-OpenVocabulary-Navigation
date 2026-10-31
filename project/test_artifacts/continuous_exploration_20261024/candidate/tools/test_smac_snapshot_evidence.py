"""Validate the closed, no-motion C++ snapshot evidence independently."""
import hashlib
import json
import subprocess
from pathlib import Path
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'test_artifacts/server_validation_20261010/smac_boundary_evidence.zip'


class SmacSnapshotEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive = zipfile.ZipFile(ARCHIVE)

    @classmethod
    def tearDownClass(cls):
        cls.archive.close()

    def document(self, name):
        return json.loads(self.archive.read('cpp_navigation_probe/' + name))

    def test_archive_and_each_member_match_recorded_hash(self):
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         '92569ea1e9b9f3e03f03f255accdd65eb30fbc9714869306b821442c9783d64a')
        manifest = self.document('manifest.json')
        for name, item in manifest['files'].items():
            with self.subTest(name=name):
                data = self.archive.read(name)
                self.assertEqual(len(data), item['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(), item['sha256'])
        self.assertIn('no BeamNG client or vehicle movement', manifest['scope'])

    def test_recorded_diagnostic_sources_match_the_frozen_repair_commit(self):
        manifest = self.document('manifest.json')
        for name in ('CMakeLists.txt', 'collision_snapshot_probe.cpp',
                     'smac_planner_snapshot_probe.cpp'):
            relative = 'cpp_navigation_probe/' + name
            frozen = subprocess.check_output([
                'git', 'show', 'a9f3b5b87491f33ee165931636c9b5afe7554b17:tools/' + relative], cwd=ROOT)
            self.assertEqual(hashlib.sha256(frozen).hexdigest(),
                             manifest['files'][relative]['sha256'])

    def test_actual_pose_and_lattice_pose_disagree_under_full_boundary_check(self):
        evidence = self.document('snapshot_result.json')
        self.assertTrue(evidence['published_map_not_private_planner_map'])
        self.assertTrue(evidence['soft_cost_inverse_is_approximate'])
        self.assertEqual(evidence['actual']['continuous_boundary_cost'], 253)
        self.assertEqual(evidence['returned_first']['continuous_boundary_cost'], 254)
        self.assertTrue(evidence['returned_first']['grid_collision'])
        self.assertFalse(evidence['returned_first']['configured_shortcut_collision'])

    def test_actual_planner_no_longer_accepts_blocked_quantized_start(self):
        before, after = (self.document('planner_' + stage + '.json')
                         for stage in ('before', 'after'))
        self.assertTrue(before['reconstructed_lethal_map'])
        self.assertTrue(after['reconstructed_lethal_map'])
        self.assertEqual(before['shortcut_cost'], after['shortcut_cost'])
        self.assertEqual(before['returned_first_body_cost'], 254)
        self.assertEqual(after['returned_first_body_cost'], 254)
        self.assertEqual(before['path_size'], 73)
        self.assertFalse(before['quantized_start_collision'])
        self.assertEqual(after['path_size'], 0)
        self.assertTrue(after['quantized_start_collision'])
        self.assertGreaterEqual(after['center_cost'], after['shortcut_cost'])

    def test_installed_cpp_controls_include_normal_failure_and_recovery(self):
        log = self.archive.read('cpp_navigation_probe/build/Testing/Temporary/LastTest.log').decode()
        self.assertIn('9 checker controls passed', log)
        self.assertIn('4 actual inflation/planner controls passed', log)
        summary = self.archive.read('cpp_navigation_probe/controls.log').decode()
        self.assertIn('100% tests passed, 0 tests failed out of 2', summary)


if __name__ == '__main__':
    unittest.main()
