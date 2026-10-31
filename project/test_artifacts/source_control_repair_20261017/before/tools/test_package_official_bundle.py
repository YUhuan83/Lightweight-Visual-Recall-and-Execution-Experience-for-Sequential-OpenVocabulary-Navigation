"""Frozen package publication must reject incomplete or changing sources."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile
from package_official_bundle import ROOT, bundle_sources, write_bundle


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.bundle = self.root/'bundle.zip'
        self.bundle.write_bytes(b'old archive')
        (self.root/'source.txt').write_bytes(b'frozen source')

    def test_published_archive_and_manifest_match_sources(self):
        report = write_bundle(self.root, self.bundle, {'source.txt'})
        with zipfile.ZipFile(self.bundle) as archive:
            self.assertEqual(archive.read('source.txt'), b'frozen source')
        self.assertEqual(report['sha256'], hashlib.sha256(self.bundle.read_bytes()).hexdigest())
        self.assertEqual(json.loads(self.bundle.with_suffix('.manifest.json').read_text()), report)
        self.assertEqual(self.bundle.with_suffix('.sha256').read_text().split()[0].lower(), report['sha256'])

    def test_missing_resource_keeps_prior_archive(self):
        with self.assertRaises(ValueError):
            write_bundle(self.root, self.bundle, {'missing.lua'})
        self.assertEqual(self.bundle.read_bytes(), b'old archive')

    def test_traversal_and_absolute_names_are_rejected(self):
        for name in ('../source.txt', str(self.root/'source.txt')):
            with self.subTest(name=name), self.assertRaises(ValueError):
                write_bundle(self.root, self.bundle, {name})
        self.assertEqual(self.bundle.read_bytes(), b'old archive')

    def test_source_change_during_archive_write_prevents_publication(self):
        real_write = zipfile.ZipFile.write
        def changing(archive, path, name):
            real_write(archive, path, name)
            Path(path).write_bytes(b'changed')
        with patch.object(zipfile.ZipFile, 'write', changing), self.assertRaises(ValueError):
            write_bundle(self.root, self.bundle, {'source.txt'})
        self.assertEqual(self.bundle.read_bytes(), b'old archive')

    def test_current_source_inventory_includes_runtime_and_regressions(self):
        names = bundle_sources(ROOT, ROOT/'official_wildos_beamng_bundle.zip')
        required = {
            'tools/wildos_console.py', 'tools/wildos_console_scene.py',
            'tools/setup_wildos_console.sh', 'tools/wildos-console.ps1',
            'tools/test_wildos_console.py', 'docs/SELF_SERVICE_CONSOLE.md',
            'config/wildos_console.example.json',
            'nav2_adapter/config/fastdds_local_udp.xml',
            'nav2_adapter/wildos_nav2_adapter/dds_profile.py',
            'nav2_adapter/test/test_dds_profile.py','tools/test_physical_trial_policy.py',
            'tools/check_continuous_start_execution.py',
            'tools/check_ackermann_cusp_execution.py','tools/test_cusp_probe_contract.py',
            'ackermann_smac/src/planner.cpp', 'ackermann_smac/src/connector.cpp',
            'ackermann_smac/CMakeLists.txt', 'ackermann_smac/plugin.xml',
            'nav2_adapter/wildos_nav2_adapter/direct_vehicle_control.py',
            'nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.py',
            'nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.lua',
            'nav2_adapter/test/test_planner_execution_ownership_ros.py',
            'tools/test_offline_goal_chain_replay.py',
            'nav2_adapter/wildos_nav2_adapter/navigation_transactions.py',
            'nav2_adapter/wildos_nav2_adapter/execution_hold.py',
            'nav2_adapter/test/test_navigation_transactions_ros.py',
            'tools/test_offline_navigation_state_audit.py',
            'config/sbr_physical_sensor_rig.json',
            'test_artifacts/server_validation_20261012/direct_steering_evidence.zip'}
        self.assertTrue(required <= names, required-names)
        self.assertTrue(any(n.startswith('ackermann_smac/include/') for n in names))
        self.assertTrue(all((ROOT/n).is_file() for n in names))


if __name__ == '__main__':
    unittest.main()
