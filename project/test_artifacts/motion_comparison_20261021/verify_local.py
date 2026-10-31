"""Verify closed-trial evidence and its measurement boundaries; no navigation pass claim."""
from pathlib import Path
import hashlib
import json
import math
import unittest
import zipfile

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
ARCHIVE_SHA = '67c356ccccc5ce25596b8607ac2ffbefecd9b9b4abd7d9886f04e6d08e24635d'


def digest(data):
    return hashlib.sha256(data).hexdigest()


class EvidenceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        archive = ROOT / 'comparison_evidence.zip'
        assert digest(archive.read_bytes()) == ARCHIVE_SHA
        cls.target = ROOT / 'verified'
        with zipfile.ZipFile(archive) as z:
            assert z.testzip() is None
            cls.manifest = json.loads(z.read('manifest.json'))
            for name, meta in cls.manifest['files'].items():
                path = (cls.target / name).resolve()
                assert path.is_relative_to(cls.target.resolve())
                data = z.read(name)
                assert len(data) == meta['bytes'] and digest(data) == meta['sha256'], name
            z.extractall(cls.target)
        cls.reports = [json.loads((cls.target / f'trial_20261020_{x}.json').read_text())
                       for x in ('215010', '224538', '225946')]

    def test_duration_conservation_and_query_scope(self):
        for report in self.reports:
            self.assertTrue(report['closed'])
            self.assertEqual(report['query'], 'green residential house')
            self.assertFalse(report['complete_navigation_passed'])
            w = report['overall']['weights']
            self.assertAlmostEqual(w['observed_sec'] + w.get('unobserved_sec', 0), 420., places=5)
            self.assertAlmostEqual(w['stationary_sec'] + w['moving_sec'], w['observed_sec'], places=5)
            self.assertAlmostEqual(sum(m['weights']['observed_sec'] for m in report['minutes']),
                                   w['observed_sec'], places=5)
            self.assertLessEqual(w['stationary_actual_brake_sec'], w['stationary_sec'])

    def test_same_physics_and_runtime_libraries(self):
        first = self.reports[0]
        fixed = ('nodeCount', 'bodyCollisionNodeCount', 'wheelCount', 'massKg',
                 'configKinematic', 'configBodyDisabled', 'collisionOverrideLoaded')
        expected_libs = {k: v['sha256'] for k, v in first['runtime_identity']['libraries'].items()}
        for report in self.reports:
            self.assertEqual({k: report['scene']['physics'][k] for k in fixed},
                             {k: first['scene']['physics'][k] for k in fixed})
            self.assertEqual({k: v['sha256'] for k, v in report['runtime_identity']['libraries'].items()},
                             expected_libs)
        for name in ('native_vehicle_control.py', 'cmd_vel_to_control.py', 'source_control.py',
                     'continuous_path.py', 'motion_path.py', 'graph_route.py'):
            key = 'nav2_adapter/wildos_nav2_adapter/' + name
            self.assertEqual(len({r['verified_sources'][key] for r in self.reports}), 1)

    def test_exact_candidate_delta_and_current_sources(self):
        old = json.loads((self.target / 'provenance/before/frozen_source_manifest.json').read_text())['members']
        new = json.loads((self.target / 'provenance/after/frozen_source_manifest.json').read_text())['members']
        differences = {name for name in old.keys() | new.keys() if old.get(name) != new.get(name)}
        self.assertEqual(len(differences), 7)
        for name, expected in new.items():
            self.assertEqual(digest((WORKSPACE / name).read_bytes()), expected, name)
        self.assertEqual(self.reports[1]['verified_sources'], self.reports[2]['verified_sources'])

    def test_native_stage_order_and_exact_partition(self):
        counterexamples = json.loads((self.target / 'native_boundary_counterexamples.json').read_text())
        for group in counterexamples:
            for row in group['timing']:
                self.assertLessEqual(row['ready_mono_ns'], row['take_begin_mono_ns'])
                self.assertLessEqual(row['take_begin_mono_ns'], row['handle_entered_mono_ns'])
                self.assertLessEqual(row['handle_entered_mono_ns'], row['take_mono_ns'])
                self.assertLessEqual(row['take_mono_ns'], row['callback_mono_ns'])
                self.assertLessEqual(row['callback_mono_ns'], row['end_mono_ns'])
                m = row['metadata']
                stages = ((m['received_timestamp'] - row['issued_wall_ns'])
                          + (row['ready_wall_ns'] - m['received_timestamp'])
                          + (row['take_wall_ns'] - row['ready_wall_ns'])
                          + (row['callback_wall_ns'] - row['take_wall_ns']))
                self.assertEqual(stages, row['callback_wall_ns'] - row['issued_wall_ns'])
                matched = [r for r in group['udp'] if r['sequence'] == row['sequence']
                           and r['issued_wall_ns'] == row['issued_wall_ns']]
                self.assertTrue(matched)
                self.assertLessEqual(min(r['kernel_wall_ns'] for r in matched), row['callback_wall_ns'])

    def test_first_dispatched_path_is_the_status_path(self):
        rows = json.loads((self.target / 'initial_public_snapshots.json').read_text())
        for row in rows:
            path = row['maps']['/wildos/planned_path']
            self.assertLess(math.dist(path['pose_xy'][-1], row['first_planned']['goal']), 1e-5)
            self.assertEqual(path['points'], row['first_planned']['poses'])
            self.assertEqual(path['frame'], 'spot1/odom')
            for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
                grid = row['maps'][topic]
                self.assertEqual(sum(grid['cost_counts'].values()), grid['width'] * grid['height'])

    def test_evidence_scope(self):
        self.assertFalse(self.manifest['production_changes'])
        self.assertFalse(self.manifest['complete_navigation_passed'])
        self.assertTrue(self.manifest['all_original_bag_hashes_match_previous_evidence'])


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(EvidenceChecks))
    (ROOT / 'local_verification.json').write_text(json.dumps(dict(
        passed=result.wasSuccessful(), checks=result.testsRun, failures=len(result.failures),
        errors=len(result.errors), production_changes=False, complete_navigation_passed=False,
        archive_sha256=ARCHIVE_SHA), indent=2) + '\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
