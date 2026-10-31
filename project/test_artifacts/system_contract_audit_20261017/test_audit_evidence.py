"""Evidence accounting gates; these are not navigation success tests."""
from pathlib import Path
import hashlib
import json
import unittest

HERE = Path(__file__).parent
ROOT = HERE.parents[1]


class EvidenceTests(unittest.TestCase):
    def test_runtime_sources_are_still_frozen(self):
        manifest = json.loads((ROOT/'deploy/20261017_full_vehicle_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
        for name, digest in manifest['members'].items():
            with self.subTest(name=name):
                self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(), digest)

    def test_stop_partitions_conserve_previous_full_observer_integral(self):
        audit = json.loads((HERE/'closed_trace.json').read_text(encoding='utf8'))
        field = json.loads((ROOT/'test_artifacts/full_vehicle_acceptance_20261017/closed_evidence/motion_metrics.json').read_text(encoding='utf8'))
        total = field['below_speed_s']['0.05']
        self.assertAlmostEqual(sum(audit['stopped_seconds_by_source_reason'].values()), total, places=7)
        self.assertAlmostEqual(sum(audit['stopped_seconds_by_sent_control'].values()), total, places=7)
        self.assertAlmostEqual(sum(audit['stopped_cross_status_source_reason'].values()), total, places=7)

    def test_hidden_planner_failures_are_not_lost_in_terminal_count(self):
        data = json.loads((HERE/'planner_timing.json').read_text(encoding='utf8'))
        self.assertEqual(data['result_counts']['failed'], len(data['failures']))
        self.assertEqual(data['candidate_failure_reasons']['empty_path'], 44)
        self.assertEqual(len(data['attempts']), sum(data['result_counts'].values()))

    def test_map_report_is_bounded_to_closed_recording(self):
        data = json.loads((HERE/'map_timing.json').read_text(encoding='utf-8-sig'))
        self.assertEqual(set(data['maps']), {'/global_costmap/costmap', '/local_costmap/costmap'})
        for value in data['maps'].values():
            self.assertGreater(value['received'], 0)
            self.assertLessEqual(value['increasing'], value['received'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
