from pathlib import Path
import hashlib,json,unittest

HERE=Path(__file__).parent
ROOT=HERE.parents[1]
def read(name):return json.loads((HERE/name).read_text(encoding='utf-8-sig'))

class FieldEvidenceTests(unittest.TestCase):
    def test_runtime_candidate_sources_remain_frozen(self):
        manifest=json.loads((ROOT/'deploy/20261018_motion_handoff_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
        self.assertEqual(manifest['sha256'],read('freeze.json')['candidate'])
        self.assertEqual(len(manifest['members']),401)
        for name,want in manifest['members'].items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),want,name)
        self.assertTrue(read('prelaunch_verification.json')['harness_verified'])

    def test_static_time_attribution_matches_independent_integration(self):
        trace=read('closed_trace.json');motion=read('motion_metrics.json')
        total=motion['below_speed_s']['0.05']
        self.assertAlmostEqual(sum(trace['stopped_seconds_by_source_reason'].values()),total,places=7)
        self.assertAlmostEqual(sum(trace['stopped_seconds_by_sent_control'].values()),total,places=7)
        self.assertEqual(trace['stopped_samples'],trace['stopped_applied_identity_matches'])

    def test_no_claim_of_moving_replacement_or_arrival_success(self):
        review=read('closed_handoff_review.json');baseline=read('baseline_summary.json')
        leases=[x for x in review['handoff_leases'] if x['long_section']]
        self.assertEqual(len(leases),6)
        self.assertEqual(review['observed_new_following_after_retirement'],0)
        self.assertTrue(all(x['outcome']['reason']=='motion_lease_exhausted' for x in leases))
        geometry=read('closed_extension_geometry.json')
        paths=[p for window in geometry for p in window['full_paths']]
        self.assertEqual(len(paths),10)
        self.assertTrue(all(p['sections']>1 for p in paths))
        self.assertEqual(baseline['goal']['state'],'timeout')
        self.assertFalse(baseline['goal']['passed'])
        self.assertEqual(read('pursuit_closed_review.json')['confirmations'],[])

    def test_shutdown_and_sensors_preserved(self):
        session=HERE/'closed_evidence/field_results/session_20261018_083258'
        stop=json.loads((session/'goal_001/stop.json').read_text(encoding='utf8'))
        self.assertLess(stop['physics']['speed'],.01)
        self.assertEqual(stop['physics']['bodyCollisionNodeCount'],467)
        self.assertTrue(all(x['collisionNodeCount']==32 for x in stop['physics']['wheels']))
        self.assertFalse(stop['physics']['configKinematic'])
        self.assertEqual(read('postrun_probe.json')['relevant_processes'],[])
        self.assertEqual(json.loads((session/'session_result.json').read_text(encoding='utf8'))['errors'],[])

    def test_returned_video_and_core_archive_match_verified_identities(self):
        video=read('video_verification.json')
        self.assertEqual(video['decode_returncode'],0)
        self.assertEqual(hashlib.sha256((ROOT/'captures/20261018_motion_handoff/rviz_live.mp4').read_bytes()).hexdigest(),video['sha256'])
        self.assertEqual(read('local_validation.json')['sha256'],read('field_archive_identity.json')['sha256'])
        self.assertEqual(read('local_validation.json')['verified_payloads'],125)

if __name__=='__main__':unittest.main(verbosity=2)
