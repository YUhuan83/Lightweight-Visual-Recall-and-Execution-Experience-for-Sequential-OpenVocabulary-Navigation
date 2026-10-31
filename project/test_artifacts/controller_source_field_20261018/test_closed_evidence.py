"""Evidence consistency gates; passing does not mean navigation succeeded."""
from pathlib import Path
import hashlib,json,unittest
ROOT=Path(__file__).parent;CLOSED=ROOT/'closed_evidence'
def read(path):return json.loads(path.read_text(encoding='utf8'))

class ClosedEvidenceTests(unittest.TestCase):
    def test_frozen_field_and_current_component_are_distinct(self):
        freeze=read(ROOT/'freeze.json');archive=read(CLOSED/'field_evidence_manifest.json')
        self.assertEqual(archive['candidate'],freeze['candidate'])
        self.assertNotEqual(freeze['candidate'],read(ROOT.parent/'controller_deadline_clock_20261018/freeze.json')['candidate'])
        self.assertEqual(len(read(CLOSED/'manifest.json')['members']),403)

    def test_source_deadline_and_identity_observations(self):
        data=read(CLOSED/'controller_chain_review.json')
        self.assertGreater(data['positive_source_to_applied_age_sec']['count'],1000)
        for key in ('source_deadline_violations','wrong_controller_frames','unmatched_positive'):
            self.assertEqual(data[key],[],key)
        self.assertLess(data['positive_source_to_applied_age_sec']['max'],.5)

    def test_stop_time_accounts_for_full_passive_window(self):
        trace=read(ROOT/'closed_trace.json');motion=read(CLOSED/'motion_metrics.json')
        for key in ('stopped_seconds_by_source_reason','stopped_seconds_by_sent_control'):
            self.assertAlmostEqual(sum(trace[key].values()),motion['below_speed_s']['0.05'],places=6)
        self.assertEqual(trace['stopped_samples'],trace['stopped_applied_identity_matches'])

    def test_next_following_does_not_prove_moving_handoff(self):
        report=read(ROOT/'verified_handoff_review.json');stopped,moving=report['checks']
        self.assertEqual(report['old_next_following_count'],2)
        self.assertEqual(report['verified_moving_replacements'],1)
        self.assertFalse(stopped['verified_moving_replacement'])
        self.assertTrue(stopped['cancel_times'] and stopped['terminal_zero_times'])
        self.assertLess(stopped['minimum_native_speed'],.05)
        self.assertTrue(moving['verified_moving_replacement'])
        self.assertTrue(moving['controller_preempt_times'])
        self.assertGreater(moving['minimum_native_speed'],.05)

    def test_physical_stop_and_video_belong_to_failed_trial(self):
        summary=read(CLOSED/'field/baseline_summary.json')
        self.assertEqual(summary['goal']['state'],'timeout');self.assertFalse(summary['goal']['passed'])
        session=next((CLOSED/'field_results').glob('session_*'))
        self.assertEqual(read(session/'session_result.json')['errors'],[])
        stop=read(session/'goal_001/stop.json')
        self.assertLess(stop['samples'][-1]['speed'],.05)
        physics=stop['physics']
        self.assertFalse(physics['configKinematic'] or physics['configBodyDisabled'] or physics['collisionOverrideLoaded'])
        self.assertEqual(physics['bodyCollisionNodeCount'],467)
        self.assertTrue(all(w['collisionNodeCount']==32 for w in physics['wheels']))
        video=read(CLOSED/'video_verification.json')
        self.assertEqual(video['decode_returncode'],0)
        path=ROOT.parents[1]/'captures/20261018_controller_source/rviz_live.mp4'
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),video['sha256'])

if __name__=='__main__':unittest.main()
