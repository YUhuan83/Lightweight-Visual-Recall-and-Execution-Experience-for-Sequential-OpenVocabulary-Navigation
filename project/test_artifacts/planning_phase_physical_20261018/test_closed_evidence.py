"""Evidence consistency only; passing cannot mean vehicle search acceptance."""
import hashlib,json,unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parent
CLOSED=ROOT/'closed_evidence'
SESSION=CLOSED/'field_results/session_20261018_151920'
def read(path):return json.loads(path.read_text(encoding='utf8'))

class ClosedEvidenceTests(unittest.TestCase):
    def test_failure_remains_failure_and_recorders_closed(self):
        driver=read(CLOSED/'field_yaw_left90_search_20261018_151916/result.json')
        self.assertFalse(driver['passed'])
        self.assertEqual(read(SESSION/'goal_001/result.json')['state'],'timeout')
        self.assertEqual(read(SESSION/'session_result.json')['errors'],[])

    def test_no_visual_arrival_is_inferred_from_graph_routes(self):
        summary=read(CLOSED/'field/baseline_summary.json')
        self.assertGreater(summary['complete_route_decisions'],0)
        self.assertEqual((summary['masks'],summary['localizations'],summary['confirmations']),(0,0,0))
        self.assertTrue(summary['memory_priors_disabled'])

    def test_stop_accounting_conserves_time(self):
        review=read(ROOT/'canonical_window_review.json')
        stopped=review['stopped_sec']
        self.assertAlmostEqual(sum(review['stopped_by_source_reason'].values()),stopped,places=6)
        self.assertAlmostEqual(sum(review['stopped_by_sent_control'].values()),stopped,places=6)
        self.assertLess(stopped,review['covered_sec'])

    def test_positive_applied_controls_have_correct_source_and_deadline(self):
        chain=read(ROOT/'canonical_window_review.json')
        self.assertEqual(chain['source_deadline_violations'],[])
        self.assertEqual(chain['unmatched_positive'],[])
        self.assertGreater(chain['positive_source_to_applied_age']['n'],0)

    def test_goal_window_uses_shared_python_monotonic(self):
        review=read(ROOT/'canonical_window_review.json')
        goal=read(SESSION/'goal_001/result.json')
        self.assertEqual(review['goal_started_monotonic'],goal['started'])
        self.assertAlmostEqual(review['goal_ended_monotonic']-goal['started'],goal['elapsed_s'],places=6)
        self.assertGreater(review['covered_sec'],420.)
        self.assertLess(review['covered_sec'],goal['elapsed_s'])

    def test_console_preparation_is_not_included_in_goal_window(self):
        review=read(ROOT/'canonical_window_review.json')
        self.assertGreater(review['console_input_to_goal_start_sec'],5.)
        events=[json.loads(l) for l in (SESSION/'goal_001/events.jsonl').read_text().splitlines()]
        first=next(e['stamp_ns']/1e9 for e in events if e.get('kind')=='plan_status')
        self.assertLess(abs(first-review['goal_started_wall']),.2)

    def test_original_physics_and_sustained_stop(self):
        stop=read(SESSION/'goal_001/stop.json')
        self.assertEqual(stop['physics']['nodeCount'],723)
        self.assertEqual(stop['physics']['bodyCollisionNodeCount'],467)
        self.assertEqual([w['collisionNodeCount'] for w in stop['physics']['wheels']],[32]*4)
        self.assertGreater(stop['samples'][-1]['elapsed'],.5)
        self.assertTrue(all(s['speed']<.05 for s in stop['samples']))

    def test_current_failure_video_identity(self):
        video=ROOT.parents[1]/'captures/20261018_planning_phase_source/rviz_live.mp4'
        report=read(ROOT/'video_verification.json')
        self.assertEqual(hashlib.sha256(video.read_bytes()).hexdigest(),report['sha256'])
        self.assertEqual(report['decode_returncode'],0)
        self.assertGreater(float(report['probe']['format']['duration']),420.)

if __name__=='__main__':unittest.main(verbosity=2)
