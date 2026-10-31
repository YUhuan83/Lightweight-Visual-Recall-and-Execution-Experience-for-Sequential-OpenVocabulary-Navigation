import json
import math
from pathlib import Path
import unittest
from wildos_nav2_adapter.tracking_preview import preview_tracking


class TrackingPreviewTests(unittest.TestCase):
    def test_real_submitted_section_rejects_actual_start_and_matches_cpp_failure(self):
        case=json.loads((Path(__file__).parent/'fixtures/town_first_cusp_tracking_20261014.json').read_text())
        actual=case['actual_start_pose']
        result=preview_tracking(case['points'],case['headings'],actual[:2],actual[2],8.)
        self.assertFalse(result.reached)
        self.assertEqual(result.reason,'tracking_preview_would_reverse')
        self.assertAlmostEqual(result.closest_m,.412002,delta=.02)

    def test_lower_curvature_control_keeps_same_physical_bound_and_strict_checker(self):
        case=json.loads((Path(__file__).parent/'fixtures/town_first_cusp_tracking_20261014.json').read_text())
        anchor=case['points'][0]
        points=[(anchor[0]+1.5*(p[0]-anchor[0]),anchor[1]+1.5*(p[1]-anchor[1])) for p in case['points']]
        actual=case['actual_start_pose']
        result=preview_tracking(points,case['headings'],actual[:2],actual[2],8.)
        self.assertTrue(result.reached)
        self.assertLessEqual(math.dist(result.trajectory[-1][:2],points[-1]),.2)

    def test_forward_reverse_and_rigid_frames_preserve_tracking(self):
        for direction in (-1,1):
            for angle in (0.,1.3,-2.9):
                transform=lambda p:(20+p[0]*math.cos(angle)-p[1]*math.sin(angle),-8+p[0]*math.sin(angle)+p[1]*math.cos(angle))
                points=[transform((direction*s,0.)) for s in (0.,2.,4.,6.)]
                result=preview_tracking(points,[angle]*4,transform((0.,.02)),angle+.01,8.)
                self.assertTrue(result.reached)
                for a,b in zip(result.trajectory,result.trajectory[1:]):
                    self.assertLessEqual(abs(b[2]-a[2])/math.dist(a[:2],b[:2]),.12501)

    def test_endpoint_behind_is_not_success_when_outside_capture_disk(self):
        result=preview_tracking([(-3.,-.205),(-.01,-.205)],[0.,0.],(0.,0.),0.,8.)
        self.assertFalse(result.reached)
        self.assertEqual(result.reason,'tracking_preview_would_reverse')

    def test_heading_is_required_and_steps_are_bounded(self):
        result=preview_tracking([(0.,0.),(2.,0.)],[0.,1.],(0.,0.),0.,8.)
        self.assertFalse(result.reached)
        result=preview_tracking([(2.*i,0.) for i in range(11)],[0.]*11,(0.,0.),0.,8.,max_steps=2)
        self.assertEqual(result.reason,'tracking_preview_budget_exhausted')
        self.assertEqual(len(result.trajectory),3)

    def test_mixed_direction_and_nonfinite_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            preview_tracking([(0.,0.),(1.,0.),(0.,0.)],[0.,0.,0.],(0.,0.),0.,8.)
        with self.assertRaises(ValueError):
            preview_tracking([(0.,0.),(1.,0.)],[0.,0.],(0.,math.nan),0.,8.)


if __name__=='__main__':unittest.main()
