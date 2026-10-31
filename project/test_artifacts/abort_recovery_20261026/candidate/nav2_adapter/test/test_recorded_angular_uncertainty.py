"""Recorded unstable intersections must not become strong target positions."""
import json
from pathlib import Path
import unittest
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import triangulate_bearings

class RecordedAngularUncertaintyTests(unittest.TestCase):
    def test_all_eight_recorded_small_baseline_positions_remain_unconfirmed(self):
        cases=json.loads((Path(__file__).parent/'fixtures/v32_multiview_uncertainty.json').read_text())
        self.assertEqual(len(cases),8)
        for row in cases:
            with self.subTest(stamp=row['stamp_ns']):
                views=[dict(origin=np.asarray(v['origin']),ray=np.asarray(v['ray']),angular_tolerance=v['angular_tolerance']) for v in row['views']]
                self.assertIsNone(triangulate_bearings(views,1.))

    def test_well_separated_views_still_localize_an_object(self):
        target=np.asarray([10.,2.,3.])
        origins=[np.asarray(p,dtype=float) for p in ((0,0,0),(5,0,0),(0,5,0))]
        result=triangulate_bearings([dict(origin=o,ray=target-o,angular_tolerance=.04) for o in origins],1.)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['position'],target,atol=1e-8)
        self.assertLessEqual(result['radius'],3.)
