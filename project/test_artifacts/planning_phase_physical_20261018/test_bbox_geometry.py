from pathlib import Path
import sys,json,unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'official_adapter'),str(Path(__file__).parent)]
from bbox_geometry import rear_axle_extents

class BBoxGeometryTests(unittest.TestCase):
    def test_historical_engine_box_and_list_valued_native_vectors(self):
        fixture=json.loads((ROOT/'test_artifacts/server_validation_20261002/wheel_geometry_probe.json').read_text(encoding='utf8'))
        state=fixture['state'];local,extents=rear_axle_extents(list(fixture['bbox'].values()),
            state['pos'],state['dir'],state['up'],[-2.172,.330,0.])
        self.assertEqual(local.shape,(8,3))
        self.assertGreater(extents['front'],3.2);self.assertLess(extents['front'],3.4)
        self.assertGreater(extents['rear'],.9);self.assertLess(extents['rear'],1.05)
        self.assertGreater(extents['half_width'],.95);self.assertLess(extents['half_width'],1.05)

    def test_invalid_corner_shape_is_explicitly_rejected(self):
        with self.assertRaises(ValueError):
            rear_axle_extents([[0.,0.,0.]]*7,[0.,0.,0.],[1.,0.,0.],[0.,0.,1.],[0.,0.,0.])

if __name__=='__main__':unittest.main()
