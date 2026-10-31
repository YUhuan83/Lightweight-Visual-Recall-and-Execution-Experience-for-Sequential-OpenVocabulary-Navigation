import json
from pathlib import Path
import tempfile
import unittest
from analyze_american_capture import analyze


class AnalyzeTests(unittest.TestCase):
    def test_imu_time_and_norm(self):
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp)
            (d/'metadata.json').write_text('{"status":"passed"}')
            rows=[dict(time=t,accRaw=[0,0,9.81],angVel=[0,0,0]) for t in (0,.01,.02,.02,.01)]
            (d/'imu_00.json').write_text(json.dumps(rows))
            r=analyze(d)['imu']
            self.assertEqual(r['duplicate_steps'],1)
            self.assertEqual(r['backward_steps'],1)
            self.assertAlmostEqual(r['median_dt_rate_hz'],100)
            self.assertAlmostEqual(r['acceleration_norm_m_s2']['mean'],9.81)
            self.assertFalse(json.loads((d/'diagnostic_report.json').read_text())['calibrated'])


if __name__=='__main__':
    unittest.main()
