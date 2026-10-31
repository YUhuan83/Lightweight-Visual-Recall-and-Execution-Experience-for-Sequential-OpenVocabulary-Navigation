"""Synthetic metric/unknown-data checks; no production code or ROS involved."""
import importlib.util
import json
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('analyzer', ROOT / 'analyze_closed_trial.py')
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def encode(text, little=True):
    value = text.encode() + b'\0'
    return (b'\0\1\0\0' if little else b'\0\0\0\0') + struct.pack('<I' if little else '>I', len(value)) + value


class AnalyzerTests(unittest.TestCase):
    def test_cdr_endianness_and_bounds(self):
        for little in (True, False):
            self.assertEqual(MOD.cdr_string(encode('中文 String', little)), '中文 String')
        with self.assertRaises(ValueError):
            MOD.cdr_string(encode('x')[:-1])

    def fixture(self, root, native=True):
        q0 = 1_700_000_000
        db = sqlite3.connect(root / 'interfaces_0.db3')
        db.execute('create table topics (id integer,name text,type text)')
        db.execute('create table messages (topic_id integer,timestamp integer,data blob)')
        names = ('/spot1/object_search/query', '/wildos/plan_status', '/wildos/control_status',
                 '/wildos/native_control_status', '/spot1/visible_target_approach')
        for i, name in enumerate(names):
            db.execute('insert into topics values (?,?,?)', (i, name, 'std_msgs/msg/String'))
        def put(topic, t, data):
            text = data if isinstance(data, str) else json.dumps(data)
            db.execute('insert into messages values (?,?,?)', (topic, int((q0+t)*1e9), encode(text)))
        put(0, 0, 'green residential house')
        for t, status, goal in ((0, 'planning', None), (3, 'following', None),
                                (4, 'visible_target_no_vehicle_path', [10, 10]),
                                (4.2, 'visible_target_approach', [13, 10]), (4.3, 'following', [13, 10])):
            put(1, t, dict(stamp_ns=int((q0+t)*1e9), query='green residential house', status=status, goal=goal))
        put(2, 0, dict(stamp_ns=int(q0*1e9), reason='tracking'))
        put(4, 4.1, dict(query='green residential house', point=[13, 10], observation_stamp_ns=int((q0+3)*1e9)))
        if native:
            for i in range(40):
                t = i / 4
                put(3, t, dict(wall_time=q0+t, speed=0 if t<2 or 6<=t<8 else 1,
                               position=[t, 0], telemetry_ok=True, source_rejection=None, sent=dict(gear=2)))
        db.commit()
        db.close()

    def analyze(self, root):
        subprocess.run([sys.executable, str(ROOT / 'analyze_closed_trial.py'), str(root), str(root/'out'), '--window', '10'], check=True, capture_output=True)
        return json.loads((root/'out/closed_trial_summary.json').read_text(encoding='utf8'))

    def test_source_integration_and_failed_observation_alternative(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
            path = Path(tmp)
            self.fixture(path)
            data = self.analyze(path)
        self.assertAlmostEqual(data['native']['covered_s'], 10)
        self.assertAlmostEqual(data['near_stop']['seconds'], 4)
        self.assertAlmostEqual(data['near_stop']['fraction_of_covered'], .4)
        self.assertAlmostEqual(data['near_stop']['longest_s'], 2)
        self.assertAlmostEqual(sum(data['plan_status_dwell_s'].values()), 10)
        self.assertAlmostEqual(data['movement']['net_displacement_m'], 9.75)
        failure = data['observation_failure_chains'][0]
        self.assertTrue(failure['proposals'][0]['different_outside_rejection'])
        self.assertEqual(failure['proposals'][0]['observation_stamp_ns'], 1_700_000_003_000_000_000)
        self.assertEqual(len(failure['subsequent_other_goal_states']), 2)

    def test_missing_native_does_not_report_zero_stop_or_motion(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
            path = Path(tmp)
            self.fixture(path, native=False)
            data = self.analyze(path)
        self.assertIsNone(data['near_stop']['seconds'])
        self.assertIsNone(data['near_stop']['fraction_of_window'])
        self.assertIsNone(data['movement'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
