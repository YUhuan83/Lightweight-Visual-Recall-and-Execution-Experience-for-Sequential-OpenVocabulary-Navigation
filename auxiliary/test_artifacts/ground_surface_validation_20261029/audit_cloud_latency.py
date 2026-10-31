"""Compare matched cloud receipt times in closed trials, never active bags."""
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def summary(values):
    return dict(count=len(values), p50=float(np.quantile(values, .5)),
                p95=float(np.quantile(values, .95)), maximum=float(max(values))) if values else None


def audit(trial):
    assert json.loads((trial / 'result.json').read_text())['closed']
    goal = next(trial.glob('sessions/*/goal_001'))
    events = [json.loads(line) for line in (goal / 'events.jsonl').read_text().splitlines()]
    query = next(e['elapsed'] for e in events if e.get('kind') == 'query_sent')
    timed = next(e for e in events if e.get('stamp_ns'))
    origin = timed['stamp_ns'] - round((timed['elapsed'] - query) * 1.e9)
    db = sqlite3.connect(next((goal / 'interfaces').glob('*.db3')).as_uri() + '?mode=ro', uri=True)
    topics = dict((name, ident) for ident, name in db.execute('select id,name from topics'))
    raw = '/spot1/ouster/front/points_filtered'
    names = [raw, '/wildos/points', '/wildos/clearing_points']
    streams = {}
    for name in names:
        rows = []
        for received, payload in db.execute(
                'select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                (topics[name], origin, origin + 180_000_000_000)):
            msg = deserialize_message(payload, PointCloud2)
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            rows.append((received, stamp, int(msg.width * msg.height)))
        streams[name] = rows
    raw_times = {stamp: received for received, stamp, _ in streams[raw]}
    result = {'trial': str(trial), 'scope': 'Matched source header stamps; bag receipt lag includes scheduling and DDS, not isolated CPU time.', 'topics': {}}
    for name, rows in streams.items():
        receipt = [r[0] for r in rows]
        intervals = [(b-a)*1.e-9 for a,b in zip(receipt, receipt[1:])]
        lag = [(received-raw_times[stamp])*1.e-9 for received,stamp,_ in rows if stamp in raw_times]
        result['topics'][name] = dict(messages=len(rows), hz=(len(rows)-1)*1.e9/(receipt[-1]-receipt[0]) if len(rows)>1 else None,
                                     intervals_s=summary(intervals), matched_raw_receipt_lag_s=summary(lag), points=summary([r[2] for r in rows]))
    db.close()
    (trial / 'cloud_latency_audit.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    print(json.dumps([audit(Path(arg)) for arg in sys.argv[1:]], indent=2))
