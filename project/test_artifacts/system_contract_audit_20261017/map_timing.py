"""Read only closed ROS records, measure published-map timing (not private cache)."""
from pathlib import Path
import bisect
import json
import sqlite3
import statistics
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root = Path('/home/ly/full_vehicle_acceptance_20261017')
driver = sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (driver/'result.json').is_file()
trial = json.loads((driver/'result.json').read_text())
session = Path(trial['session'])
goal = session/'goal_001'
assert (session/'session_result.json').exists() and (goal/'result.json').exists()
lo = next(c['wall'] for c in trial['commands'] if c['text'] == 'green residential house')
hi = lo + json.loads((goal/'result.json').read_text())['elapsed_s']
events = [json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines() if l]
waits = [x['stamp_ns']/1e9 for x in events if x.get('status') == 'waiting_for_costmap_update']
database = next((goal/'interfaces').glob('*.db3'))


def stats(values):
    values = sorted(values)
    return dict(count=len(values), median=statistics.median(values),
                p95=values[int(.95*(len(values)-1))], max=max(values)) if values else None


maps = {}
with sqlite3.connect(database.as_uri()+'?mode=ro', uri=True) as connection:
    for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
        pair = connection.execute('select id,type from topics where name=?', (topic,)).fetchone()
        if not pair:
            continue
        cls = get_message(pair[1])
        rows = []
        for timestamp, data in connection.execute('select timestamp,data from messages where topic_id=? order by timestamp', (pair[0],)):
            t = timestamp/1e9
            if not lo-5 <= t <= hi:
                continue
            message = deserialize_message(data, cls)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
            rows.append((t, stamp))
        increasing, previous = [], 0.
        for row in rows:
            if row[1] > previous:
                increasing.append(row)
                previous = row[1]
        receipt_times = [x[0] for x in rows]
        new_times = [x[0] for x in increasing]
        ages = []
        for t in waits:
            i = bisect.bisect_right(receipt_times, t)-1
            j = bisect.bisect_right(new_times, t)-1
            ages.append(dict(wait_wall=t, latest_publish_receipt_age=t-receipt_times[i] if i >= 0 else None,
                             latest_increasing_stamp_receipt_age=t-new_times[j] if j >= 0 else None))
        maps[topic] = dict(received=len(rows), increasing=len(increasing),
            receipt_gaps=stats([b[0]-a[0] for a, b in zip(rows, rows[1:])]),
            acquisition_ages=stats([r-s for r, s in rows]),
            increasing_receipt_gaps=stats([b[0]-a[0] for a, b in zip(increasing, increasing[1:])]),
            at_bridge_waits=ages)
output = dict(scope='Closed public-map recorder receipts; not Bridge callback entry or plugin private cache; '
                   'wait status may persist over a planner search.', maps=maps)
print(json.dumps(output, indent=2))
