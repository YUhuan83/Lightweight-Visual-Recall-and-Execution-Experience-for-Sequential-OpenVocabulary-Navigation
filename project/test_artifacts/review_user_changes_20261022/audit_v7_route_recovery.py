"""Read the closed v7 bag and snapshot the input to its repeated failures."""
from pathlib import Path
import json
import math
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

trial = Path(sys.argv[1]) if len(sys.argv)>1 else Path('/home/ly/execution_continuity_review_20261022/v7/full/trial_20261022_223039')
assert json.loads((trial / 'result.json').read_text())['closed']
goal = next(trial.glob('sessions/*/goal_001'))
out = trial / 'route_recovery_audit'
out.mkdir(exist_ok=True)
db = sqlite3.connect(next(goal.rglob('*.db3')).as_uri() + '?mode=ro', uri=True)
topics = {name: (tid, typ) for tid, name, typ in db.execute('select id,name,type from topics')}
events = [json.loads(s) for s in (goal / 'events.jsonl').read_text().splitlines()]
plans = [e for e in events if e.get('kind') == 'plan_status']
origin = plans[0]['stamp_ns'] / 1e9 - plans[0]['elapsed']

def read(name, ns):
    tid, typ = topics[name]
    row = db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1', (tid, ns)).fetchone()
    if row is None:
        return None, None
    return row[0], deserialize_message(row[1], get_message(typ))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def pose(p):
    return [p.position.x, p.position.y, yaw(p.orientation)]

report = []
for elapsed in ([float(v) for v in sys.argv[2:]] or [69.94, 77.43, 88.35, 91.89, 98.53, 102.03, 141.65]):
    ns = int((origin + elapsed)*1e9)
    entry = {'elapsed': elapsed}
    _, odom = read('/spot1/odom', ns)
    entry['rear_axle'] = pose(odom.pose.pose)
    _, route = read('/spot1/graphnav_planner/path', ns)
    entry['route'] = [pose(p.pose) for p in route.poses]
    _, metadata = read('/spot1/graphnav_planner/route_status', ns)
    entry['metadata'] = json.loads(metadata.data)
    _, grid = read('/global_costmap/costmap', ns)
    public = dict(width=grid.info.width, height=grid.info.height,
                  resolution=grid.info.resolution, origin=pose(grid.info.origin),
                  stamp_ns=grid.header.stamp.sec*10**9+grid.header.stamp.nanosec,
                  data=list(grid.data))
    (out / ('public_grid_%s.json' % elapsed)).write_text(json.dumps(public))
    for name in ('/spot1/visible_target_approach', '/spot1/object_localization', '/spot1/object_confirmation'):
        stamp, message = read(name, ns)
        entry[name] = {'bag_age_s': (ns-stamp)/1e9, 'data': json.loads(message.data)} if message is not None else None
    report.append(entry)
(out / 'snapshots.json').write_text(json.dumps(report, indent=2))

hints = []
for name in ('/spot1/visible_target_approach', '/spot1/object_localization', '/spot1/object_confirmation'):
    tid, typ = topics[name]
    rows = db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp', (tid, int((origin+65)*1e9), int((origin+89)*1e9)))
    for stamp, data in rows:
        hints.append(dict(topic=name, elapsed=stamp/1e9-origin, data=json.loads(deserialize_message(data,get_message(typ)).data)))
(out / 'hint_window.json').write_text(json.dumps(hints, indent=2))
print(json.dumps({'snapshots': [{k:v for k,v in e.items() if k not in ('route',)} | {'route_poses':len(e['route']), 'route_ends': e['route'][:2]+e['route'][-2:]} for e in report], 'hint_count':len(hints)}, indent=2))
