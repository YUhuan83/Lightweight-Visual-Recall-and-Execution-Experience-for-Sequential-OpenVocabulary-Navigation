"""Summarize route, bridge, control and measurement topics from a Town trial bag."""
from pathlib import Path
import json
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root = Path('/home/ly/town_memory_validation_20261003') / sys.argv[1] / 'rosbag'
db = sqlite3.connect(next(root.glob('*.db3')))
topics = {name: (topic_type, ident) for ident, name, topic_type in
          db.execute('SELECT id,name,type FROM topics')}
start_ns = db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]

def read(name):
    if name not in topics:
        return []
    topic_type, ident = topics[name]
    cls = get_message(topic_type)
    return [(ts, deserialize_message(raw, cls)) for ts, raw in
            db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (ident,))]

result = {'topic_counts': {name: db.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?',
                                             (ident,)).fetchone()[0]
                           for name, (_, ident) in topics.items()}}
for name in ('/spot1/graphnav_planner/route_status', '/wildos/plan_status',
             '/wildos/execution_feedback', '/wildos/control_status'):
    rows = read(name)
    transitions = []
    last = None
    for ts, msg in rows:
        try:
            value = json.loads(msg.data)
        except Exception:
            value = msg.data
        if name.endswith('/route_status') and isinstance(value, dict):
            key = json.dumps({k: value.get(k) for k in ('goal', 'complete', 'mode')}, sort_keys=True)
        else:
            key = json.dumps(value, sort_keys=True) if isinstance(value, dict) else value
        if key != last:
            transitions.append({'elapsed': round((ts-start_ns)*1e-9, 3), 'value': value})
            last = key
    result[name] = (transitions[:3] + ([{'omitted_transitions': len(transitions)-6}]
                     if len(transitions) > 6 else []) + transitions[-3:])

paths = read('/spot1/graphnav_planner/path')
path_summaries = [{'elapsed': round((ts-start_ns)*1e-9, 3),
                             'header_age_sec': round(ts*1e-9 -
                                 (msg.header.stamp.sec*1e9 + msg.header.stamp.nanosec)*1e-9, 3),
                             'poses': len(msg.poses),
                             'start': [msg.poses[0].pose.position.x, msg.poses[0].pose.position.y]
                                     if msg.poses else None,
                             'end': [msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y]
                                   if msg.poses else None}
                            for ts, msg in paths]
result['graphnav_paths'] = {'count': len(paths), 'first': path_summaries[:3], 'last': path_summaries[-3:]}

cmds = read('/wildos/cmd_vel')
active = [(round((ts-start_ns)*1e-9, 3), msg.linear.x, msg.angular.z)
          for ts, msg in cmds if abs(msg.linear.x) > .01 or abs(msg.angular.z) > .01]
result['cmd_vel'] = {'count': len(cmds), 'active_count': len(active), 'active_examples': active[:30]}

for name in ('/spot1/odom', '/global_costmap/costmap'):
    rows = read(name)
    if rows:
        receipts = np.array([ts for ts, _ in rows], dtype=np.float64)*1e-9
        ages = np.array([(ts*1e-9 - (msg.header.stamp.sec*1e9 + msg.header.stamp.nanosec)*1e-9)
                         for ts, msg in rows])
        result[name] = {'count': len(rows),
                        'receipt_gap_p50_p95_max': np.percentile(np.diff(receipts), [50, 95, 100]).tolist(),
                        'header_age_p50_p95_max': np.percentile(ages, [50, 95, 100]).tolist()}

print(json.dumps(result, indent=2))
