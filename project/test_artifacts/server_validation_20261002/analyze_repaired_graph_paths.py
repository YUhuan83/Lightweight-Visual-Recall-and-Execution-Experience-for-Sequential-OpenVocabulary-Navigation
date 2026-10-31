"""Check graph endpoints in the finished repaired trial; read-only bag access."""
from pathlib import Path
import json
import math
import sqlite3
import subprocess
import sys
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, *sys.argv[1:], '--runtime-child'],
                                   env=runtime_env()).returncode)
from nav_msgs.msg import Path as Route
from rclpy.serialization import deserialize_message
name = sys.argv[1] if sys.argv[1:] and sys.argv[1] != '--runtime-child' else 'navigation_repair_trial1'
folder = ROOT/name
audit = json.loads((ROOT/'failure_audit'/name/'audit.json').read_text())
base = audit['trial_base_wall_time']
db = sqlite3.connect('file:'+str(next((folder/'rosbag').glob('*.db3')))+'?mode=ro', uri=True)
ident = db.execute('select id from topics where name=?', ('/spot1/graphnav_planner/path',)).fetchone()[0]
case = json.loads((folder/'full_test.json').read_text())
first_follow = next(e for e in case['events'] if e['kind']=='plan_status'
                    and e['phase']=='station_return' and e['status']=='following')
goal = first_follow['graph_goal']
endpoints = []
for stamp, data in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? order by timestamp',
                              (ident, int((base+first_follow['elapsed'])*1e9))):
    path = deserialize_message(data, Route)
    if path.poses:
        point = path.poses[-1].pose.position
        endpoints.append([point.x, point.y])
result = dict(scope='Recorded actual return stage after FollowPath began', outputs=len(endpoints),
              distinct_endpoints=sorted(set(map(tuple, endpoints))), goal=goal,
              all_endpoints_keep_memory_goal=bool(endpoints) and all(math.dist(p, goal) < .01 for p in endpoints))
assert result['all_endpoints_keep_memory_goal'], result
(folder/'graph_goal_retention.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
db.close()
