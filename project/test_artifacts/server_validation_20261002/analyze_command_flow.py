"""Compare recorded command deliveries with the native receiver's watchdog."""
from pathlib import Path
import bisect
import json
import sqlite3
import subprocess
import sys
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, '--runtime-child'],
                                   env=runtime_env()).returncode)
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
folder = ROOT/'navigation_repair_trial1'
audit = json.loads((ROOT/'failure_audit/navigation_repair_trial1/audit.json').read_text())
base = audit['trial_base_wall_time']
case = json.loads((folder/'full_test.json').read_text())
statuses = [e for e in case['events'] if e['kind']=='plan_status']
times = [e['elapsed'] for e in statuses]
db = sqlite3.connect('file:'+str(next((folder/'rosbag').glob('*.db3')))+'?mode=ro', uri=True)
ids = dict(db.execute('select name,id from topics'))
command_times = [t/1e9 for (t,) in db.execute('select timestamp from messages where topic_id=? order by timestamp',
                                            (ids['/wildos/native_control'],))]
examples = []
ages, following_count = [], 0
for stamp, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                              (ids['/wildos/native_control_status'],)):
    n = json.loads(deserialize_message(data, String).data)
    # Receiver measures command_age at tick start, before blocking RPC.
    started = n['wall_time']-n['roundtrip_ms']/1000
    index = bisect.bisect_right(times, started-base)-1
    if index < 0 or statuses[index]['status'] != 'following' or not n['watchdog_stop']:
        continue
    following_count += 1
    j = bisect.bisect_right(command_times, started)-1
    if j < 0:
        continue
    bag_age = started-command_times[j]
    ages.append(bag_age)
    if len(examples) < 5:
        examples.append(dict(elapsed=started-base, native_command_age=n['command_age'],
                             latest_recorded_command_age_approx=bag_age, roundtrip_ms=n['roundtrip_ms']))
result = dict(command_messages=len(command_times),
              command_gap_seconds_p50_p95_max=np.percentile(np.diff(command_times), [50,95,100]).tolist(),
              watchdog_samples_while_following=following_count,
              watchdog_with_recent_recorded_command_under_100ms=sum(a < .1 for a in ages),
              latest_recorded_command_age_p50_p95_max=np.percentile(ages, [50,95,100]).tolist(),
              examples=examples,
              limitation='Recorder reception is not native callback reception; tick-start wall time inferred from RPC duration. This localizes receiver-side delay but does not prove a unique scheduler/DDS cause.')
(folder/'command_flow_analysis.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
db.close()
