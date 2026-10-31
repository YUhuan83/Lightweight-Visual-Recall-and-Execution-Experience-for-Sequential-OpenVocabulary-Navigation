import json,sqlite3,zipfile
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
base=Path('/home/ly/execution_continuity_review_20261022')
root=base/'frontier_endpoint_contract_review'
receipt=json.loads((base/'v54_confirmed_graph_handoff/field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes']
session=next(Path(receipt['trial']).glob('sessions/*'));goal=session/'goal_001'
events=[json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines()]
start=next(e for e in events if e.get('status')=='object_standoff_approach')
end=next(e for e in events if e.get('status')=='target_instance_superseded')
stamp=start['replay_inputs']['odom_stamp_ns']
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topic=db.execute("select id from topics where name='/spot1/odom'").fetchone()[0]
odom=None
for data, in db.execute('select data from messages where topic_id=?',(topic,)):
    m=deserialize_message(data,Odometry)
    if m.header.stamp.sec*10**9+m.header.stamp.nanosec==stamp:odom=m;break
assert odom is not None
db.close()
selected=[]
for line in (session/'graph_pairs/snapshots.jsonl').read_text().splitlines():
    r=json.loads(line)
    if r['kind']!='pair' or 'raw' not in r or not all(r['contract'].values()):continue
    # Actual reception-time slice, same confirmed destination before replacement.
    if not start['stamp_ns']<=r['ros_ns']<end['stamp_ns']:continue
    selected.append(r)
with (root/'recorded_graphs.txt').open('w') as f:
    for r in selected:
        raw=session/'graph_pairs'/r['raw']['file'];scored=session/'graph_pairs'/r['scored']['file']
        f.write(f"{raw} {scored} {start['goal'][0]} {start['goal'][1]} {odom.pose.pose.position.z} {r['ros_ns']}\n")
(root/'replay_scope.json').write_text(json.dumps(dict(cases=len(selected),goal=start['goal'],goal_z=odom.pose.pose.position.z,source='recorded graph pairs, exact topology/score overlay; reception time approximates callback time; no vehicle counterfactual',snapshots=selected),indent=2))
with zipfile.ZipFile(base/'review_before.zip') as z:
    (root/'before_planner.cpp').write_bytes(z.read('candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner.cpp'))
print('recorded paired graphs:',len(selected))
