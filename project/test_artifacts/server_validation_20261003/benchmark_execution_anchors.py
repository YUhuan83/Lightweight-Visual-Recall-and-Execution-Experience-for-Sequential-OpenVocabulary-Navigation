"""Compare recorded DDS payloads and decoding cost for execution feedback."""
from pathlib import Path
import json
import sqlite3
import sys
import time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/ly/town_memory_validation_20261003')
out=ROOT/sys.argv[1]
db=sqlite3.connect(f"file:{next((out/'rosbag').glob('*.db3'))}?mode=ro",uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('SELECT id,name,type FROM topics')}
result={}
for name in ('/spot1/scored_nav_graph','/spot1/execution_anchor_graph'):
    identifier,kind=topics[name]
    samples=[]
    for t,b in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 3',(identifier,)):
        start=time.perf_counter()
        msg=deserialize_message(b,kind)
        samples.append({'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                        'bytes':len(b),'nodes':len(msg.nodes),'edges':len(msg.edges),
                        'decode_ms':(time.perf_counter()-start)*1000})
    result[name]=samples
assert all(s['edges']==0 for s in result['/spot1/execution_anchor_graph'])
(out/'execution_anchor_benchmark.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
