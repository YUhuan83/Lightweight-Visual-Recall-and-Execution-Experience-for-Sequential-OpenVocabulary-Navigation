"""Separate source delays from graph deserialization and bridge callback age."""
from pathlib import Path
import json
import sqlite3
import sys
import time
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/ly/town_memory_validation_20261003')
trial=ROOT/sys.argv[1]
db=sqlite3.connect(f"file:{next((trial/'rosbag').glob('*.db3'))}?mode=ro",uri=True)
topics={name:(identifier,get_message(kind)) for identifier,name,kind in db.execute('SELECT id,name,type FROM topics')}
result={}
for name in ('/spot1/odom','/global_costmap/costmap'):
    identifier,kind=topics[name]
    samples=[]
    for t,b in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(identifier,)):
        m=deserialize_message(b,kind)
        s=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        samples.append((t,(t-s)*1e-9))
    result[name]={'count':len(samples),'age_p50_p95_max':np.percentile([a for _,a in samples],[50,95,100]).tolist(),
                  'interval_p50_p95_max':np.percentile(np.diff([t*1e-9 for t,_ in samples]),[50,95,100]).tolist()}
identifier,kind=topics['/spot1/scored_nav_graph']
profiles=[]
for t,b in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 3',(identifier,)):
    start=time.perf_counter();m=deserialize_message(b,kind)
    profiles.append({'bytes':len(b),'nodes':len(m.nodes),'edges':len(m.edges),'deserialize_ms':(time.perf_counter()-start)*1000})
result['graph_deserialization']=profiles
(trial/'input_latency.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
