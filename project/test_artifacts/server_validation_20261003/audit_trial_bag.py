"""Read recorded DDS data and profile the exact messages used by the bridge."""
from pathlib import Path
from copy import deepcopy
import json
import math
import sqlite3
import sys
import time
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import validate_vehicle_path
from wildos_nav2_adapter.graph_path_bridge import xy, yaw

ROOT = Path('/home/ly/town_memory_validation_20261003')
trial = ROOT / sys.argv[1]
db = sqlite3.connect(next((trial/'rosbag').glob('*.db3')))
topics = {name: (identifier, get_message(kind))
          for identifier, name, kind in db.execute('SELECT id,name,type FROM topics')}
start = db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]

def read(name):
    identifier, kind = topics[name]
    for ts, blob in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (identifier,)):
        yield ts, deserialize_message(blob, kind)

result = {}
for name in ['/spot1/odom', '/global_costmap/costmap']:
    pairs = list(read(name))
    ages = np.array([(ts - (m.header.stamp.sec*10**9 + m.header.stamp.nanosec))*1e-9 for ts,m in pairs])
    intervals = np.diff([ts*1e-9 for ts,m in pairs])
    result[name] = {'count': len(pairs), 'age_p50_p95_max': np.percentile(ages,[50,95,100]).tolist(),
                    'interval_p50_p95_max': np.percentile(intervals,[50,95,100]).tolist()}

graphs = list(read('/spot1/scored_nav_graph'))
profiles = []
for ts, graph in (graphs[0], graphs[len(graphs)//2], graphs[-1]):
    t = time.perf_counter()
    deepcopy(graph)
    profiles.append({'elapsed': (ts-start)*1e-9, 'nodes':len(graph.nodes), 'edges':len(graph.edges),
                     'deepcopy_ms': (time.perf_counter()-t)*1000})
result['graph_copy_profile'] = profiles

odom = list(read('/spot1/odom'))
grids = list(read('/global_costmap/costmap'))
paths = list(read('/wildos/planned_path'))
profiles = []
for ts, path in paths[-6:]:
    grid = next(m for t,m in reversed(grids) if t <= ts)
    pose = next(m for t,m in reversed(odom) if t <= ts).pose.pose
    points, headings = [xy(p.pose) for p in path.poses], [yaw(p.pose) for p in path.poses]
    t = time.perf_counter()
    reason = validate_vehicle_path(points,headings,[xy(pose)]+points,grid,points[-1],start=xy(pose),
                                   minimum_turning_radius=8.,heading_quantization_bins=72)
    profiles.append({'elapsed':(ts-start)*1e-9,'poses':len(points),'validation_ms':(time.perf_counter()-t)*1000,'reason':reason})
result['path_validation_profile'] = profiles

controls = []
for ts, m in read('/wildos/control_status'):
    d = json.loads(m.data)
    if d.get('actuating'):
        controls.append(((ts-start)*1e-9,d))
result['active_control_count'] = len(controls)
result['curvature_bad_count'] = sum(bool(d.get('curvature_infeasible')) for _,d in controls)
result['control_examples'] = [{'elapsed':t,**d} for t,d in controls[::max(1,len(controls)//12)]]
result['path_summary'] = [{'elapsed':(ts-start)*1e-9,'poses':len(p.poses),
                          'start':xy(p.poses[0].pose),'end':xy(p.poses[-1].pose)} for ts,p in paths]
(trial/'bag_diagnosis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
