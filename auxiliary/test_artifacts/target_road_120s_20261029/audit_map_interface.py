"""Inspect the same world points across existing closed-bag map versions."""
import json
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
from wildos_nav2_adapter.graph_route import cell_cost, footprint_blockers

root = Path(__file__).resolve().parent
trial = root/'full/trial_20261029_212751'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
source=json.loads((trial/'abort_obstacle_source.json').read_text())
tracking=json.loads((trial/'tracking_collision_audit.json').read_text())[0]
stamp=tracking['stamp_ns']
points=[row['world'][:2] for row in source['nearby_current_markings']]
pose=tracking['path_blocked_within4m'][0]['pose']
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
maps={}
for label in ('local','global'):
    topic='/'+label+'_costmap/costmap'
    index=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
    maps[label]=[]
    for receipt,data in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(index,stamp-2_000_000_000,stamp+2_000_000_000)):
        m=deserialize_message(data,OccupancyGrid)
        maps[label].append(dict(receipt_delta_s=(receipt-stamp)*1e-9,
            stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
            resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],
            source_point_costs=[cell_cost(m,*p) for p in points],
            path_footprint_blockers=footprint_blockers(m,pose[:2],pose[2])))
db.close()
patch=np.asarray(source['raw_patch_world'])
summary=dict(blocking_pose=pose,source_points=points,maps=maps,
             raw_patch_points=len(patch),raw_patch_z_quantiles=np.quantile(patch[:,2],[0,.1,.5,.9,1]).tolist())
(trial/'map_interface_audit.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
