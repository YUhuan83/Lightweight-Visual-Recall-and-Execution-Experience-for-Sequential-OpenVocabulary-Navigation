"""Estimate observed lower-surface height in the actual body frame from a closed bag."""
from pathlib import Path
import json
import math
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.geometry import quaternion_matrix
from wildos_nav2_adapter.pointcloud import local_ground_plane

session = Path('/home/ly/smoothing_boundary_repair_20261018/field_native_20261018_164222/sessions/session_20261018_164222')
assert (session/'session_result.json').is_file()
review = json.loads((session/'full_query_review.json').read_text())
db = session/'goal_001/interfaces/interfaces_0.db3'
con = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
topics = {name: (i, get_message(typ)) for i, name, typ in con.execute('SELECT id,name,type FROM topics')}
def ns(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
results = []
try:
    for elapsed in (277.05990433396073, 283.0453311619931, 288.0453311619931):
        target = round((review['goal_started_wall']+elapsed)*1e9)
        i, cls = topics['/spot1/local_terrain']
        row = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 1', (i, target)).fetchone()
        terrain = deserialize_message(row[1], cls)
        assert terrain.header.frame_id == 'spot1/terrain_base_link'
        assert terrain.outer_start_index == terrain.inner_start_index == 0
        n = round(terrain.info.length_x/terrain.info.resolution)
        layers = {name: np.asarray(array.data).reshape(n, n)[::-1, ::-1]
                  for name, array in zip(terrain.layers, terrain.data)}
        yy, xx = np.indices((n, n)); x=(xx+.5-n/2)*terrain.info.resolution; y=(yy+.5-n/2)*terrain.info.resolution
        valid = ((layers['elevation_valid'] > .5) & (layers['unknown'] < .5)
                 & (layers['traversability'] == 0.) & (np.hypot(x, y) <= 8.))
        points = np.column_stack((x[valid], y[valid], layers['elevation'][valid]))
        j, odom_cls = topics['/spot1/odom']
        candidates = [deserialize_message(data, odom_cls) for _, data in con.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ?',
            (j, ns(terrain)-1_000_000_000, ns(terrain)+2_000_000_000))]
        odom = min(candidates, key=lambda o: abs(ns(o)-ns(terrain)))
        q = odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        level = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        body = quaternion_matrix([q.x, q.y, q.z, q.w])
        body_points = points @ level.T @ body
        plane = local_ground_plane(body_points, 8.)
        results.append(dict(goal_elapsed=elapsed, terrain_header_ns=ns(terrain),
            pose_source_skew_sec=(ns(odom)-ns(terrain))*1e-9,
            observed_near_free_cells=len(points), body_plane=plane.tolist() if plane is not None else None,
            observed_body_z_quantiles=np.quantile(body_points[:, 2], [.05, .5, .95]).tolist() if len(points) else None,
            limits='Aggregated observed lower-surface cells, not original raw LiDAR. Plane diagnostic cannot identify the stalled obstruction.'))
finally:
    con.close()
output = Path('/home/ly/native_delay_20261018/stall_connectivity_164222/terrain_review.json')
report = dict(scope=__doc__, snapshots=results, production_changed=False,
              original_lidar_replayed=False, field_obstruction_identified=False)
output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
