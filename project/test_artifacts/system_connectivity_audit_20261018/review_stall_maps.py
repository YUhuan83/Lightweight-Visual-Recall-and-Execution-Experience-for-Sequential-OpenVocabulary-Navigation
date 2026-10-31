"""Read only the closed 164222 interface bag around its actuated stall."""
from pathlib import Path
import hashlib
import json
import math
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.graph_route import validate_local_prefix, footprint_blockers
import wildos_nav2_adapter.continuous_path as continuous_module
import wildos_nav2_adapter.graph_route as route_module

SESSION = Path('/home/ly/smoothing_boundary_repair_20261018/field_native_20261018_164222/sessions/session_20261018_164222')
OUT = Path('/home/ly/native_delay_20261018/stall_connectivity_164222')
OUT.mkdir(exist_ok=False)
assert (SESSION/'session_result.json').is_file()
canonical = json.loads((SESSION/'full_query_review.json').read_text())
start_wall = canonical['goal_started_wall']
db = SESSION/'goal_001/interfaces/interfaces_0.db3'
con = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
topics = {name: (i, typ) for i, name, typ in con.execute('SELECT id,name,type FROM topics')}
def latest(topic, stamp):
    i, typ = topics[topic]
    row = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 1', (i, stamp)).fetchone()
    return (row[0], deserialize_message(row[1], get_message(typ))) if row else None
def stamp_ns(message): return message.header.stamp.sec*10**9+message.header.stamp.nanosec
def xy(p): return (p.x, p.y)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

report = dict(scope=__doc__, source_session=str(SESSION),
              source_db_sha256=hashlib.sha256(db.read_bytes()).hexdigest(),
              production_helpers={name: hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
                                  for name, module in [('continuous_path.py', continuous_module), ('graph_route.py', route_module)]},
              snapshots=[], private_costmap_identity_proven=False,
              raw_lidar_recorded='/vehicles/ego/sensors/front_lidar' in topics,
              limits='Public maps are not proven identical to controller private cache. Snapshots follow recorder wall time, and report original header age. No contact object or body contact force in this bag.')
try:
    for label, elapsed in [('section_dispatched', 277.05990433396073),
                           ('stall_window_start', 283.0453311619931),
                           ('stall_window_end', 288.0453311619931)]:
        target = round((start_wall+elapsed)*1e9)
        odom_row = latest('/spot1/odom', target)
        path_row = latest('/wildos/planned_path', target)
        assert odom_row and path_row
        odom = odom_row[1]; path = path_row[1]
        position = xy(odom.pose.pose.position); heading = yaw(odom.pose.pose.orientation)
        cropped = trim_moving_path(path, position, heading)
        record = dict(label=label, goal_elapsed=elapsed, wall_ns=target,
                      pose=[*position, heading], odom_header_age_sec=(target-stamp_ns(odom))*1e-9,
                      original_path_header_ns=stamp_ns(path), actual_pose_can_trim=cropped is not None,
                      grids={})
        evidence = dict(odom=message_to_ordereddict(odom), path=message_to_ordereddict(path))
        for topic in ('/local_costmap/costmap', '/global_costmap/costmap', '/spot1/local_map'):
            row = latest(topic, target); assert row
            grid = row[1]
            details = dict(frame=grid.header.frame_id, resolution=grid.info.resolution,
                           width=grid.info.width, height=grid.info.height,
                           source_age_sec=(target-stamp_ns(grid))*1e-9,
                           recorder_age_sec=(target-row[0])*1e-9)
            if grid.header.frame_id == 'spot1/odom':
                details['current_footprint_blockers'] = footprint_blockers(grid, position, heading, limit=8)
                if cropped:
                    points = [xy(p.pose.position) for p in cropped.poses]
                    headings = [yaw(p.pose.orientation) for p in cropped.poses]
                    details['current_forward_prefix_issue'] = validate_local_prefix(points, headings, grid, position, heading, maximum=2.)
            evidence[topic] = message_to_ordereddict(grid)
            record['grids'][topic] = details
        evidence['review'] = record
        (OUT/(label+'.json')).write_text(json.dumps(evidence)+'\n')
        report['snapshots'].append(record)
finally:
    con.close()
(OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
