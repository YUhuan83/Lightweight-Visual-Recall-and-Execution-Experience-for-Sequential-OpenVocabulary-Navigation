"""Summarize recorded route, command, odometry and local-map evidence."""
from __future__ import annotations

import json
import math
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path('/home/ly/town_memory_validation_20261003')
TRIAL = ROOT / sys.argv[1]
DB = sqlite3.connect(f"file:{next((TRIAL / 'rosbag').glob('*.db3'))}?mode=ro", uri=True)
TOPICS = {name: (topic_id, get_message(type_name))
          for topic_id, name, type_name in DB.execute('SELECT id,name,type FROM topics')}
START = DB.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]


def rows(name):
    if name not in TOPICS:
        return
    topic_id, message_type = TOPICS[name]
    for stamp, blob in DB.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topic_id,)):
        yield (stamp - START) / 1e9, deserialize_message(blob, message_type)


def plain(message):
    if hasattr(message, 'data'):
        try:
            return json.loads(message.data)
        except (ValueError, TypeError):
            return message.data[:500]
    return {key: getattr(message, key) for key in message.get_fields_and_field_types()}


series = {
    '/spot1/odom': list(rows('/spot1/odom')),
    '/wildos/cmd_vel': list(rows('/wildos/cmd_vel')),
    '/wildos/control_status': list(rows('/wildos/control_status')),
    '/wildos/native_control_status': list(rows('/wildos/native_control_status')),
    '/spot1/graphnav_planner/route_status': list(rows('/spot1/graphnav_planner/route_status')),
    '/wildos/plan_status': list(rows('/wildos/plan_status')),
    '/wildos/execution_feedback': list(rows('/wildos/execution_feedback')),
    '/spot1/local_map': list(rows('/spot1/local_map')),
}


def nearest(topic, time_s):
    values = series.get(topic, [])
    return min(values, key=lambda pair: abs(pair[0]-time_s)) if values else (None, None)


def xy(p):
    return [float(p.x), float(p.y)]


def pose_info(t, msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    return {'time': round(t, 3), 'xy': xy(p), 'yaw': round(2*math.atan2(q.z, q.w), 4),
            'odom_velocity': [round(v.x, 3), round(v.y, 3), round(v.z, 3)],
            'angular': [round(w.x, 3), round(w.y, 3), round(w.z, 3)]}


def grid_probe(t, grid_pair, odom_pair, goal=(705.9216738638091, -11.575910795781986)):
    if grid_pair[1] is None or odom_pair[1] is None:
        return None
    _, grid = grid_pair
    _, odom = odom_pair
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    yaw = 2*math.atan2(q.z, q.w)
    dx, dy = goal[0]-p.x, goal[1]-p.y
    local_x = math.cos(yaw)*dx + math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx + math.cos(yaw)*dy
    info = grid.info
    ox, oy = info.origin.position.x, info.origin.position.y
    ix, iy = math.floor((local_x-ox)/info.resolution), math.floor((local_y-oy)/info.resolution)
    data = list(grid.data)
    value = data[iy*info.width+ix] if 0 <= ix < info.width and 0 <= iy < info.height else None
    window = []
    for offset in (-2, -1, 0, 1, 2):
        row = []
        for lateral in (-2, -1, 0, 1, 2):
            x, y = ix+offset, iy+lateral
            row.append(data[y*info.width+x] if 0 <= x < info.width and 0 <= y < info.height else None)
        window.append(row)
    return {'topic': '/spot1/local_map', 'header_frame': grid.header.frame_id,
            'origin': [ox, oy], 'resolution': info.resolution, 'goal_local': [local_x, local_y],
            'goal_cell': [ix, iy], 'goal_cell_value': value, 'window_5x5': window}


end = max(row[0] for row in series['/spot1/odom'])
samples = []
for time_s in range(0, int(end)+1, 10):
    ot, odom = nearest('/spot1/odom', time_s)
    ct, cmd = nearest('/wildos/cmd_vel', time_s)
    st, status = nearest('/wildos/control_status', time_s)
    nt, native = nearest('/wildos/native_control_status', time_s)
    rt, route = nearest('/spot1/graphnav_planner/route_status', time_s)
    pt, plan = nearest('/wildos/plan_status', time_s)
    et, feedback = nearest('/wildos/execution_feedback', time_s)
    gt, grid = nearest('/spot1/local_map', time_s)
    samples.append({
        't': time_s,
        'odom': pose_info(ot, odom) if odom else None,
        'cmd_vel': {'t': ct, 'linear_x': cmd.linear.x, 'angular_z': cmd.angular.z} if cmd else None,
        'control_status': {'t': st, **plain(status)} if status else None,
        'native_status': {'t': nt, **plain(native)} if native else None,
        'graph_route_status': {'t': rt, **plain(route)} if route else None,
        'plan_status': {'t': pt, **plain(plan)} if plan else None,
        'execution_feedback': {'t': et, **plain(feedback)} if feedback else None,
        'local_map_near_graph_goal': grid_probe(time_s, (gt, grid), (ot, odom)) if grid and odom else None,
    })

result = {
    'trial': sys.argv[1],
    'bag_duration_s': end,
    'message_counts': {name: len(values) for name, values in series.items()},
    'timeline': samples,
    'last_20_control_status': [
        {'t': round(t, 3), **plain(message)} for t, message in series['/wildos/control_status'][-20:]],
    'last_20_native_status': [
        {'t': round(t, 3), **plain(message)} for t, message in series['/wildos/native_control_status'][-20:]],
}
path = TRIAL / 'follow_stall_diagnosis.json'
path.write_text(json.dumps(result, indent=2, default=str))
print(json.dumps({k: v for k, v in result.items() if k not in ('last_20_control_status', 'last_20_native_status')}, indent=2, default=str))
