#!/usr/bin/env python3
"""Offline replay of GoalStandoff inputs from a rosbag2 SQLite recording.

This script reads only object_confirmation, local_map, and odometry topics. It
does not initialize ROS nodes, publish commands, or change the simulation.
"""
import json
import math
import sys
from bisect import bisect_right

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = sys.argv[1]
OUTPUT = sys.argv[2]
TOPICS = {
    "/spot1/object_confirmation": "std_msgs/msg/String",
    "/spot1/local_map": "nav_msgs/msg/OccupancyGrid",
    "/spot1/odom": "nav_msgs/msg/Odometry",
    "/spot1/imgnav_waypoint": "geometry_msgs/msg/PoseStamped",
}


def stamp_ns(h):
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def pose_xy(msg):
    p = msg.pose.pose.position
    return np.array([float(p.x), float(p.y)])


def quat_matrix(q):
    x, y, z, w = map(float, (q.x, q.y, q.z, q.w))
    n = x*x + y*y + z*z + w*w
    s = 2.0 / n
    return np.array([[1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
                     [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
                     [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]])


def safe_travel(grid, resolution, origin, target, desired, occupied_margin, unknown_margin):
    tx, ty = map(float, target)
    length = float(np.hypot(tx, ty))
    desired = min(max(float(desired), 0.0), length)
    if length < 1e-6 or desired <= 0:
        return 0.0, "zero"
    step = max(float(resolution) * 0.5, 0.05)
    for distance in np.arange(step, desired + step, step):
        x, y = tx * distance / length, ty * distance / length
        ix = math.floor((x - float(origin[0])) / resolution)
        iy = math.floor((y - float(origin[1])) / resolution)
        if not (0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0]):
            return max(0.0, distance - unknown_margin), "outside"
        cell = int(grid[iy, ix])
        if cell < 0:
            return max(0.0, distance - unknown_margin), "unknown"
        if cell >= 50:
            return max(0.0, distance - occupied_margin), "occupied"
    return desired, "clear"


def ring_candidate(grid, resolution, origin, target, radius, occupied_margin, unknown_margin):
    target = np.asarray(target, dtype=float)
    distance = float(np.linalg.norm(target))
    if distance <= radius:
        return np.zeros(2), "already_inside"
    base = math.atan2(target[1], target[0])
    offsets = [0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90,
               105, -105, 120, -120, 135, -135, 150, -150, 165, -165, 180]
    for degrees in offsets:
        angle = base + math.pi + math.radians(degrees)
        candidate = target + radius * np.array([math.cos(angle), math.sin(angle)])
        travel = float(np.linalg.norm(candidate))
        if travel < 0.1:
            return candidate, "at_robot"
        safe, cause = safe_travel(grid, resolution, origin, candidate, travel,
                                  occupied_margin, unknown_margin)
        if safe + 0.1 >= travel:
            return candidate, "ring_clear:" + cause
    return None, "no_ring_ray_clear"


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    classes = {topic: get_message(types[topic]) for topic in TOPICS}
    maps, odoms, confirmations, waypoints = [], [], [], []
    while reader.has_next():
        topic, raw, record_ns = reader.read_next()
        msg = deserialize_message(raw, classes[topic])
        if topic == "/spot1/local_map":
            maps.append((int(record_ns), msg, stamp_ns(msg.header)))
        elif topic == "/spot1/odom":
            odoms.append((int(record_ns), msg, stamp_ns(msg.header)))
        elif topic == "/spot1/object_confirmation":
            try:
                data = json.loads(msg.data)
            except Exception:
                continue
            if data.get("position_confirmed"):
                confirmations.append((int(record_ns), data))
        else:
            waypoints.append((int(record_ns), msg))

    map_records = [r for r in maps]
    odom_records = [r for r in odoms]
    odom_record_ns = [r[0] for r in odom_records]
    rows = []
    for confirm_record_ns, data in confirmations:
        mi = bisect_right([r[0] for r in map_records], confirm_record_ns) - 1
        oi = bisect_right(odom_record_ns, confirm_record_ns) - 1
        if mi < 0 or oi < 0:
            continue
        map_record_ns, occ, map_stamp = map_records[mi]
        current_odom = odom_records[oi][1]
        wi = bisect_right([r[0] for r in waypoints], confirm_record_ns) - 1
        latest_waypoint = waypoints[wi][1] if wi >= 0 else None
        # GoalStandoff keeps the most recent 200 odometry messages, then pairs
        # the current occupancy map with the closest odometry header stamp.
        history = odom_records[max(0, oi - 199):oi + 1]
        paired = min(history, key=lambda r: abs(r[2] - map_stamp))
        pair_odom = paired[1]
        i = occ.info
        grid = np.asarray(occ.data, dtype=np.int16).reshape(int(i.height), int(i.width))
        robot = pose_xy(pair_odom)
        target_world = np.asarray(data["object_position"][:2], dtype=float)
        q = pair_odom.pose.pose.orientation
        rot = quat_matrix(q)
        target_local = np.array([target_world[0] - robot[0],
                                 target_world[1] - robot[1], 0.0]) @ rot
        target_local = target_local[:2]
        origin = (float(i.origin.position.x), float(i.origin.position.y))
        # Published object-to-waypoint distances cluster at 4 m in this trial;
        # use the measured runtime setting rather than the launch default.
        standoff, obstacle_margin = 4.0, 2.0
        unknown_margin = min(obstacle_margin, max(float(i.resolution), 0.5))
        cand, reason = ring_candidate(grid, float(i.resolution), origin,
                                     target_local, standoff, obstacle_margin, unknown_margin)
        if cand is None:
            dist = float(np.linalg.norm(target_world - robot))
            travel, cause = safe_travel(grid, float(i.resolution), origin, target_local,
                                        max(0.0, dist-standoff), obstacle_margin, unknown_margin)
            local_dir = target_local / max(float(np.linalg.norm(target_local)), 1e-6)
            xy_world = robot + rot[:2, :2] @ (local_dir * travel)
            output_dist = float(np.linalg.norm(xy_world - pose_xy(current_odom)))
            output_gap = float(np.linalg.norm(target_world - xy_world))
            stop_reason = cause
        else:
            xy_world = robot + rot[:2, :2] @ cand
            output_dist = float(np.linalg.norm(xy_world - pose_xy(current_odom)))
            output_gap = float(np.linalg.norm(target_world - xy_world))
            stop_reason = reason
        robot_now = pose_xy(current_odom)
        robot_object_gap = float(np.linalg.norm(target_world - robot_now))
        failure = robot_object_gap > standoff + .2 and output_dist < .2
        rows.append({
            "bag_elapsed_s": round((confirm_record_ns - maps[0][0]) / 1e9, 3),
            "query": data.get("query"), "logged_ready": data.get("approach_goal_ready"),
            "logged_reason": data.get("approach_failure_reason"),
            "replay_reason": stop_reason, "replay_no_reachable": bool(failure),
            "grid_age_s": round((confirm_record_ns-map_record_ns)/1e9, 3),
            "map_odom_header_delta_s": round(abs(paired[2]-map_stamp)/1e9, 4),
            "resolution": float(i.resolution), "grid_shape": [int(i.height), int(i.width)],
            "robot_object_gap_m": round(robot_object_gap, 3),
            "goal_distance_m": round(output_dist, 3), "goal_object_gap_m": round(output_gap, 3),
            "target_local_xy": np.round(target_local, 3).tolist(),
            "target_ray_safe": round(safe_travel(grid, float(i.resolution), origin,
                target_local, max(0.0, float(np.linalg.norm(target_local))-standoff),
                obstacle_margin, unknown_margin)[0], 3),
            "logged_standoff": data.get("standoff"),
            "latest_waypoint": ([float(latest_waypoint.pose.position.x),
                                  float(latest_waypoint.pose.position.y)]
                                 if latest_waypoint else None),
            "current_robot_xy": np.round(pose_xy(current_odom), 4).tolist(),
            "paired_robot_xy": np.round(robot, 4).tolist(),
            "actual_waypoint_to_current_m": (round(float(np.linalg.norm(
                np.asarray([latest_waypoint.pose.position.x, latest_waypoint.pose.position.y]) -
                pose_xy(current_odom))), 3) if latest_waypoint else None),
            "actual_waypoint_to_paired_m": (round(float(np.linalg.norm(
                np.asarray([latest_waypoint.pose.position.x, latest_waypoint.pose.position.y]) -
                robot)), 3) if latest_waypoint else None),
            "actual_waypoint_object_gap_m": (round(float(np.linalg.norm(
                np.asarray([latest_waypoint.pose.position.x, latest_waypoint.pose.position.y]) -
                target_world)), 3) if latest_waypoint else None),
            "waypoint_age_s": (round((confirm_record_ns-waypoints[wi][0])/1e9, 3)
                               if wi >= 0 else None),
            "map_origin": list(origin), "unknown_cells": int(np.count_nonzero(grid < 0)),
            "occupied_cells": int(np.count_nonzero(grid >= 50)),
            "map_frame": occ.header.frame_id,
            "origin_quaternion_norm": round(math.sqrt(
                occ.info.origin.orientation.x**2 + occ.info.origin.orientation.y**2 +
                occ.info.origin.orientation.z**2 + occ.info.origin.orientation.w**2), 6),
            "robot_center_cell": int(grid[math.floor((0-origin[1])/i.resolution),
                                           math.floor((0-origin[0])/i.resolution)])
                                 if 0 <= math.floor((0-origin[1])/i.resolution) < grid.shape[0]
                                 and 0 <= math.floor((0-origin[0])/i.resolution) < grid.shape[1]
                                 else None,
            "actual_waypoint_ray_cells": [],
            "actual_waypoint_local_xy": None,
        })
        row = rows[-1]
        if data.get("standoff"):
            actual_local = (np.asarray(data["standoff"], dtype=float) - robot) @ rot[:2, :2]
            row["actual_waypoint_local_xy"] = np.round(actual_local, 3).tolist()
            actual_len = float(np.linalg.norm(actual_local))
            for d in np.arange(max(float(i.resolution)*0.5, 0.05), actual_len + 1e-6,
                               max(float(i.resolution)*0.5, 0.05)):
                xy = actual_local * (d / max(actual_len, 1e-9))
                ix = math.floor((xy[0]-origin[0])/float(i.resolution))
                iy = math.floor((xy[1]-origin[1])/float(i.resolution))
                cell = int(grid[iy, ix]) if 0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0] else None
                row["actual_waypoint_ray_cells"].append([round(float(d), 2), cell])
                if cell is None or cell < 0 or cell >= 50:
                    break
    out = {"bag": BAG, "map_messages": len(maps), "odom_messages": len(odoms),
           "waypoint_messages": len(waypoints),
           "confirmed_messages": len(confirmations), "replayed": len(rows), "rows": rows}
    with open(OUTPUT, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    counts = {}
    for r in rows:
        key = (r["logged_reason"], r["replay_reason"], r["replay_no_reachable"])
        counts[str(key)] = counts.get(str(key), 0) + 1
    print(json.dumps({"counts": counts, "rows": rows[:5]}, indent=2))


if __name__ == "__main__":
    main()
