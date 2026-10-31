"""Observe current graph and route outputs without publishing a goal."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    import rclpy
    from graphnav_msgs.msg import NavigationGraph
    from nav_msgs.msg import Odometry, Path as Route
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("graph_route_readonly_probe")
    latest = {"graph": None, "route": None, "route_status": None,
              "bridge_status": None, "rejected_endpoint": None, "odom": None}

    def graph_cb(message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        graph = {"stamp_ns": stamp, "frame_id": message.header.frame_id,
                 "nodes": len(message.nodes), "edges": len(getattr(message, "edges", [])),
                 "current_node_idx": int(message.current_node_idx),
                 "robot_profiles": [{"profile_id": item.profile_id,
                                     "holonomic": item.holonomic,
                                     "min_turning_radius": item.min_turning_radius,
                                     "max_linear_velocity": item.max_linear_velocity}
                                    for item in message.robot_profiles]}
        if latest["odom"] is not None:
            origin = latest["odom"]["position"]
            yaw_origin = latest["odom"]["yaw_rad"]
            near = []
            ahead = []
            for node_index, node_msg in enumerate(message.nodes):
                point = [float(node_msg.pose.position.x), float(node_msg.pose.position.y)]
                dx, dy = point[0] - origin[0], point[1] - origin[1]
                distance = math.hypot(dx, dy)
                forward = dx * math.cos(yaw_origin) + dy * math.sin(yaw_origin)
                lateral = -dx * math.sin(yaw_origin) + dy * math.cos(yaw_origin)
                robot_trav = [{"profile_id": item.robot_profile_id,
                               "feasible": item.feasible,
                               "physical_traversability": item.physical_traversability,
                               "minimum_clearance": item.minimum_clearance,
                               "roughness": item.roughness,
                               "dynamic_risk": item.dynamic_risk}
                              for item in node_msg.robot_traversability]
                if distance <= 12.0:
                    q = node_msg.pose.orientation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    selected_headings = [{"heading_frame": item.heading_frame,
                                          "heading": item.heading,
                                          "selected": item.selected,
                                          "traversable": item.traversable,
                                          "minimum_clearance": item.minimum_clearance,
                                          "motion_cost": item.motion_cost}
                                         for item in node_msg.heading_candidates
                                         if item.selected or item.traversable][:8]
                    near.append({"index": node_index,
                                 "uuid": [int(value) for value in node_msg.uuid.id],
                                 "position": point, "yaw_rad": yaw,
                                 "distance_m": distance,
                                 "robot_traversability": robot_trav,
                                 "selected_headings": selected_headings})
                if 6.0 < forward < 20.0 and abs(lateral) < 3.0:
                    ahead.append({"index": node_index, "position": point,
                                  "distance_m": distance,
                                  "forward_m": forward, "lateral_m": lateral,
                                  "robot_traversability": robot_trav})
            graph["nearest_nodes"] = sorted(near, key=lambda item: item["distance_m"])[:12]
            graph["ahead_nodes"] = sorted(ahead, key=lambda item: (abs(item["lateral_m"]),
                                                                    item["forward_m"]))[:20]
        latest["graph"] = graph

    def route_cb(message):
        poses = message.poses
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in poses]
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        latest["route"] = {"stamp_ns": message.header.stamp.sec * 1_000_000_000
                            + message.header.stamp.nanosec,
                            "frame_id": message.header.frame_id, "poses": len(poses),
                            "length_m": length, "start": points[0] if points else None,
                            "end": points[-1] if points else None}

    def text_cb(key):
        def callback(message):
            try:
                latest[key] = json.loads(message.data)
            except (ValueError, TypeError):
                latest[key] = {"raw": message.data[:500]}
        return callback

    def odom_cb(message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        latest["odom"] = {"stamp_ns": message.header.stamp.sec * 1_000_000_000
                          + message.header.stamp.nanosec, "frame_id": message.header.frame_id,
                          "position": [p.x, p.y, p.z], "yaw_rad": yaw}

    subscriptions = [
        node.create_subscription(NavigationGraph, "/spot1/nav_graph", graph_cb,
                                 qos_profile_sensor_data),
        node.create_subscription(Route, "/spot1/graphnav_planner/path", route_cb,
                                 qos_profile_sensor_data),
        node.create_subscription(String, "/spot1/graphnav_planner/route_status",
                                 text_cb("route_status"), 10),
        node.create_subscription(String, "/wildos/plan_status", text_cb("bridge_status"), 10),
        node.create_subscription(String, "/spot1/graphnav_planner/rejected_route_endpoint",
                                 text_cb("rejected_endpoint"), 10),
        node.create_subscription(Odometry, "/spot1/odom", odom_cb,
                                 qos_profile_sensor_data),
    ]
    start = time.monotonic()
    deadline = start + max(1.0, min(args.duration, 20.0))
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = {"duration_s": time.monotonic() - start, "observed": latest,
              "goal_published": False, "motion_command_published": False}
    report["passed"] = bool(latest["graph"] and latest["odom"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
