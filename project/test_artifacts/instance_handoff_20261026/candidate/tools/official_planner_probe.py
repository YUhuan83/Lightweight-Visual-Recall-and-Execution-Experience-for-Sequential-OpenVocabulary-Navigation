#!/usr/bin/env python3
"""Inspect the live Town costmap and request plans without moving the vehicle.

Run in the sourced ROS 2 / WildOS workspace. This script never publishes cmd_vel
or calls FollowPath. OccupancyGrid values are rounded for visualization, so the
footprint counts below are diagnostics rather than Nav2's exact collision test.

At the Town birth point, use --require-near-spawn. For the second user-defined
area, pass --expected-spawn 150.596 -576.816 --require-near-spawn.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from wildos_nav2_adapter.pointcloud import decode_xyz


def yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def footprint_costs(grid: OccupancyGrid, pose, margin: float = 0.0) -> dict:
    """Summarize published costmap cells inside the oriented vehicle rectangle."""
    width = int(grid.info.width)
    height = int(grid.info.height)
    values = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    rows, cols = np.indices((height, width))
    resolution = float(grid.info.resolution)
    origin = grid.info.origin
    origin_yaw = yaw(origin.orientation)
    mx = (cols + 0.5) * resolution
    my = (rows + 0.5) * resolution
    wx = origin.position.x + mx * math.cos(origin_yaw) - my * math.sin(origin_yaw)
    wy = origin.position.y + mx * math.sin(origin_yaw) + my * math.cos(origin_yaw)
    dx = wx - pose.position.x
    dy = wy - pose.position.y
    robot_yaw = yaw(pose.orientation)
    local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
    local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
    inside = ((local_x >= -2.3 - margin) & (local_x <= 2.3 + margin)
              & (np.abs(local_y) <= 1.0 + margin))
    selected = values[inside]
    return {
        "margin_m": margin,
        "cells": int(selected.size),
        "unknown": int(np.count_nonzero(selected < 0)),
        "inscribed_99": int(np.count_nonzero(selected == 99)),
        "lethal_100": int(np.count_nonzero(selected == 100)),
        "other_occupied_1_to_98": int(np.count_nonzero(
            (selected >= 1) & (selected <= 98))),
    }


def center_cost(grid: OccupancyGrid, pose) -> int | None:
    """Read the published cell below the vehicle center, if it is in bounds."""
    resolution = float(grid.info.resolution)
    if resolution <= 0:
        return None
    origin = grid.info.origin
    angle = yaw(origin.orientation)
    dx = pose.position.x - origin.position.x
    dy = pose.position.y - origin.position.y
    col = math.floor((dx * math.cos(angle) + dy * math.sin(angle)) / resolution)
    row = math.floor((-dx * math.sin(angle) + dy * math.cos(angle)) / resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if not (0 <= col < width and 0 <= row < height):
        return None
    return int(grid.data[row * width + col])


def stamp_seconds(message) -> float | None:
    if message is None:
        return None
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def cloud_summary(message, *, filtered: bool) -> dict | None:
    if message is None:
        return None
    summary = {"frame": message.header.frame_id,
               "stamp_s": stamp_seconds(message),
               "message_points": int(message.width) * int(message.height)}
    if filtered:
        try:
            points = decode_xyz(message)
        except ValueError as exc:
            summary["decode_error"] = str(exc)
            return summary
        near = ((np.abs(points[:, 0]) <= 2.55)
                & (np.abs(points[:, 1]) <= 1.25))
        inside = ((np.abs(points[:, 0]) <= 2.3)
                  & (np.abs(points[:, 1]) <= 1.0))
        summary["body_points_after_filter"] = int(np.count_nonzero(inside))
        summary["near_body_band_points"] = int(np.count_nonzero(near & ~inside))
    return summary


def wait_for_messages(node: Node, state: dict, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if state["odom"] is not None and state["costmap"] is not None:
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    return False


def plan(node: Node, client: ActionClient, pose, frame: str,
         planner_id: str, ahead: float, timeout: float) -> dict:
    heading = yaw(pose.orientation)
    request = ComputePathToPose.Goal()
    request.use_start = False
    request.planner_id = planner_id
    request.goal.header.frame_id = frame
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x = pose.position.x + ahead * math.cos(heading)
    request.goal.pose.position.y = pose.position.y + ahead * math.sin(heading)
    request.goal.pose.position.z = pose.position.z
    request.goal.pose.orientation = pose.orientation
    output = {"planner_id": planner_id, "ahead_m": ahead,
              "goal_xy": [request.goal.pose.position.x,
                          request.goal.pose.position.y]}
    try:
        sent = client.send_goal_async(request)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=timeout)
    except Exception as exc:
        output.update(status="send_error", detail=str(exc)[:200])
        return output
    if not sent.done():
        output["status"] = "accept_timeout"
        return output
    try:
        handle = sent.result()
    except Exception as exc:
        output.update(status="accept_error", detail=str(exc)[:200])
        return output
    if handle is None or not handle.accepted:
        output["status"] = "rejected"
        return output
    try:
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(node, finished, timeout_sec=timeout)
    except Exception as exc:
        output.update(status="result_error", detail=str(exc)[:200])
        return output
    if not finished.done():
        handle.cancel_goal_async()
        output["status"] = "result_timeout"
        return output
    try:
        wrapped = finished.result()
    except Exception as exc:
        output.update(status="result_error", detail=str(exc)[:200])
        return output
    output["action_status"] = int(wrapped.status)
    output["poses"] = len(wrapped.result.path.poses)
    error_code = getattr(wrapped.result, "error_code", None)
    error_msg = getattr(wrapped.result, "error_msg", None)
    if isinstance(error_code, int):
        output["error_code"] = error_code
    if isinstance(error_msg, str) and error_msg:
        output["detail"] = error_msg[:200]
    planning_time = wrapped.result.planning_time
    output["planning_seconds"] = (planning_time.sec
                                   + planning_time.nanosec * 1e-9)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ahead", nargs="+", type=float, default=[4.5, 10.0])
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--expected-spawn", nargs=2, type=float,
                        default=[734.294, -11.946], metavar=("X", "Y"))
    parser.add_argument("--spawn-tolerance", type=float, default=3.0)
    parser.add_argument("--require-near-spawn", action="store_true")
    args = parser.parse_args()
    if (any(distance <= 0 for distance in args.ahead) or args.timeout <= 0
            or args.spawn_tolerance < 0):
        parser.error("ahead distances and timeout must be positive; tolerance must be non-negative")

    rclpy.init()
    node = Node("wildos_official_planner_probe")
    state = {"odom": None, "costmap": None, "raw_cloud": None,
             "input_cloud": None, "filtered_cloud": None,
             "clearing_cloud": None}
    node.create_subscription(
        Odometry, "/spot1/odom",
        lambda msg: state.__setitem__("odom", msg), 10)
    node.create_subscription(
        OccupancyGrid, "/global_costmap/costmap",
        lambda msg: state.__setitem__("costmap", msg), 10)
    node.create_subscription(
        PointCloud2, "/vehicles/ego/sensors/front_lidar",
        lambda msg: state.__setitem__("raw_cloud", msg), 10)
    node.create_subscription(
        PointCloud2, "/spot1/ouster/front/points_filtered",
        lambda msg: state.__setitem__("input_cloud", msg), 10)
    node.create_subscription(
        PointCloud2, "/wildos/points",
        lambda msg: state.__setitem__("filtered_cloud", msg), 10)
    node.create_subscription(
        PointCloud2, "/wildos/clearing_points",
        lambda msg: state.__setitem__("clearing_cloud", msg), 10)
    try:
        if not wait_for_messages(node, state, args.timeout):
            print(json.dumps({"error": "odom_or_costmap_timeout"}))
            return 1
        cloud_deadline = time.monotonic() + min(args.timeout, 2.0)
        while ((state["raw_cloud"] is None or state["input_cloud"] is None
                or state["filtered_cloud"] is None
                or state["clearing_cloud"] is None)
               and time.monotonic() < cloud_deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
        odom = state["odom"]
        grid = state["costmap"]
        if odom.header.frame_id != grid.header.frame_id:
            print(json.dumps({"error": "frame_mismatch",
                              "odom_frame": odom.header.frame_id,
                              "costmap_frame": grid.header.frame_id}))
            return 1
        pose = odom.pose.pose
        spawn_distance = math.hypot(
            pose.position.x - args.expected_spawn[0],
            pose.position.y - args.expected_spawn[1])
        near_spawn = spawn_distance <= args.spawn_tolerance
        report = {
            "frame": grid.header.frame_id,
            "robot_xy_yaw": [pose.position.x, pose.position.y,
                             yaw(pose.orientation)],
            "expected_spawn_xy": args.expected_spawn,
            "distance_from_spawn_m": spawn_distance,
            "near_expected_spawn": near_spawn,
            "costmap_resolution_m": grid.info.resolution,
            "costmap_stamp_s": stamp_seconds(grid),
            "odom_to_costmap_stamp_s": stamp_seconds(odom) - stamp_seconds(grid),
            "center_cost": center_cost(grid, pose),
            "raw_cloud": cloud_summary(state["raw_cloud"], filtered=False),
            "input_cloud": cloud_summary(state["input_cloud"], filtered=False),
            "filtered_cloud": cloud_summary(state["filtered_cloud"], filtered=True),
            "clearing_cloud": cloud_summary(state["clearing_cloud"], filtered=False),
            "odom_to_input_cloud_stamp_s": (
                None if state["input_cloud"] is None else
                stamp_seconds(odom) - stamp_seconds(state["input_cloud"])),
            "raw_to_input_cloud_stamp_s": (
                None if state["raw_cloud"] is None or state["input_cloud"] is None else
                stamp_seconds(state["raw_cloud"]) - stamp_seconds(state["input_cloud"])),
            "odom_to_filtered_cloud_stamp_s": (
                None if state["filtered_cloud"] is None else
                stamp_seconds(odom) - stamp_seconds(state["filtered_cloud"])),
            "odom_to_clearing_cloud_stamp_s": (
                None if state["clearing_cloud"] is None else
                stamp_seconds(odom) - stamp_seconds(state["clearing_cloud"])),
            "assumed_footprint_m": {"front": 2.3, "rear": 2.3,
                                    "half_width": 1.0},
            "footprint_cells": [footprint_costs(grid, pose),
                                footprint_costs(grid, pose, margin=0.25)],
            "plans": [],
        }
        if args.require_near_spawn and not near_spawn:
            report["error"] = "not_at_expected_spawn"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 1
        client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        if not client.wait_for_server(timeout_sec=args.timeout):
            report["error"] = "planner_server_unavailable"
        else:
            for ahead in args.ahead:
                for planner_id in ("CarLike", "GridBased"):
                    report["plans"].append(plan(
                        node, client, pose, grid.header.frame_id,
                        planner_id, ahead, args.timeout))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if "error" not in report else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
