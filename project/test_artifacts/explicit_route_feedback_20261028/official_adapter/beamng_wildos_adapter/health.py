"""Message-content validation for the official WildOS graph."""
from __future__ import annotations

import math


def graph_stats(graph):
    nodes = list(graph.nodes)
    edges = list(graph.edges)
    classes = list(graph.trav_classes)
    default_idx = classes.index("default") if "default" in classes else None
    frontiers = 0
    frontier_points = 0
    scored_nodes = 0
    scores = []
    for node in nodes:
        if default_idx is not None and default_idx < len(node.trav_properties):
            properties = node.trav_properties[default_idx]
            if properties.is_frontier:
                frontiers += 1
                frontier_points += len(properties.frontier_points)
        for prop in node.properties:
            if prop.key == "frontier_scores":
                values = [float(v) for v in prop.value]
                if values and all(math.isfinite(v) for v in values):
                    scored_nodes += 1
                    scores.extend(values)
    edge_indices_valid = all(
        int(edge.from_idx) < len(nodes) and int(edge.to_idx) < len(nodes)
        for edge in edges)
    return {
        "frame": str(graph.header.frame_id),
        "nodes": len(nodes),
        "edges": len(edges),
        "trav_classes": classes,
        "current_node_valid": bool(nodes) and int(graph.current_node_idx) < len(nodes),
        "edge_indices_valid": edge_indices_valid,
        "frontiers": frontiers,
        "frontier_points": frontier_points,
        "scored_nodes": scored_nodes,
        "score_count": len(scores),
        "score_min": min(scores) if scores else None,
        "score_max": max(scores) if scores else None,
    }


def graph_valid(stats, require_scores=False):
    valid = (
        stats["frame"] == "spot1/odom"
        and stats["nodes"] > 0
        and stats["current_node_valid"]
        and stats["edge_indices_valid"]
        and "default" in stats["trav_classes"]
        and stats["frontiers"] > 0
        and stats["frontier_points"] > 0
    )
    return valid and (not require_scores or (
        stats["scored_nodes"] > 0 and stats["score_count"] > 0))


def image_valid(msg):
    return (msg.header.frame_id == "spot1/realsense/front_color_optical_frame"
            and int(msg.width) == 960 and int(msg.height) == 640
            and msg.encoding == "rgb8"
            and len(msg.data) >= int(msg.step) * int(msg.height))


def camera_info_valid(msg):
    values = [float(v) for v in msg.k]
    return (msg.header.frame_id == "spot1/realsense/front_color_optical_frame"
            and int(msg.width) == 960 and int(msg.height) == 640
            and len(values) == 9 and all(math.isfinite(v) for v in values)
            and values[0] > 0 and values[4] > 0)


def odom_valid(msg):
    q = msg.pose.pose.orientation
    qn = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    return (msg.header.frame_id == "spot1/odom"
            and msg.child_frame_id == "spot1/base_link"
            and math.isfinite(qn) and 0.9 < qn < 1.1)


def cloud_valid(msg):
    return (msg.header.frame_id == "spot1/ouster/front/os_lidar"
            and int(msg.width) > 100 and int(msg.point_step) >= 12
            and len(msg.data) >= int(msg.row_step) * int(msg.height))


def occupancy_valid(msg):
    values = set(int(v) for v in msg.data)
    return (msg.header.frame_id == "spot1/base_link"
            and int(msg.info.width) > 0 and int(msg.info.height) > 0
            and len(msg.data) == int(msg.info.width) * int(msg.info.height)
            and -1 in values and 0 in values)


def path_valid(msg, minimum=2):
    return msg.header.frame_id == "spot1/odom" and len(msg.poses) >= minimum


def goal_valid(msg):
    p = msg.pose.position
    return msg.header.frame_id == "spot1/odom" and all(
        math.isfinite(float(v)) for v in (p.x, p.y, p.z))
