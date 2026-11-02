"""Bounded query-independent scene samples using the executor's node contract."""

import math
import numpy as np


def append_scene_sample(
    samples, embedding, graph, odom, now, ttl, heading_count, uuid_to_str
):
    if graph.header.frame_id != odom.header.frame_id or not graph.nodes:
        return False
    stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
    graph_stamp = graph.header.stamp.sec + graph.header.stamp.nanosec * 1e-9
    if stamp <= 0 or abs(stamp - graph_stamp) > 5.0 or not np.isfinite(now):
        return False
    vector = np.asarray(embedding, dtype=np.float32).reshape(-1)
    p = odom.pose.pose.position
    if (
        not vector.size
        or not np.all(np.isfinite(vector))
        or np.linalg.norm(vector) <= 1e-8
        or not all(map(math.isfinite, (p.x, p.y)))
        or not 1 <= heading_count <= 1024
    ):
        return False
    nodes = [
        n
        for n in graph.nodes
        if any(n.uuid.id)
        and math.isfinite(n.pose.position.x)
        and math.isfinite(n.pose.position.y)
    ]
    if not nodes:
        return False
    node = min(
        nodes,
        key=lambda n: math.hypot(n.pose.position.x - p.x, n.pose.position.y - p.y),
    )
    if math.hypot(node.pose.position.x - p.x, node.pose.position.y - p.y) > 6.0:
        return False
    while samples and now - samples[0]["created_at"] > ttl:
        samples.popleft()
    uid = uuid_to_str(node.uuid)
    if samples:
        if stamp <= samples[-1]["ros_stamp"]:
            return False
        if uid == samples[-1]["node_uuid"] and stamp - samples[-1]["ros_stamp"] < 0.25:
            return False
    q = getattr(odom.pose.pose, "orientation", None)
    quaternion = None if q is None else np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    scene_heading = None
    if (
        quaternion is not None
        and np.isfinite(quaternion).all()
        and abs(np.linalg.norm(quaternion) - 1.0) < 0.01
    ):
        x, y, z, w = quaternion / np.linalg.norm(quaternion)
        scene_heading = float(
            np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        )
    samples.append(
        dict(
            created_at=now,
            ros_stamp=stamp,
            embedding=vector.copy(),
            heading_scores=None,
            scene_heading=scene_heading,
            scene_frame=odom.header.frame_id,
            heading_count=int(heading_count),
            node_uuid=uid,
        )
    )
    return True
