"""Choose a drivable observation pose for in-process object-goal replay."""

from __future__ import annotations

from copy import deepcopy
import math


def observed_viewpoint_pose(odom, target_frame):
    """Return the pose that saw the object, or None for invalid observations.

    The object detector's triangulated point may lie inside the object and its
    quaternion is neutral. The observer pose was occupied by the vehicle and
    preserves the viewing heading for a later approach.
    """
    if odom is None or odom.header.frame_id != target_frame:
        return None
    pose = odom.pose.pose
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(float(value)) for value in values):
        return None
    norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
    if abs(norm - 1.0) > 0.05:
        return None
    return deepcopy(pose)
