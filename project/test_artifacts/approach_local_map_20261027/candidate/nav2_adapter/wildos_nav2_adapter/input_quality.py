"""Validate measurement time and geometry before updating navigation inputs."""
import math


def stamp_ns(message):
    return int(message.header.stamp.sec) * 10**9 + int(message.header.stamp.nanosec)


def measurement_age(message, now_ns, timeout, previous_ns=None, future_tolerance_sec=0.05):
    stamp = stamp_ns(message)
    age = (now_ns - stamp) * 1e-9
    if stamp <= 0 or age < -future_tolerance_sec or age > timeout:
        return None
    if previous_ns is not None and stamp <= previous_ns:
        return None
    return max(0., age)


def valid_pose(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return (all(map(math.isfinite, values))
            and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) < .1)


def valid_grid(grid, frame):
    i = grid.info
    return (grid.header.frame_id == frame and i.width > 0 and i.height > 0
            and math.isfinite(i.resolution) and i.resolution > 0
            and len(grid.data) == i.width * i.height and valid_pose(i.origin)
            and all(-1 <= v <= 100 for v in grid.data))
