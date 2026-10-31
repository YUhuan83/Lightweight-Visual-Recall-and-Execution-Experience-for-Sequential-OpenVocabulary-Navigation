"""Choose a bounded observation pose from a visible target's direction.

The pose is an exploration instruction, never an object position or arrival.
All travel remains inside the measured free part of the local occupancy map.
"""
import math
import numpy as np
from visual_navigation.utils.semantic_mask import largest_mask_component
from .mapping import safe_travel_distance


def visible_direction(message):
    dims = [int(d.size) for d in message.object_mask.layout.dim]
    if len(dims) != 4 or dims[1] != 1 or min(dims) <= 0:
        raise ValueError('invalid target mask dimensions')
    offset = int(message.object_mask.layout.data_offset)
    masks = np.asarray(message.object_mask.data[offset:], dtype=np.uint8).reshape(dims)
    if len(message.cam_infos) != dims[0] or len(message.cam_transforms.transforms) != dims[0]:
        raise ValueError('missing target camera calibration')
    components = [largest_mask_component(masks[i, 0] > 0) for i in range(dims[0])]
    index = max(range(dims[0]), key=lambda i: int(np.count_nonzero(components[i])))
    pixels = np.argwhere(components[index])
    if not len(pixels):
        return None
    info = message.cam_infos[index]
    tf = message.cam_transforms.transforms[index]
    if (tf.header.frame_id.lstrip('/') != 'spot1/odom'
            or tf.child_frame_id.lstrip('/') != info.header.frame_id.lstrip('/')
            or (info.height, info.width) != tuple(dims[2:])):
        raise ValueError('target camera frame mismatch')
    stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
    for header in (info.header, tf.header):
        other = header.stamp.sec * 10**9 + header.stamp.nanosec
        if abs(other-stamp) > 50_000_000:
            raise ValueError('target camera time mismatch')
    k = np.asarray(info.k, float).reshape(3, 3)
    if not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('invalid target intrinsics')
    from .geometry import quaternion_matrix
    q = tf.transform.rotation
    values = [q.x, q.y, q.z, q.w]
    if not all(map(math.isfinite, values)) or abs(sum(v*v for v in values)-1.) > .1:
        raise ValueError('invalid target camera rotation')
    rotation = quaternion_matrix(values)
    v, u = np.median(pixels, axis=0)
    ray = rotation @ np.linalg.solve(k, [u, v, 1.])
    if not np.all(np.isfinite(ray)) or np.linalg.norm(ray[:2]) < .1:
        return None
    return math.atan2(ray[1], ray[0])


def observation_step(grid, resolution, origin, map_xy, map_yaw, bearing, maximum=8.,
                     accept=None):
    """Select measured free travel in a narrow cone; retain obstacle margins."""
    if not all(map(math.isfinite, (*map_xy, map_yaw, bearing, resolution, maximum))) or resolution <= 0:
        return None
    candidates = []
    for offset in (0., -.13, .13, -.26, .26):
        direction = bearing + offset
        local = direction - map_yaw
        ray = (maximum*math.cos(local), maximum*math.sin(local))
        travel = safe_travel_distance(grid, resolution, origin, ray, maximum,
                                     safety_margin=2., unknown_safety_margin=max(resolution, .5))
        if travel < 2.:
            continue
        # Reuse this ray's measured free segment. If its endpoint cannot hold
        # the real vehicle, look for a shorter pose before abandoning the ray.
        if accept is not None:
            while travel >= 2.:
                point=(map_xy[0]+travel*math.cos(direction),
                       map_xy[1]+travel*math.sin(direction))
                if accept(point):
                    break
                travel-=resolution
            if travel < 2.:
                continue
        # The whole segment was checked, including unknown cells. A wider turn
        # must earn additional free progress before replacing the central ray.
        merit = travel*math.cos(offset) - 2.*abs(offset)
        candidates.append((merit, (map_xy[0]+travel*math.cos(direction),
                                   map_xy[1]+travel*math.sin(direction))))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None
