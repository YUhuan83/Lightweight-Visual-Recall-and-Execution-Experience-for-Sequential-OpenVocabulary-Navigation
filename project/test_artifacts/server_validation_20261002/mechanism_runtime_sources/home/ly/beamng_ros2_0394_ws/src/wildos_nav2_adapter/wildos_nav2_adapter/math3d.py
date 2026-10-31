"""Pure geometry helpers with no ROS dependency (unit-testable offline)."""
from __future__ import annotations

import math


def to_xyz(value):
    """Accept a geometry Vector3 message or any 3-sequence."""
    if hasattr(value, "x"):
        return float(value.x), float(value.y), float(value.z)
    return float(value[0]), float(value[1]), float(value[2])


def quaternion_from_forward_up(forward, up):
    """Approximate xyzw quaternion from forward/up vectors.

    Builds base axes x=forward, y=left, z=up and converts the rotation matrix
    to a quaternion. Returns a unit quaternion for well-formed input.
    """
    fx, fy, fz = to_xyz(forward)
    ux, uy, uz = to_xyz(up)
    flen = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fx, fy, fz = fx / flen, fy / flen, fz / flen
    rx = fy * uz - fz * uy
    ry = fz * ux - fx * uz
    rz = fx * uy - fy * ux
    rlen = math.sqrt(rx * rx + ry * ry + rz * rz) or 1.0
    rx, ry, rz = rx / rlen, ry / rlen, rz / rlen
    ux, uy, uz = ry * fz - rz * fy, rz * fx - rx * fz, rx * fy - ry * fx
    matrix = (
        (fx, -rx, ux),
        (fy, -ry, uy),
        (fz, -rz, uz),
    )
    return quaternion_from_matrix(matrix)


def quaternion_from_matrix(m):
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def matrix_from_quaternion(q):
    """Return a 3x3 rotation matrix from an xyzw quaternion."""
    x, y, z, w = (float(v) for v in q)
    norm = x*x + y*y + z*z + w*w
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    scale = 2.0 / norm
    return (
        (1.0 - scale*(y*y + z*z), scale*(x*y - z*w), scale*(x*z + y*w)),
        (scale*(x*y + z*w), 1.0 - scale*(x*x + z*z), scale*(y*z - x*w)),
        (scale*(x*z - y*w), scale*(y*z + x*w), 1.0 - scale*(x*x + y*y)),
    )


def world_velocity_to_body(velocity, forward, up):
    """Project a world velocity into ROS base axes x=forward,y=left,z=up."""
    vx, vy, vz = to_xyz(velocity)
    fx, fy, fz = to_xyz(forward)
    ux, uy, uz = to_xyz(up)
    flen = math.sqrt(fx*fx + fy*fy + fz*fz) or 1.0
    fx, fy, fz = fx/flen, fy/flen, fz/flen
    rx, ry, rz = fy*uz - fz*uy, fz*ux - fx*uz, fx*uy - fy*ux
    rlen = math.sqrt(rx*rx + ry*ry + rz*rz) or 1.0
    rx, ry, rz = rx/rlen, ry/rlen, rz/rlen
    left = (-rx, -ry, -rz)
    upn = (fy*left[2] - fz*left[1],
           fz*left[0] - fx*left[2],
           fx*left[1] - fy*left[0])
    return (
        vx*fx + vy*fy + vz*fz,
        vx*left[0] + vy*left[1] + vz*left[2],
        vx*upn[0] + vy*upn[1] + vz*upn[2],
    )
