"""Convert a measured vehicle origin/dir/up to BeamNG's passive spawn quaternion."""
from pathlib import Path
import math
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'official_adapter'), str(ROOT/'nav2_adapter')]
from beamng_wildos_adapter.geometry import body_rotation, matrix_quaternion
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from types import SimpleNamespace as S

MODEL_BODY_AXES = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


def spawn_from_measurement(native):
    vectors = []
    for name in ('position', 'direction', 'up'):
        value = native[name]
        if (not isinstance(value, (list, tuple)) or len(value) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
            raise ValueError('Invalid measured '+name)
        vectors.append(np.asarray(value, float))
    position, forward, up = vectors
    if (not .5 <= np.linalg.norm(forward) <= 1.5 or not .5 <= np.linalg.norm(up) <= 1.5
            or abs(np.dot(forward, up)/(np.linalg.norm(forward)*np.linalg.norm(up))) > .02):
        raise ValueError('Invalid measured orientation basis')
    vector = lambda a: S(x=a[0], y=a[1], z=a[2])
    body = body_rotation(vector(forward), vector(up))
    passive = MODEL_BODY_AXES @ body.T
    quaternion = np.asarray(matrix_quaternion(passive))
    quaternion /= np.linalg.norm(quaternion)
    rear = position+body@np.asarray(SBR_BASE_OFFSET)
    return dict(position=position.tolist(), rot_quat=quaternion.tolist(),
                measured_body_rotation=body.tolist(), rear_axle=rear.tolist(),
                yaw=math.atan2(body[1, 0], body[0, 0]),
                convention='BeamNG passive quaternion; model forward=(0,-1,0), up=(0,0,1)',
                scene_semantics='Fresh stationary stock SBR; not historical deformation/velocity replay')
