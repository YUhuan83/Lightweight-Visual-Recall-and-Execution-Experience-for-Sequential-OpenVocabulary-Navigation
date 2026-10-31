"""Pure coordinate calculation for engine bbox + Native list-valued vectors."""
from types import SimpleNamespace
import numpy as np
from beamng_wildos_adapter.geometry import body_rotation

def rear_axle_extents(corners,position,direction,up,offset):
    corners=np.asarray(corners,dtype=float)
    position=np.asarray(position,dtype=float);offset=np.asarray(offset,dtype=float)
    direction=np.asarray(direction,dtype=float);up=np.asarray(up,dtype=float)
    if (corners.shape!=(8,3) or any(v.shape!=(3,) for v in (position,offset,direction,up))
            or not all(np.isfinite(v).all() for v in (corners,position,offset,direction,up))):
        raise ValueError('Expected finite eight bbox corners and three-vector geometry')
    vector=lambda values:SimpleNamespace(**dict(zip(('x','y','z'),values)))
    rotation=body_rotation(vector(direction),vector(up))
    local=(corners-position)@rotation-offset
    extents=dict(front=float(local[:,0].max()),rear=float(-local[:,0].min()),
                 half_width=float(np.abs(local[:,1]).max()),
                 z_min=float(local[:,2].min()),z_max=float(local[:,2].max()))
    return local,extents
