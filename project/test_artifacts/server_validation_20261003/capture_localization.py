"""Save one actual synchronized mask/LiDAR projection for diagnosis."""
from pathlib import Path
from collections import deque
import json
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from object_search_msgs.msg import ObjectMaskWithTf
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from beamng_wildos_adapter.pointcloud import decode_xyz
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth

ROOT = Path('/home/ly/town_memory_validation_20261003')
OUT = ROOT
if len(sys.argv)>1:
    if not sys.argv[1].replace('_','').isalnum():
        raise ValueError('Invalid output directory')
    OUT = ROOT/sys.argv[1]
    OUT.mkdir(exist_ok=True)
rclpy.init()
n = rclpy.create_node('capture_station_projection_20261003')
tf = Buffer()
listener = TransformListener(tf,n)
scans = deque(maxlen=60)
pending = []
n.create_subscription(PointCloud2, '/spot1/ouster/front/points_filtered', scans.append, qos_profile_sensor_data)
n.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', pending.append, qos_profile_sensor_data)
stamp = lambda m: m.header.stamp.sec*10**9+m.header.stamp.nanosec
saved = False
end=time.monotonic()+16
while time.monotonic()<end and not saved:
    rclpy.spin_once(n,timeout_sec=.05)
    if not pending or not scans:continue
    mask_msg=pending.pop()
    pending.clear()
    scan=min(scans,key=lambda s:abs(stamp(s)-stamp(mask_msg)))
    if abs(stamp(scan)-stamp(mask_msg))>.2*1e9:continue
    try:
        transform=tf.lookup_transform('spot1/odom',scan.header.frame_id,rclpy.time.Time.from_msg(scan.header.stamp))
    except Exception:continue
    p=decode_xyz(scan)
    t=transform.transform
    q=t.rotation
    rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    world=p@rotation.T+[t.translation.x,t.translation.y,t.translation.z]
    cam=mask_msg.cam_transforms.transforms[0].transform
    q=cam.rotation
    rwc=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    twc=np.array([cam.translation.x,cam.translation.y,cam.translation.z])
    camera=(world-twc)@rwc
    k=np.asarray(mask_msg.cam_infos[0].k).reshape(3,3)
    dims=[d.size for d in mask_msg.object_mask.layout.dim]
    mask=np.asarray(mask_msg.object_mask.data).reshape(dims)[0,0].astype(bool)
    quality=foreground_depth(camera,k,mask)
    np.savez_compressed(OUT/'station_localization_projection.npz',points_cam=camera,k=k,mask=mask,
                        rotation=rwc,translation=twc,stamp_ns=stamp(mask_msg))
    selected=camera[np.isfinite(camera).all(axis=1)&(camera[:,2]>.1)]
    pix=selected@k.T
    pix=pix[:,:2]/pix[:,2:]
    h,w=mask.shape
    good=(pix[:,0]>=0)&(pix[:,0]<w)&(pix[:,1]>=0)&(pix[:,1]<h)
    selected,pix=selected[good],pix[good].astype(int)
    good=mask[pix[:,1],pix[:,0]]
    selected,pix=selected[good],pix[good]
    order=np.argsort(selected[:,2])
    _,unique=np.unique(pix[order,1]*w+pix[order,0],return_index=True)
    selected,pix=selected[order[unique]],pix[order[unique]]
    order=np.argsort(selected[:,2]);selected,pix=selected[order],pix[order]
    cuts=np.flatnonzero(np.diff(selected[:,2])>np.maximum(.6,.04*selected[:-1,2]))+1
    clusters=np.split(np.arange(len(selected)),cuts)
    info=dict(query=mask_msg.query,total_points=len(camera),masked_unique=len(selected),positive_pixels=int(mask.sum()),
              source_skew_sec=abs(stamp(scan)-stamp(mask_msg))*1e-9,
              mask_age_sec=(n.get_clock().now().nanoseconds-stamp(mask_msg))*1e-9,quality=quality,
              clusters=[dict(count=len(c),support=len(c)/len(selected),median_depth=float(np.median(selected[c,2])),
                   depth_sigma=float(1.4826*np.median(np.abs(selected[c,2]-np.median(selected[c,2])))),
                   pixel_extent=np.ptp(pix[c],axis=0).tolist()) for c in clusters if len(c)>=3])
    def encode(x):
        return x.tolist() if isinstance(x,np.ndarray) else x.item() if isinstance(x,np.generic) else str(x)
    (OUT/'station_localization_projection.json').write_text(json.dumps(info,indent=2,default=encode))
    print(json.dumps(info,indent=2,default=encode),flush=True)
    saved=True
n.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if saved else 1)
