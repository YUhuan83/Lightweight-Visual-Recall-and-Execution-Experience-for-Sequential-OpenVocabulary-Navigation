"""Read-only capture: require advancing, fresh source timestamps. Never arrival evidence."""
import json,time
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image
root=Path('/home/ly/wildos-console/results')
session=sorted(root.glob('session_*'))[-1]
out=session/'goal_001/diagnostic_mid_search.png'
rclpy.init();node=rclpy.create_node('console_fresh_frame_diagnostic')
frames=[]
def receive(msg):
    stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    age=(node.get_clock().now().nanoseconds-stamp)/1e9
    if -.05<=age<=.75 and (not frames or stamp>frames[-1][0]):frames.append((stamp,age,msg))
node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',receive,10)
try:
    until=time.monotonic()+10
    while len(frames)<3 and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
    assert len(frames)>=3,'No three advancing fresh camera frames'
    stamp,age,msg=frames[-1]
    assert msg.encoding in ('rgb8','bgr8')
    pixels=np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=msg.data,strides=(msg.step,3,1)).copy()
    if msg.encoding=='rgb8':pixels=cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
    assert cv2.imwrite(str(out),pixels)
    meta=dict(is_arrival_evidence=False,query='green residential house',capture_wall=time.time(),
        source_stamp_ns=stamp,frame_id=msg.header.frame_id,source_age_sec=age,
        advancing_frame_stamps=[f[0] for f in frames])
    out.with_suffix('.json').write_text(json.dumps(meta,indent=2))
    print(json.dumps(dict(path=str(out),**meta)))
finally:node.destroy_node();rclpy.shutdown()
