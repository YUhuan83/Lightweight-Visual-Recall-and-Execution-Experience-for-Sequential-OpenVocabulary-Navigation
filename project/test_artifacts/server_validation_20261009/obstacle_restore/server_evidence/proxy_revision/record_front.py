from pathlib import Path
import sys,time,json,signal
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
out=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('component_front_recorder')
writer=cv2.VideoWriter(str(out/'front_camera.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5,(960,640))
assert writer.isOpened()
log=(out/'front_camera_frames.jsonl').open('w');state={};started=time.monotonic();count=0;stop=False
signal.signal(signal.SIGINT,lambda *args:globals().__setitem__('stop',True))
def cb(m):
 channels=3 if m.encoding in ('rgb8','bgr8') else 4 if m.encoding in ('rgba8','bgra8') else 1
 a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*channels].reshape(m.height,m.width,channels)
 if m.encoding=='rgb8': a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
 elif m.encoding=='rgba8': a=cv2.cvtColor(a,cv2.COLOR_RGBA2BGR)
 elif m.encoding=='bgra8': a=cv2.cvtColor(a,cv2.COLOR_BGRA2BGR)
 elif channels==1:a=cv2.cvtColor(a,cv2.COLOR_GRAY2BGR)
 state.update(frame=a.copy(),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec)
 (out/'camera_ready').touch()
node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',cb,qos_profile_sensor_data)
next_frame=started
try:
 while not stop and time.monotonic()-started<65:
  rclpy.spin_once(node,timeout_sec=.02)
  now=time.monotonic()
  if 'frame' in state and now>=next_frame:
   writer.write(cv2.resize(state['frame'],(960,640)))
   log.write(json.dumps({'index':count,'elapsed':now-started,'stamp_ns':state['stamp_ns']})+'\n');count+=1;next_frame=now+.2
finally:
 writer.release();log.close();node.destroy_node();rclpy.shutdown()
 (out/'recording.json').write_text(json.dumps({'frames':count,'elapsed':time.monotonic()-started}))
