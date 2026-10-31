"""Bounded read-only camera review clip, started on explicit user request."""
import time,json,signal
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile,ReliabilityPolicy
r=Path('/home/ly/wildos-console/results');s=sorted(r.glob('session_*'))[-1];g=s/'goal_001'
folder=s/'requested_video';folder.mkdir(exist_ok=True)
out=folder/'front_camera.mp4';assert not out.exists()
rclpy.init();node=rclpy.create_node('requested_camera_review_recorder')
state={};samples=[];started=time.time();origin=time.monotonic();writer=None
stop=False

def receive(m):
 ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 if m.encoding not in ('rgb8','bgr8'):return
 if ns<=state.get('ns',0):return
 a=np.ndarray((m.height,m.width,3),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1)).copy()
 if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
 state.update(ns=ns,image=a,received=time.monotonic())
node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
try:
 deadline=origin+600.;next_frame=origin
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.01)
  now=time.monotonic()
  if now<next_frame:continue
  if not state:
   next_frame=now+.1;continue
  image=state['image'].copy();age=(node.get_clock().now().nanoseconds-state['ns'])/1e9
  cv2.rectangle(image,(0,0),(image.shape[1],54),(0,0,0),-1)
  label='LIVE' if -.05<=age<=.75 else 'STALE'
  cv2.putText(image,f'{label} | clip {now-origin:.1f}s | RGB age {age:.3f}s',(10,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0) if label=='LIVE' else (0,0,255),1)
  cv2.putText(image,time.strftime('%Y-%m-%d %H:%M:%S')+' | green residential house',(10,45),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
  if writer is None:
   writer=cv2.VideoWriter(str(out),cv2.VideoWriter_fourcc(*'mp4v'),10.,(image.shape[1],image.shape[0]));assert writer.isOpened()
   (folder/'started.json').write_text(json.dumps(dict(started_wall=started,first_source_stamp_ns=state['ns'])))
  writer.write(image);samples.append(dict(elapsed=now-origin,source_stamp_ns=state['ns'],age_sec=age))
  next_frame+=.1
  if now-next_frame>.2:next_frame=now+.1
  if (g/'result.json').exists():break
finally:
 if writer:writer.release()
 node.destroy_node();rclpy.shutdown()
 report=dict(path=str(out),session=str(s),started_wall=started,ended_wall=time.time(),partial_trial=False,recording_started_before_query=True,
  scope='actual live RGB camera only; no RViz screen, no inferred target or navigation commands',frames=len(samples),frame_samples=samples)
 out.with_suffix('.json').write_text(json.dumps(report,indent=2))
 print(json.dumps({k:v for k,v in report.items() if k!='frame_samples'}))
