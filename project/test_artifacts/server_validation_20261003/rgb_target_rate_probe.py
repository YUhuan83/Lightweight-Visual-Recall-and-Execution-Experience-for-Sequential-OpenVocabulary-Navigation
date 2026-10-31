"""Bounded stationary stream verification; no navigation success claim."""
from pathlib import Path
import json,time,signal,subprocess
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from beamngpy import BeamNGpy
root=Path('/home/ly/town_memory_validation_20261003/rgb_arrival_fix')
root.mkdir(exist_ok=True)
video_path=root/'rgb_target_stream_stationary_maskcheck.mp4'
if video_path.exists():raise RuntimeError('Preserve existing recording')
rclpy.init();node=Node('rgb_target_rate_probe');samples=defaultdict(list)
def observe(key,m):
 samples[key].append((time.monotonic(),m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.width,m.height))
for topic,key in [('/spot1/realsense/front/color/image_raw','camera'),('/wildos/viz/rgb_target','rgb_target')]:
 node.create_subscription(Image,topic,lambda m,key=key:observe(key,m),qos_profile_sensor_data)
query=node.create_publisher(String,'/spot1/object_search/query',10)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
b.control.resume()
f=(root/'video_probe_ffmpeg_maskcheck.log').open('w')
recorder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab','-filter_threads','1','-video_size','1280x1024','-framerate','15','-i',':99','-t','25','-c:v','libx264','-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',str(video_path)],stdout=f,stderr=subprocess.STDOUT)
try:
 started=time.monotonic()
 while time.monotonic()-started<22:
  rclpy.spin_once(node,timeout_sec=.05)
  if int((time.monotonic()-started)*10)%10==0:query.publish(String(data='gas station'))
 result={'scope':'stationary visualization stream only','camera_request_hz':10,'capture_fps':15,'navigation_test':False,'topics':{}}
 for key,rows in samples.items():
  selected=[r for r in rows if r[0]>started+3]
  span=selected[-1][0]-selected[0][0] if len(selected)>1 else 0
  result['topics'][key]={'frames':len(selected),'received_hz':(len(selected)-1)/span if span else 0,'unique_stamps':len(set(r[1] for r in selected)),'width':selected[-1][2] if selected else 0,'height':selected[-1][3] if selected else 0}
 (root/'rate_probe_maskcheck.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
 if not all(result['topics'].get(k,{}).get('received_hz',0)>=7 for k in ('camera','rgb_target')):raise RuntimeError('Camera/display cadence below7Hz')
finally:
 b.control.pause();b.disconnect()
 if recorder.poll() is None:recorder.send_signal(signal.SIGINT);recorder.wait(timeout=10)
 f.close();node.destroy_node();rclpy.shutdown()
