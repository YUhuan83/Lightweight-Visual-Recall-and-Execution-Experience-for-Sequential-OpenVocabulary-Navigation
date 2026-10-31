"""Compare bus-stop prompts against the stationary live Town RGB frame."""
import json, os, re, time
from collections import deque
from pathlib import Path

runtime = Path('/home/ly/full_town_memory_20261002/runtime_env.json')
os.environ.update(json.loads(runtime.read_text()))
os.environ.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1', ROS2CLI_NO_DAEMON='1')
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_search_msgs.msg import ObjectMaskWithTf

OUT = Path('/home/ly/town_memory_validation_20261005/prompt_masks_20261005')
OUT.mkdir(parents=True, exist_ok=True)
PROMPTS = [
    'bus stop sign with a bus icon',
    'blue bus stop sign with a bus symbol',
    'bus shelter with a bench and bus stop sign',
]

class Probe(Node):
    def __init__(self):
        super().__init__('bus_stop_prompt_mask_probe')
        self.pub = self.create_publisher(String, '/spot1/object_search/query', 10)
        self.query = None
        self.frames = deque(maxlen=50)
        self.rows = []
        self.pose = None
        self.create_subscription(Image, '/spot1/realsense/front/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', self.on_mask, 10)
        self.create_subscription(String, '/wildos/native_control_status', self.on_native, 10)

    def on_image(self, msg):
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3]
        frame=cv2.cvtColor(a.reshape(msg.height,msg.width,3),cv2.COLOR_RGB2BGR)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.frames.append((stamp,frame.copy()))

    def on_native(self,msg):
        try:
            d=json.loads(msg.data); self.pose=d.get('position'); self.speed=d.get('speed')
        except Exception: pass

    def on_mask(self,msg):
        if msg.query != self.query: return
        dims=[d.size for d in msg.object_mask.layout.dim]
        mask=np.asarray(msg.object_mask.data,dtype=np.uint8).reshape(dims)[0,0]>0
        count=int(mask.sum())
        if not count or not self.frames: return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        frame_stamp,frame=min(self.frames,key=lambda x:abs(x[0]-stamp))
        if mask.shape != frame.shape[:2]:
            mask=cv2.resize(mask.astype(np.uint8),(frame.shape[1],frame.shape[0]),interpolation=cv2.INTER_NEAREST)>0
        overlay=frame.copy(); overlay[mask]=(0.45*overlay[mask]+0.55*np.array([0,255,0])).astype(np.uint8)
        ys,xs=np.where(mask)
        slug=re.sub(r'[^a-z0-9]+','_',self.query.lower()).strip('_')
        cv2.imwrite(str(OUT/f'{slug}_rgb.png'),frame)
        cv2.imwrite(str(OUT/f'{slug}_mask.png'),overlay)
        self.rows.append({'query':self.query,'mask_pixels':count,
                          'bbox':[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())],
                          'mask_stamp':stamp,'nearest_frame_stamp':frame_stamp})

    def run_prompt(self,q,seconds=7):
        self.query=q
        before=len(self.rows)
        self.pub.publish(String(data=q))
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1)
        if len(self.rows)==before and self.frames:
            slug=re.sub(r'[^a-z0-9]+','_',q.lower()).strip('_')
            cv2.imwrite(str(OUT/f'{slug}_rgb.png'),self.frames[-1][1])
        return self.rows[before:]

rclpy.init()
p=Probe()
try:
    if not p.frames:
        end=time.monotonic()+5
        while time.monotonic()<end and not p.frames: rclpy.spin_once(p,timeout_sec=.1)
    start_pose=p.pose
    all_results=[]
    for q in PROMPTS:
        result=p.run_prompt(q)
        all_results.append({'prompt':q,'masks':result})
    report={'prompts':all_results,'start_pose':start_pose,'end_pose':p.pose,
            'speed':getattr(p,'speed',None),'motion_during_probe_m':None}
    if start_pose and p.pose:
        report['motion_during_probe_m']=float(np.linalg.norm(np.asarray(start_pose[:2])-np.asarray(p.pose[:2])))
    (OUT/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
finally:
    p.destroy_node(); rclpy.shutdown()
