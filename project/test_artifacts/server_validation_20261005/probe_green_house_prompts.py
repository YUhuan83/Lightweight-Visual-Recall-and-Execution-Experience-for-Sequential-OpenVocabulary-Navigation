"""Compare green-house and gas-station masks on a stationary live Town frame."""
import json
import re
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_search_msgs.msg import ObjectMaskWithTf


OUT = Path('/home/ly/town_memory_validation_20261005/green_house_prompt_masks')
OUT.mkdir(parents=True, exist_ok=True)
PROMPTS = [
    'green house',
    'green residential house',
    'green detached house',
    'house with a green exterior',
    'gas station',
]


class Probe(Node):
    def __init__(self):
        super().__init__('green_house_prompt_mask_probe')
        self.pub = self.create_publisher(String, '/spot1/object_search/query', 10)
        self.query = None
        self.frames = deque(maxlen=50)
        self.rows = []
        self.image_count = 0
        self.mask_count = 0
        self.latest_frame_stamp = None
        self.pose = None
        self.speed = None
        self.create_subscription(
            Image, '/spot1/realsense/front/color/image_raw', self.on_image,
            qos_profile_sensor_data)
        self.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', self.on_mask, 10)
        self.create_subscription(String, '/wildos/native_control_status', self.on_native, 10)

    def on_image(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        rgb = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames.append((stamp, frame.copy()))
        self.image_count += 1
        self.latest_frame_stamp = stamp

    def on_native(self, msg):
        try:
            data = json.loads(msg.data)
            self.pose = data.get('position')
            self.speed = data.get('speed')
        except (ValueError, TypeError):
            pass

    def on_mask(self, msg):
        self.mask_count += 1
        if msg.query != self.query:
            return
        dims = [dim.size for dim in msg.object_mask.layout.dim]
        if not dims:
            return
        mask = np.asarray(msg.object_mask.data, dtype=np.uint8).reshape(dims)[0, 0] > 0
        if not mask.any() or not self.frames:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame_stamp, frame = min(self.frames, key=lambda item: abs(item[0] - stamp))
        if mask.shape != frame.shape[:2]:
            mask = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]),
                              interpolation=cv2.INTER_NEAREST) > 0
        overlay = frame.copy()
        overlay[mask] = (0.45 * overlay[mask] +
                         0.55 * np.array([0, 255, 0])).astype(np.uint8)
        ys, xs = np.where(mask)
        slug = re.sub(r'[^a-z0-9]+', '_', self.query.lower()).strip('_')
        cv2.imwrite(str(OUT / f'{slug}_rgb.png'), frame)
        cv2.imwrite(str(OUT / f'{slug}_mask.png'), overlay)
        self.rows.append({
            'query': self.query,
            'mask_pixels': int(mask.sum()),
            'bbox': [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
            'mask_stamp': stamp,
            'nearest_frame_stamp': frame_stamp,
        })

    def run_prompt(self, query, seconds=8):
        self.query = query
        before = len(self.rows)
        self.pub.publish(String(data=query))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.rows[before:]


rclpy.init()
probe = Probe()
try:
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and not probe.frames:
        rclpy.spin_once(probe, timeout_sec=0.1)
    start_pose = probe.pose
    results = []
    for prompt in PROMPTS:
        masks = probe.run_prompt(prompt)
        results.append({'prompt': prompt, 'masks': masks})
    report = {
        'prompts': results,
        'start_pose': start_pose,
        'end_pose': probe.pose,
        'speed': probe.speed,
        'image_messages_received': probe.image_count,
        'mask_messages_received': probe.mask_count,
        'latest_frame_stamp': probe.latest_frame_stamp,
        'motion_during_probe_m': None,
    }
    if start_pose and probe.pose:
        report['motion_during_probe_m'] = float(np.linalg.norm(
            np.asarray(start_pose[:2], dtype=float) -
            np.asarray(probe.pose[:2], dtype=float)))
    (OUT / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
finally:
    probe.destroy_node()
    rclpy.shutdown()
