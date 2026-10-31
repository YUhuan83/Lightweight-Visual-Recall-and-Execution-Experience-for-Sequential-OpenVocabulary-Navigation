import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from object_search_msgs.msg import ObjectMaskWithTf

rclpy.init()
n = Node('full_town_test_peek')
s = {}
events = []
for key, kind, topic in [('camera', Image, '/spot1/realsense/front/color/image_raw'),
                          ('status', String, '/wildos/plan_status'),
                          ('native', String, '/wildos/native_control_status'),
                          ('memory', String, '/spot1/remembered_object_candidates'),
                          ('waypoint', PoseStamped, '/spot1/imgnav_waypoint'),
                          ('mask', ObjectMaskWithTf, '/spot1/object_mask')]:
    def receive(msg, key=key):
        s[key] = msg
        if key == 'mask':
            events.append({'query': msg.query, 'pixels': sum(msg.object_mask.data)})
    n.create_subscription(kind, topic, receive, 10 if key != 'camera' else 1)
pub = n.create_publisher(String, '/spot1/object_search/query', 10)
end = time.monotonic() + 12
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.1)
out = Path('/home/ly/full_town_memory_20260930')
if 'camera' in s:
    m = s['camera']
    a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height, m.width, 3)
    cv2.imwrite(str(out / 'initial_view.png'), cv2.cvtColor(a, cv2.COLOR_RGB2BGR))
pub.publish(String(data='gas station'))
report = {'topics': list(s), 'masks': events}
for key in ('status', 'native', 'memory'):
    if key in s:
        report[key] = json.loads(s[key].data)
if 'waypoint' in s:
    p = s['waypoint'].pose.position
    report['waypoint'] = [p.x, p.y, p.z]
out.joinpath('peek.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
n.destroy_node()
rclpy.shutdown()
