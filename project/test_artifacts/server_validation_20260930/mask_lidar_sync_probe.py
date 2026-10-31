import json
from pathlib import Path
import time
import rclpy
from beamngpy import BeamNGpy
from rclpy.node import Node
from object_search_msgs.msg import ObjectMaskWithTf
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
b=BeamNGpy('127.0.0.1',25252)
b.open(launch=False)
b.control.resume()  # The existing actuator and FollowPath remain disabled.
rclpy.init()
n=Node('mask_lidar_sync_probe')
s={'masks':[], 'lidar':[]}
def receive(m, key):
    v={'received':time.time(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
    if key=='masks':v.update(query=m.query, odom_stamp=m.odom.header.stamp.sec+m.odom.header.stamp.nanosec*1e-9)
    s[key].append(v)
n.create_subscription(ObjectMaskWithTf,'/spot1/object_mask',lambda m:receive(m,'masks'),10)
n.create_subscription(PointCloud2,'/spot1/ouster/front/points_filtered',lambda m:receive(m,'lidar'),qos_profile_sensor_data)
end=time.monotonic()+10
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
b.control.pause()
b.disconnect()
for m in s['masks']:
    m['nearest_lidar_stamp_gap']=min((abs(m['stamp']-l['stamp']) for l in s['lidar']),default=None)
    m['mask_delay_from_latest_lidar']=max((l['stamp'] for l in s['lidar'] if l['received']<=m['received']), default=m['stamp'])-m['stamp']
Path('/home/ly/full_town_memory_20260930/mask_lidar_sync.json').write_text(json.dumps(s,indent=2))
print(json.dumps({'masks':s['masks'],'lidar_count':len(s['lidar']),'lidar_start':s['lidar'][:1],'lidar_end':s['lidar'][-1:]},indent=2))
n.destroy_node();rclpy.shutdown()
