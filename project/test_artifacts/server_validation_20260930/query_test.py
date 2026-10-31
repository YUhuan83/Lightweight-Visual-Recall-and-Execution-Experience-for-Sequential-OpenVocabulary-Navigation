import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as Route
from beamng_msgs.msg import VehicleControl

rclpy.init()
n = Node('same_process_query_validation')
pub = n.create_publisher(String, '/spot1/object_search/query', 10)
events = []
phase = None
last_status = None
unsafe = False
def status(msg):
    global last_status
    data = json.loads(msg.data)
    if data != last_status:
        events.append({'t': time.monotonic(), 'phase': phase, 'topic': 'status', 'data': data})
        print(json.dumps(events[-1]), flush=True)
        last_status = data
def pose(msg, topic):
    p = msg.pose.position
    events.append({'t': time.monotonic(), 'phase': phase, 'topic': topic, 'xy': [p.x,p.y], 'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9})
    print(json.dumps(events[-1]), flush=True)
def control(msg):
    global unsafe
    unsafe |= msg.throttle > 0 or msg.brake < .99
n.create_subscription(String, '/wildos/plan_status', status, 10)
n.create_subscription(PoseStamped, '/spot1/remembered_object_goal', lambda m: pose(m,'memory'), 10)
n.create_subscription(PoseStamped, '/spot1/graphnav_destination', lambda m: pose(m,'destination'), 10)
n.create_subscription(VehicleControl, '/control', control, 10)
def wait(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n, timeout_sec=.1)
wait(2)
for query, seconds in [('trash can',15),('tree',15),('gas station',15),('tree',15)]:
    phase = query
    events.append({'t':time.monotonic(),'phase':phase,'topic':'query_sent'})
    print(json.dumps(events[-1]), flush=True)
    pub.publish(String(data=query))
    wait(seconds)
report = {'events':events,'control_remained_braked':not unsafe,'wildos_pid_retained':Path('/proc/1575639').exists()}
Path('/home/ly/navigation_validation_20260930_logs/query_test.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'control_remained_braked':not unsafe,'wildos_pid_retained':report['wildos_pid_retained']}),flush=True)
n.destroy_node()
rclpy.shutdown()
