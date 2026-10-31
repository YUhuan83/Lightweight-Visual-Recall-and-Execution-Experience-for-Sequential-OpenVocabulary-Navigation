"""Read-only actual sensor DDS load and independent source receipt comparison."""
import json
from pathlib import Path
import signal
import statistics
import sys
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String
halt=threading.Event()
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:halt.set())
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node=rclpy.create_node('read_only_native_delay_sensor_load')
counts={};sources=[];native=[]
def count(kind,msg):counts[kind]=counts.get(kind,0)+1
def source(msg):
    wall=time.time_ns();v=json.loads(msg.data)
    sources.append(dict(sequence=v['sequence'],issued_wall_ns=v['issued_wall_ns'],received_wall_ns=wall))
node.create_subscription(Image,'/vehicles/ego/sensors/front_cam/colour',lambda m:count('image',m),1)
node.create_subscription(PointCloud2,'/vehicles/ego/sensors/front_lidar',lambda m:count('lidar',m),1)
node.create_subscription(String,'/wildos/native_control_command',source,1)
node.create_subscription(String,'/wildos/native_control_status',lambda m:native.append(json.loads(m.data)),10)
executor=SingleThreadedExecutor();executor.add_node(node)
try:
    while not halt.is_set():executor.spin_once(timeout_sec=.05)
finally:
    executor.shutdown();node.destroy_node();rclpy.shutdown()
    Path(sys.argv[1]).write_text(json.dumps(dict(counts=counts,sources=sources,native=native),indent=2)+'\n')
