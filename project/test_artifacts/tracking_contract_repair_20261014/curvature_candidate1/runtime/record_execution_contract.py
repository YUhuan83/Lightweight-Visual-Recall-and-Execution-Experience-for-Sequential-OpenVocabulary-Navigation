"""Bounded DDS observer, no publishers, control or simulator mutation."""
import json,time,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,PointStamped
from nav_msgs.msg import Odometry,Path as Route
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict

out=Path(sys.argv[1]);out.mkdir(exist_ok=False)
rclpy.init();node=Node('execution_contract_observer')
log=(out/'messages.jsonl').open('x');count=0;last_odom_stamp=None;requested=False
def save(topic,msg):
    global count,last_odom_stamp,requested
    row={'topic':topic,'received_ros_ns':node.get_clock().now().nanoseconds,
        'received_monotonic_ns':time.monotonic_ns(),'message':message_to_ordereddict(msg)}
    log.write(json.dumps(row,separators=(',',':'))+'\n');count+=1
    if count%32==0:log.flush()
    if topic=='/spot1/odom':last_odom_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    if len(sys.argv)>2 and sys.argv[2]=='--debug-snapshot' and topic=='/received_global_plan' and last_odom_stamp:
        now=row['received_ros_ns']/1e9;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not requested and now-stamp>.75 and now-last_odom_stamp<.3:
            requested=True
            request=out.parent/'resume_session3/snapshot.request'
            request.write_text(json.dumps({'received_ros_ns':row['received_ros_ns'],
                'path_stamp_sec':stamp,'odom_stamp_sec':last_odom_stamp,
                'scope':'Debugger interrupt may perturb this diagnostic; not acceptance'}))
for cls,topic in [(Odometry,'/spot1/odom'),(Twist,'/wildos/cmd_vel'),
    (String,'/wildos/control_status'),(String,'/wildos/native_control_status'),
    (String,'/wildos/plan_status'),(Route,'/received_global_plan'),
    (PointStamped,'/lookahead_point'),(Route,'/wildos/planned_path'),
    (TFMessage,'/tf')]:
    node.create_subscription(cls,topic,lambda m,t=topic:save(t,m),qos_profile_sensor_data)
(out/'ready.json').write_text(json.dumps({'pid':__import__('os').getpid()}))
began=time.monotonic()
while time.monotonic()-began<125:rclpy.spin_once(node,timeout_sec=.1)
log.close();node.destroy_node();rclpy.shutdown()
(out/'result.json').write_text(json.dumps({'scope':'Observer only','messages':count}))
