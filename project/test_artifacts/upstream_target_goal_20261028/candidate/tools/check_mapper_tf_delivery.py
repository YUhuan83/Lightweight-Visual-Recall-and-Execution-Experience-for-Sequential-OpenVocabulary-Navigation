"""Real DDS controlled topic ordering; synthetic maps, no simulator motion."""
import argparse,copy,json,pathlib,os,subprocess,signal,time
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
from pose_graph_msgs.msg import PoseGraph,Node as GraphNode
parser=argparse.ArgumentParser();parser.add_argument('--output',type=pathlib.Path,required=True);parser.add_argument('--expect-recovery',action='store_true');a=parser.parse_args();assert not a.output.exists()
# Start the executable directly: terminating a ros2-run wrapper can leave its
# mapper child alive and contaminate the next DDS ordering experiment.
mapper = pathlib.Path(get_package_prefix('graphnav_mapper'))/'lib/graphnav_mapper/mapper_node'
assert mapper.is_file() and os.access(mapper, os.X_OK)
rclpy.init();n=Node('mapper_tf_delivery_probe')
# Publish before starting the mapper: its late-joining static listener must
# still receive the retained reliable calibration, alongside best-effort poses.
static_pub=n.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,
 durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
static_tf=TransformStamped();static_tf.header.frame_id='probe/graph';static_tf.child_frame_id='probe/root';static_tf.header.stamp=n.get_clock().now().to_msg();static_tf.transform.rotation.w=1.
static_pub.publish(TFMessage(transforms=[static_tf]))
log=a.output.with_suffix('.log').open('w');proc=subprocess.Popen([str(mapper),'--ros-args','-r','__ns:=/tf_delivery','-p','use_occupancy_input:=false'],stdout=log,stderr=subprocess.STDOUT);dbg=[];cases=[]
n.create_subscription(GridMap,'/tf_delivery/dbg_map',lambda m:dbg.append(dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,layers=list(m.layers),frame=m.header.frame_id,received_at=time.monotonic())),10)
tf=n.create_publisher(TFMessage,'/tf',QoSProfile(depth=100));maps=n.create_publisher(GridMap,'/tf_delivery/traversability',1);poses=n.create_publisher(PoseGraph,'/tf_delivery/pose_graph',10);odoms=n.create_publisher(Odometry,'/tf_delivery/odometry',1)
def spin(sec):
 t=time.monotonic()
 while time.monotonic()-t<sec:rclpy.spin_once(n,timeout_sec=.01)
def stamp(ns):return rclpy.time.Time(nanoseconds=ns).to_msg()
def send_tf(ns,x=0.):
 t=TransformStamped();t.header.frame_id='probe/root';t.child_frame_id='probe/terrain';t.header.stamp=stamp(ns);t.transform.translation.x=x;t.transform.rotation.w=1.;tf.publish(TFMessage(transforms=[t]))
def grid(ns):
 m=GridMap();m.header.frame_id='probe/terrain';m.header.stamp=stamp(ns);m.info.resolution=.5;m.info.length_x=20.;m.info.length_y=20.;m.info.pose.orientation.w=1.;m.layers=['traversability','unknown','elevation','elevation_valid'];m.basic_layers=['traversability','unknown']
 for v in [0.,0.,0.,1.]:
  d=Float32MultiArray();d.layout.dim=[MultiArrayDimension(label='column_index',size=40,stride=1600),MultiArrayDimension(label='row_index',size=40,stride=40)];d.data=[v]*1600;m.data.append(d)
 return m
try:
 deadline=time.monotonic()+8
 while maps.get_subscription_count()<1 and time.monotonic()<deadline:spin(.1)
 assert maps.get_subscription_count()==1
 pg=PoseGraph();pg.header.frame_id='probe/graph';node=GraphNode();node.pose.orientation.w=1.;node.uuid.id[0]=1;pg.nodes=[node];pg.current_node=node.uuid
 poses.publish(pg);spin(.3);poses.publish(pg);spin(.3)
 base=n.get_clock().now().nanoseconds
 odom=Odometry();odom.header.frame_id='probe/graph';odom.header.stamp=stamp(base);odom.pose.pose.orientation.w=1.;odoms.publish(odom);spin(.3)
 for i,delay in enumerate([.03,.1,.18,None]):
  previous=base+(i*2)*10**9;requested=previous+10**9;send_tf(previous);spin(.1);m=grid(requested);start=time.monotonic();maps.publish(m)
  if delay is not None:spin(delay);send_tf(requested,1.)
  spin(.45)
  found=[d for d in dbg if d['stamp_ns']==requested and d['layers']]
  cases.append(dict(delay_sec=delay,requested_ns=requested,accepted=len(found)>0,elapsed_to_accept=(found[0]['received_at']-start if found else None)))
  # The same valid input must work once its exact acquisition transform exists.
  send_tf(requested,1.);spin(.1);maps.publish(m);spin(.4)
  cases[-1]['ordinary_recovery']=any(d['stamp_ns']==requested and d['layers'] for d in dbg)
 expected=[a.expect_recovery]*3+[False]
 passed=[c['accepted'] for c in cases]==expected and all(c['ordinary_recovery'] for c in cases)
 report=dict(scope='Synthetic real DDS delayed/missing dynamic TF through a retained static calibration published before mapper startup; no BeamNG, Nav2 action or search',passed=passed,expect_recovery=a.expect_recovery,static_calibration_published_before_mapper=True,cases=cases,debug_messages=dbg)
 a.output.write_text(json.dumps(report,indent=2));print(json.dumps(dict(passed=passed,cases=cases)))
finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=8)
 except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 log.close();n.destroy_node();rclpy.shutdown()
