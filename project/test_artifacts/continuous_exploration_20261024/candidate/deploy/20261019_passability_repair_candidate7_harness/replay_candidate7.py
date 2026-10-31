"""Actual installed Mapper replay in an isolated ROS domain; no vehicle/control."""
from pathlib import Path
import hashlib
import json
import math
import os
import signal
import sqlite3
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import deserialize_message,serialize_message
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from graphnav_msgs.msg import NavigationGraph
from pose_graph_msgs.msg import PoseGraph,Node as PoseGraphNode
from ament_index_python.packages import get_package_prefix
from beamng_wildos_adapter.terrain import terrain_layers, grid_map_values
from wildos_console import OwnedProcesses,proc_identity

HARNESS=Path(__file__).resolve().parent
BASE=HARNESS.parent
OUT=BASE/('replay_candidate7_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
assert os.environ['ROS_DOMAIN_ID']=='200'
owned=OwnedProcesses(OUT,BASE);halt=False
def interrupted(*_):
    global halt
    halt=True
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node=Node('mapper_replay_audit');graph=None;graph_at=0.;graph_count=0
report=dict(scope=__doc__,completed=False,errors=[],ros_domain=200,beamng_started=False,
    control_publishers_created=False,production_modified=False,complete_navigation_passed=False,snapshots=[])
def default(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError(type(value).__name__)
def plain(message):return json.loads(json.dumps(message_to_ordereddict(message),default=default))
def ns(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec
def observed_ns(value):return value.observed_at.sec*10**9+value.observed_at.nanosec
def on_graph(message):
    global graph,graph_at,graph_count
    graph=message;graph_at=time.monotonic();graph_count+=1
node.create_subscription(NavigationGraph,'/spot1/nav_graph',on_graph,10)
clock_pub=node.create_publisher(Clock,'/clock',10)
tf_pub=node.create_publisher(TFMessage,'/tf',QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
odom_pub=node.create_publisher(Odometry,'/spot1/odom',10)
terrain_pub=node.create_publisher(GridMap,'/spot1/local_terrain',10)
pose_pub=node.create_publisher(PoseGraph,'/spot1/pose_graph',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if halt:raise InterruptedError('Replay interrupted')
        owned.healthy();rclpy.spin_once(node,timeout_sec=.02)
def wait_for(predicate,seconds,label):
    until=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>until:raise TimeoutError(label)
        spin(.02)
def dist_segment(point,a,b):
    a=np.asarray(a,float);b=np.asarray(b,float);p=np.asarray(point,float)
    d=b-a;t=float(np.clip((p-a)@d/max(d@d,1e-12),0.,1.))
    return float(np.linalg.norm(p-a-t*d))
try:
    source=Path('/home/ly/contact_frame_audit_20261019/frame_20261019_084254')
    assert json.loads((source/'result.json').read_text())['component_closed']
    db=next((source/'interfaces').glob('*.db3'))
    sha=hashlib.sha256(db.read_bytes()).hexdigest()
    assert sha=='0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc'
    report['source_database']=dict(path=str(db),sha256=sha)
    c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in c.execute('SELECT id,name,type FROM topics')}
    def load(name):
        i,typ=topics[name];cls=get_message(typ)
        return [deserialize_message(data,cls) for data, in c.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(i,))]
    odoms={ns(m):m for m in load('/spot1/odom')}
    transforms={}
    for m in load('/tf'):
        for t in m.transforms:transforms.setdefault(ns(t),[]).append(t)
    c.close()
    # Regenerate the terrain input with the candidate7 adapter instead of
    # replaying the old recorded layers. The level points are relative to the
    # rear axle, the same frame the adapter publishes.
    arrays=np.load(BASE/'replay_inputs.npz',allow_pickle=False)
    manifest=json.loads((BASE/'input_manifest.json').read_text(encoding='utf8'))
    terrain={}
    for sample in manifest['snapshots']:
        key=sample['key'];mapped=int(sample['mapped_stamp_ns'])
        layers=terrain_layers(arrays[key+'_level'],
            sensor_origin_xy=arrays[key+'_sensor_level'][:2],
            obstacle_min_height=.10)
        message=GridMap();message.header.stamp.sec=mapped//10**9
        message.header.stamp.nanosec=mapped%10**9
        message.header.frame_id='spot1/terrain_base_link'
        message.info.resolution=.5;message.info.length_x=message.info.length_y=40.
        message.info.pose.orientation.w=1.
        message.layers=['traversability','unknown','elevation','elevation_valid','witnessed_step']
        message.basic_layers=['traversability','unknown']
        for name in message.layers:
            data=Float32MultiArray()
            data.layout.dim=[MultiArrayDimension(label='column_index',size=80,stride=6400),
                             MultiArrayDimension(label='row_index',size=80,stride=80)]
            data.data=grid_map_values(layers[name]);message.data.append(data)
        terrain[mapped]=message
    stamps=sorted(set(odoms)|set(terrain)|set(transforms))
    report['input_counts']=dict(odoms=len(odoms),terrain=len(terrain),tf_stamps=len(transforms))
    prefix=Path(get_package_prefix('graphnav_mapper'))
    assert prefix==BASE/'install/graphnav_mapper',str(prefix)
    exe=prefix/'lib/graphnav_mapper/mapper_node';assert exe.is_file()
    child=owned.spawn('mapper',[str(exe),'--ros-args','-r','__ns:=/spot1','-r','__node:=graphnav_mapper',
        '-p','use_occupancy_input:=false','-p','use_sim_time:=true',
        '-r','odometry:=/spot1/odom','-r','traversability:=/spot1/local_terrain','-r','pose_graph:=/spot1/pose_graph'])
    service=node.create_client(GetParameters,'/spot1/graphnav_mapper/get_parameters')
    assert service.wait_for_service(timeout_sec=10.),'Mapper parameter service unavailable'
    names=['robot_profile_id','robot_max_slope','robot_max_step_height','robot_max_roughness','robot_minimum_clearance','use_occupancy_input','use_sim_time']
    future=service.call_async(GetParameters.Request(names=names))
    wait_for(future.done,5.,'Parameter response');assert not future.exception()
    report['actual_parameters']=dict(zip(names,[parameter_value_to_python(v) for v in future.result().values]))
    assert all(report['actual_parameters'][name] is not None for name in names),'Actual Mapper lacks expected physical parameters'
    report['runtime_executable']=dict(path=str(exe),sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),identity=proc_identity(child.pid))
    libraries={}
    for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines():
        name=line.split()[-1]
        if name.startswith('/') and '.so' in name and 'graphnav' in name:
            p=Path(name);libraries[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    report['runtime_libraries']=libraries
    (OUT/'runtime_identity.json').write_text(json.dumps(dict(executable=report['runtime_executable'],libraries=libraries,parameters=report['actual_parameters']),indent=2)+'\n')
    wait_for(lambda: all(pub.get_subscription_count()>0 for pub in (clock_pub,tf_pub,odom_pub,terrain_pub,pose_pub)),10.,'Input subscriptions')
    assert node.count_publishers('/wildos/native_control_command')==0 and node.count_publishers('/wildos/cmd_vel_stamped')==0
    pose=PoseGraph();pose.header.frame_id='spot1/odom';pn=PoseGraphNode()
    pn.uuid.id=[i+1 for i in range(len(pn.uuid.id))];pn.pose.orientation.w=1.
    pose.nodes=[pn];pose.current_node=pn.uuid
    first=stamps[0];stamp=odoms[first].header.stamp if first in odoms else transforms[first][0].header.stamp
    clock_pub.publish(Clock(clock=stamp));spin(.1)
    pose.header.stamp=stamp
    for _ in range(3):pose_pub.publish(pose);spin(.1)
    probe=json.loads((source/'contact_result.json').read_text())
    body=[n for s in probe['contacts'] for n in (s['contacts']['nodes'] or []) if n['class']=='body']
    peak=max(body,key=lambda n:n['max_normal_force'])
    contact=[peak['peak_reference']['node_world'][a] for a in ('x','y')]
    previous=first
    for stamp_ns in stamps:
        spin(min(.12,max(.0,(stamp_ns-previous)*1e-9)));previous=stamp_ns
        sample=odoms.get(stamp_ns) or terrain.get(stamp_ns)
        stamp=sample.header.stamp if sample is not None else transforms[stamp_ns][0].header.stamp
        clock_pub.publish(Clock(clock=stamp));spin(.005)
        if stamp_ns in transforms:tf_pub.publish(TFMessage(transforms=transforms[stamp_ns]))
        if stamp_ns in odoms:odom_pub.publish(odoms[stamp_ns])
        if stamp_ns in terrain:
            spin(.025);terrain_pub.publish(terrain[stamp_ns]);published=time.monotonic()
            def processed():
                return (graph is not None and graph_at>published and graph.nodes
                    and any(observed_ns(p)>=stamp_ns for n in graph.nodes for p in n.robot_traversability))
            wait_for(processed,4.,'Mapper terrain update '+str(stamp_ns))
            seq=len(report['snapshots']);(OUT/('graph_%02d.cdr'%seq)).write_bytes(serialize_message(graph))
            near=[]
            for i,n in enumerate(graph.nodes):
                p=[n.pose.position.x,n.pose.position.y]
                if math.dist(p,contact)<=3.:
                    near.append(dict(index=i,position=p,distance=math.dist(p,contact),metadata=plain(n)))
            edges=[]
            for edge in graph.edges:
                a=graph.nodes[edge.from_idx].pose.position;b=graph.nodes[edge.to_idx].pose.position
                d=dist_segment(contact,(a.x,a.y),(b.x,b.y))
                if d<=1.05:edges.append(dict(distance=d,metadata=plain(edge)))
            report['snapshots'].append(dict(input_stamp_ns=stamp_ns,graph_stamp_ns=ns(graph),nodes=len(graph.nodes),edges=len(graph.edges),
                near_contact_nodes=near,near_contact_edges=edges,robot_profiles=plain(graph)['robot_profiles'],
                graph_file='graph_%02d.cdr'%seq))
            (OUT/'progress.json').write_text(json.dumps(dict(maps=len(report['snapshots']),nodes=len(graph.nodes),edges=len(graph.edges)))+'\n')
    report['graph_messages']=graph_count;report['completed']=True
except BaseException as exc:report['errors'].append(repr(exc))
finally:
    report['errors'].extend(owned.close())
    node.destroy_node();rclpy.shutdown()
    report['completed']=report['completed'] and not report['errors']
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in dict(output=str(OUT),**report).items() if k not in ('snapshots','runtime_executable')},indent=2))
raise SystemExit(0 if report['completed'] else 1)
