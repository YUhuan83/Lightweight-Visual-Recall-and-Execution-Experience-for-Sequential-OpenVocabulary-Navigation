import pathlib,json,time,subprocess,os,signal,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from grid_map_msgs.msg import GridMap
out=pathlib.Path('/home/ly/town_memory_validation_20261010');root=pathlib.Path('/home/ly/town_memory_validation_20261009/physical_profile_fix/candidate');deadline=time.monotonic()+100
while not (out/'tf_ab_witness.json').exists():
 if time.monotonic()>deadline:raise RuntimeError('First witness did not finish')
 time.sleep(.2)
pid=int((out/'mapper_be.pid').read_text());cmd=pathlib.Path('/proc')/str(pid)/'cmdline'
if cmd.exists():
 assert str(out/'tf_be_probe/install/graphnav_mapper/lib/graphnav_mapper/mapper_node').encode() in cmd.read_bytes();os.kill(pid,signal.SIGINT)
rclpy.init();n=Node('matched_mapper_tf_qos_comparison');rows=[];processes=[];logs=[]
def tfcb(kind):
 def cb(m):
  for t in m.transforms:
   if t.child_frame_id=='spot1/terrain_base_link':rows.append(dict(kind=kind,stamp_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec,receipt_wall_ns=time.time_ns()))
 return cb
for kind,rel in [('tf_RE',ReliabilityPolicy.RELIABLE),('tf_BE',ReliabilityPolicy.BEST_EFFORT)]:n.create_subscription(TFMessage,'/tf',tfcb(kind),QoSProfile(depth=100,reliability=rel))
for kind in ['RE','BE']:
 n.create_subscription(GridMap,'/tf_twin_'+kind.lower()+'/dbg_map',lambda m,k=kind:rows.append(dict(kind=k+'_accepted' if m.layers else k+'_empty',stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,receipt_wall_ns=time.time_ns())),1)
n.create_subscription(GridMap,'/spot1/local_terrain',lambda m:rows.append(dict(kind='terrain',stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,receipt_wall_ns=time.time_ns())),1)
try:
 for kind,prefix in [('RE',root/'install/graphnav_mapper'),('BE',out/'tf_be_probe/install/graphnav_mapper')]:
  env=dict(os.environ,LD_LIBRARY_PATH=str(prefix/'lib')+':'+os.environ.get('LD_LIBRARY_PATH',''));f=(out/('tf_twin_'+kind+'.log')).open('w');logs.append(f);args=[str(prefix/'lib/graphnav_mapper/mapper_node'),'--ros-args','-r','__ns:=/tf_twin_'+kind.lower(),'-r','__node:=mapper_'+kind.lower(),'-p','use_occupancy_input:=false','-r','odometry:=/spot1/odom','-r','traversability:=/spot1/local_terrain','-r','pose_graph:=/spot1/pose_graph'];processes.append(subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
  (out/('tf_twin_'+kind+'.pid')).write_text(str(processes[-1].pid))
 warm=time.monotonic()
 while time.monotonic()-warm<5:rclpy.spin_once(n,timeout_sec=.01)
 start=time.time_ns();t=time.monotonic()
 while time.monotonic()-t<60:rclpy.spin_once(n,timeout_sec=.01)
 end=time.time_ns();records=[r for r in rows if start<=r['receipt_wall_ns']<=end];summary={k:sum(r['kind']==k for r in records) for k in ['RE_accepted','BE_accepted','RE_empty','BE_empty','terrain','tf_RE','tf_BE']};report=dict(scope='Two freshly started equivalent mappers, same live stationary source and exact 60s window; only dynamic TF reliability differs, separate outputs, no vehicle control',start_wall_ns=start,end_wall_ns=end,counts=summary,records=records);(out/'tf_twin_compare.json').write_text(json.dumps(report,indent=2));print('RESULT',summary,flush=True)
finally:
 for p in processes:
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for p in processes:
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
 for f in logs:f.close()
 n.destroy_node();rclpy.shutdown()
