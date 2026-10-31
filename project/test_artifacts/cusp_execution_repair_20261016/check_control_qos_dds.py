"""Compare latest-value QoS with the same three-thread callback layout; no vehicle."""
import rclpy,time,threading,json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile,ReliabilityPolicy
from beamng_msgs.msg import VehicleControl
from pathlib import Path
class Receiver(Node):
 def __init__(self,name,best):
  super().__init__(name);self.receipts=[];self.command_at=time.monotonic();self.ages=[];self.sent=[]
  self.groups=[MutuallyExclusiveCallbackGroup() for _ in range(3)]
  qos=QoSProfile(depth=1)
  if best:qos.reliability=ReliabilityPolicy.BEST_EFFORT
  self.sub=self.create_subscription(VehicleControl,'/probe/native_control',self.on_control,qos,callback_group=self.groups[0])
  self.create_timer(.05,self.poll,callback_group=self.groups[1])
  self.create_timer(.05,self.control,callback_group=self.groups[2])
 def on_control(self,m,info):
  self.command_at=time.monotonic();self.receipts.append((time.time_ns()-info.source_timestamp)*1e-9)
 def poll(self):time.sleep(.006)
 def control(self):
  self.ages.append(time.monotonic()-self.command_at);time.sleep(.003)
rclpy.init();pubnode=Node('control_qos_source');pub=pubnode.create_publisher(VehicleControl,'/probe/native_control',10)
pubnode.create_timer(.05,lambda:pub.publish(VehicleControl(throttle=.1,gear=2)))
nodes=[Receiver('control_reliable',False),Receiver('control_best_effort',True)]
executors=[];threads=[]
for n in [pubnode,*nodes]:
 e=MultiThreadedExecutor(num_threads=3);e.add_node(n);executors.append(e)
 t=threading.Thread(target=e.spin);t.start();threads.append(t)
time.sleep(12)
for e in executors:e.shutdown()
for t in threads:t.join()
def stats(a):
 a=sorted(a);return dict(count=len(a),median=a[len(a)//2],p95=a[int((len(a)-1)*.95)],maximum=a[-1])
report={n.get_name():dict(source_latency_s=stats(n.receipts),sample_hold_age_s=stats(n.ages[40:])) for n in nodes}
Path('/home/ly/cusp_execution_repair_20261016/qos_probe.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
for n in [pubnode,*nodes]:n.destroy_node()
rclpy.shutdown()
