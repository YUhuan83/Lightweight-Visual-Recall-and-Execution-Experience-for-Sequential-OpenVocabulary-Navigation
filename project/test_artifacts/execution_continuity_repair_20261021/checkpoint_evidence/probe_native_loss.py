"""Real Native Node/timers and DDS, mocked BeamNG sockets; brake-only isolated domain."""
import argparse
import json
from pathlib import Path
import statistics
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as S
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor, await_or_execute
from rclpy.node import Node
from std_msgs.msg import String
from wildos_nav2_adapter import native_vehicle_control as native
from wildos_nav2_adapter.source_control import SourceControlSender, COMMAND_TOPIC
import os
if os.environ.get('DIAGNOSTIC_BEST_EFFORT')=='1':
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    original_subscribe=Node.create_subscription
    def subscribe(self,msg_type,topic,callback,qos_profile,**kwargs):
        if topic==COMMAND_TOPIC:qos_profile=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        return original_subscribe(self,msg_type,topic,callback,qos_profile,**kwargs)
    Node.create_subscription=subscribe


def stats(a):
    a=sorted(a)
    return dict(n=len(a), median=statistics.median(a), p95=a[min(len(a)-1,int(.95*len(a)))], max=a[-1]) if a else None


class TraceExecutor(MultiThreadedExecutor):
    def __init__(self, rows):
        super().__init__(num_threads=3)
        self.rows=rows

    def _take_subscription(self, sub):
        with sub.handle:
            result=sub.handle.take_message(sub.msg_type,sub.raw)
        if result is None:return None
        msg,info=result
        return msg,info,time.time_ns(),time.monotonic_ns()

    async def _execute_subscription(self, sub, packet):
        if packet is None:return
        msg,info,take_wall,take_mono=packet
        before=time.time_ns()
        frame=json.loads(msg.data)
        row=dict(sequence=frame['sequence'],issued_wall_ns=frame['issued_wall_ns'],
                 take_wall_ns=take_wall,take_mono_ns=take_mono,
                 callback_wall_ns=before,metadata=info)
        await await_or_execute(sub.callback,msg)
        row['callback_end_wall_ns']=time.time_ns()
        self.rows.append(row)


class Sensors:
    def __init__(self,delay):self.delay=delay
    def attach(self,*a,**k):pass
    def poll(self):time.sleep(self.delay)
    def __getitem__(self,key):return {'wheelspeed':0.}


class FakeVehicle:
    def __init__(self,*a,**k):
        self.vid='ego';self.model='etk';self.extensions=[];self.connection=None
        self.sensors=Sensors(args.poll_ms/1000.)
        self.state=dict(pos=[0.,0.,0.],vel=[0.,0.,0.],dir=[0.,1.,0.],up=[0.,0.,1.],time=0.)
        self.ai=S(set_mode=lambda *_:None)
    def connect(self,*a,**k):pass
    def disconnect(self):pass
    def set_shift_mode(self,*a):pass
    def control(self,**k):
        assert k.get('throttle',0.)==0.,'Brake-only probe'
        time.sleep(args.control_ms/1000.)


class FakeBeamNG:
    def __init__(self,*a,**k):
        self.vehicles=S(get_current=lambda:{'ego':FakeVehicle()},start_connection=lambda *a:dict(vid='ego',result=25253))
    def open(self,*a,**k):pass
    def disconnect(self):pass


def run():
    import os
    assert os.environ.get('ROS_DOMAIN_ID')=='183'
    assert os.environ.get('ROS_LOCALHOST_ONLY') in ('0','1')
    assert os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp'
    assert not args.output.exists()
    ros_args=['--ros-args','-p','vehicle_port:='+str(args.vehicle_port)] if args.real else None
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    if args.publisher:
        sender_node=Node('isolated_native_external_brake_source')
        pub=sender_node.create_publisher(String,COMMAND_TOPIC,10)
        sender=SourceControlSender()
        stopped=threading.Event()
        signal.signal(signal.SIGTERM,lambda *_:stopped.set())
        deadline=time.monotonic()+args.seconds
        target=time.monotonic()
        while not stopped.is_set() and time.monotonic()<deadline:
            frame=sender.issue(dict(throttle=0.,brake=1.,parkingbrake=0.,steering=0.,clutch=0.,gear=2),time.time_ns(),time.monotonic_ns())
            pub.publish(String(data=json.dumps(frame)))
            target+=.05
            stopped.wait(max(0.,target-time.monotonic()))
        sender_node.destroy_node();rclpy.shutdown();return
    patches=[patch.object(native,'BeamNGpy',FakeBeamNG),patch.object(native,'Vehicle',FakeVehicle),
             patch.object(native,'Connection',lambda *a,**k:S()),
             patch.object(native,'VehicleControlLease',lambda *a,**k:S(owner='fake',release=lambda *_:None)),
             patch.object(native,'install_direct_steering_control',lambda *_:None)]
    if args.real:patches=[]
    for p in patches:p.start()
    rows=[];status=[];cleanup_errors=[]
    node=native.NativeVehicleControl()
    native_qos=[dict(topic=sub.topic_name,depth=sub.qos_profile.depth,reliability=int(sub.qos_profile.reliability)) for sub in node.subscriptions if sub.topic_name==COMMAND_TOPIC]
    import hashlib
    native_identity=dict(path=native.__file__,sha256=hashlib.sha256(Path(native.__file__).read_bytes()).hexdigest())
    # Intercept status locally; real source subscription, StateSensor publisher,
    # timers, callback groups, receiver and control/poll methods stay unchanged.
    original_publish=node.telemetry.publish
    def record(msg):status.append(json.loads(msg.data));original_publish(msg)
    node.telemetry.publish=record
    executor=TraceExecutor(rows);executor.add_node(node)
    sender_node=Node('isolated_native_delay_brake_source')
    pub=sender_node.create_publisher(String,COMMAND_TOPIC,10)
    sender=SourceControlSender();halt=threading.Event()
    def publish():
        deadline=time.monotonic()+2
        while pub.get_subscription_count()<1 and time.monotonic()<deadline:time.sleep(.01)
        target=time.monotonic()
        while not halt.is_set():
            frame=sender.issue(dict(throttle=0.,brake=1.,parkingbrake=0.,steering=0.,clutch=0.,gear=2),time.time_ns(),time.monotonic_ns())
            pub.publish(String(data=json.dumps(frame)))
            target+=.05
            halt.wait(max(0.,target-time.monotonic()))
    thread=None;external=None
    if args.external:
        external=subprocess.Popen([sys.executable,__file__,'--publisher','--seconds',str(args.seconds+1),'--output',str(args.output)])
    else:
        thread=threading.Thread(target=publish);thread.start()
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.03)
    finally:
        halt.set()
        if thread:thread.join()
        if external and external.poll() is None:
            external.terminate();external.wait(timeout=2)
        executor.shutdown()
        try:node.stop()
        except Exception as exc:cleanup_errors.append(repr(exc))
        node.destroy_node();sender_node.destroy_node();rclpy.shutdown()
        for p in reversed(patches):p.stop()
    metrics={
        'issued_to_callback_ms':stats([(r['callback_wall_ns']-r['issued_wall_ns'])/1e6 for r in rows]),
        'dds_receive_to_take_ms':stats([(r['take_wall_ns']-r['metadata']['received_timestamp'])/1e6 for r in rows if r['metadata'].get('received_timestamp',0)>0]),
        'dds_source_to_receive_ms':stats([(r['metadata']['received_timestamp']-r['metadata']['source_timestamp'])/1e6 for r in rows if r['metadata'].get('received_timestamp',0)>0]),
        'callback_duration_ms':stats([(r['callback_end_wall_ns']-r['callback_wall_ns'])/1e6 for r in rows]),
        'tick_period_ms':stats([s['period']*1000 for s in status]),
        'watchdog':sum(s['watchdog_stop'] for s in status),
        'received':len(rows),'ticks':len(status)}
    args.output.write_text(json.dumps(dict(native_qos=native_qos,native_identity=native_identity,scope=__doc__,real=args.real,cleanup_errors=cleanup_errors,localhost_only=os.environ['ROS_LOCALHOST_ONLY'],poll_ms=args.poll_ms,control_ms=args.control_ms,metrics=metrics,rows=rows,status=status),indent=2)+'\n')
    print(json.dumps(metrics,indent=2))
    if cleanup_errors:raise RuntimeError(str(cleanup_errors))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=18)
    p.add_argument('--poll-ms',type=float,default=8);p.add_argument('--control-ms',type=float,default=16)
    p.add_argument('--publisher',action='store_true');p.add_argument('--external',action='store_true')
    p.add_argument('--real',action='store_true');p.add_argument('--vehicle-port',type=int,default=0)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args();run()
