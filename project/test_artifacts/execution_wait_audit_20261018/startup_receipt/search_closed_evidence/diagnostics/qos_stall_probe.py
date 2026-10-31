"""Isolated DDS age comparison: brake data only, no vehicle or live topic."""
import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

TOPIC = '/wildos/diagnostic_source_age'

def summary(values):
    a = sorted(values)
    return dict(count=len(a), median=statistics.median(a), p95=a[min(len(a)-1,int(.95*len(a)))],max=a[-1]) if a else None

def run(args):
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from wildos_nav2_adapter.source_control import SourceControlSender, SourceControlReceiver
    rclpy.init()
    node = Node('source_age_' + args.role)
    sender = SourceControlSender()
    receiver = SourceControlReceiver()
    ages=[]; seq=[]; rejected=[]; ticks=[]; observer=[]
    reader_started=time.monotonic(); injected=False
    profile=QoSProfile(depth=args.depth,reliability=(ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE))
    if args.role=='publisher':
        pub=node.create_publisher(String,TOPIC,10)
        def send():
            frame=sender.issue(dict(throttle=0.,brake=1.,steering=0.,parkingbrake=0.,clutch=0.,gear=2),time.time_ns(),time.monotonic_ns())
            pub.publish(String(data=json.dumps(frame)))
        node.create_timer(.05,send)
    else:
        def receive(msg):
            nonlocal injected
            wall=time.time_ns(); mono=time.monotonic_ns()
            frame=json.loads(msg.data)
            ages.append((wall-frame['issued_wall_ns'])/1e9); seq.append(frame['sequence'])
            error=receiver.accept(msg.data,wall,mono,stopped=True)
            if error: rejected.append(error)
            if args.stall_sec and not injected and time.monotonic()-reader_started>5:
                injected=True
                time.sleep(args.stall_sec)
        node.create_subscription(String,TOPIC,receive,profile,callback_group=MutuallyExclusiveCallbackGroup())
        # Independent reliable observer of the same source, as in the field.
        node.create_subscription(String,TOPIC,lambda m: observer.append((time.time_ns()-json.loads(m.data)['issued_wall_ns'])/1e9),10,callback_group=MutuallyExclusiveCallbackGroup())
        def tick():
            frame=receiver.current(time.time_ns(),time.monotonic_ns())
            ticks.append(frame is not None)
            time.sleep(.005)
        node.create_timer(.05,tick,callback_group=MutuallyExclusiveCallbackGroup())
        node.create_timer(.05,lambda:time.sleep(.003),callback_group=MutuallyExclusiveCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    deadline=time.monotonic()+args.seconds
    try:
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown()
    if args.role=='reader':
        report=dict(depth=args.depth,best_effort=args.best_effort,stall_sec=args.stall_sec,injected=injected,source_age=summary(ages),observer_age=summary(observer),sequences=seq,ages=ages,rejections=rejected,valid_tick_count=sum(ticks),ticks=len(ticks))
        Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('ages','sequences')},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('role',choices=['reader','publisher']);p.add_argument('--depth',type=int,default=1);p.add_argument('--best-effort',action='store_true');p.add_argument('--seconds',type=float,default=16);p.add_argument('--stall-sec',type=float,default=0);p.add_argument('--output',default='qos_result.json');run(p.parse_args())
