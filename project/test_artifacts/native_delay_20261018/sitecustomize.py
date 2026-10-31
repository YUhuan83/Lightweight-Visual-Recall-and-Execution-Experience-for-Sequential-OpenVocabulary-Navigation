"""Opt-in diagnostic executor only for the Native entry point; no control policy change."""
import os
from pathlib import Path
import sys

if Path(sys.argv[0]).name=='native_vehicle_control' and os.environ.get('WILDOS_NATIVE_TIMING_FILE'):
    import atexit
    import json
    import time
    import rclpy.executors as executors
    OriginalExecutor=executors.MultiThreadedExecutor
    output=open(os.environ['WILDOS_NATIVE_TIMING_FILE'],'x',buffering=1)
    atexit.register(output.close)
    if os.environ.get('WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT')=='1':
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        original_subscribe=Node.create_subscription
        def subscribe(self,msg_type,topic,callback,qos_profile,**kwargs):
            if topic=='/wildos/native_control_command':
                qos_profile=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
                output.write(json.dumps(dict(kind='diagnostic_qos',depth=1,reliability='BEST_EFFORT'))+'\n')
            return original_subscribe(self,msg_type,topic,callback,qos_profile,**kwargs)
        Node.create_subscription=subscribe

    class TimingExecutor(OriginalExecutor):
        def _make_handler(self,entity,node,take_from_wait_list,call_coroutine):
            if getattr(entity,'topic_name','')=='/wildos/native_control_command':
                self.native_ready_wall_ns=time.time_ns()
                self.native_ready_mono_ns=time.monotonic_ns()
            return super()._make_handler(entity,node,take_from_wait_list,call_coroutine)

        def _take_subscription(self,sub):
            begin_wall=time.time_ns();begin_mono=time.monotonic_ns()
            with sub.handle:
                entered_mono=time.monotonic_ns()
                result=sub.handle.take_message(sub.msg_type,sub.raw)
            if result is None:return None
            before=dict(take_begin_wall_ns=begin_wall,take_begin_mono_ns=begin_mono,
                        handle_entered_mono_ns=entered_mono,
                        ready_wall_ns=getattr(self,'native_ready_wall_ns',None),
                        ready_mono_ns=getattr(self,'native_ready_mono_ns',None))
            return result[0],result[1],time.time_ns(),time.monotonic_ns(),before

        async def _execute_timer(self,tmr,ready):
            began=time.monotonic_ns()
            try:return await super()._execute_timer(tmr,ready)
            finally:
                output.write(json.dumps(dict(kind='timer',callback=getattr(tmr.callback,'__name__','unknown'),begin_mono_ns=began,end_mono_ns=time.monotonic_ns()))+'\n')

        async def _execute_subscription(self,sub,packet):
            if packet is None:return
            msg,info,take_wall,take_mono,before=packet
            callback_wall=time.time_ns();callback_mono=time.monotonic_ns()
            try:
                await executors.await_or_execute(sub.callback,msg)
            finally:
                if sub.topic_name=='/wildos/native_control_command':
                    try:
                        frame=json.loads(msg.data)
                        row=dict(issued_wall_ns=frame.get('issued_wall_ns'),sequence=frame.get('sequence'),
                            sender_id=frame.get('sender_id'),generation=frame.get('generation'),
                            metadata=info,take_wall_ns=take_wall,take_mono_ns=take_mono,
                            callback_wall_ns=callback_wall,callback_mono_ns=callback_mono,
                            end_wall_ns=time.time_ns(),end_mono_ns=time.monotonic_ns(),**before)
                        output.write(json.dumps(row)+'\n')
                    except Exception as exc:
                        output.write(json.dumps(dict(diagnostic_error=repr(exc)))+'\n')
    executors.MultiThreadedExecutor=TimingExecutor
