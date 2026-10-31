"""Real DDS control/native feedback leases, isolated from vehicle execution."""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'nav2_adapter'))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import os
    if os.environ.get('ROS_DOMAIN_ID')!='85':
        raise RuntimeError('Use isolated domain 85; this probe must not publish into the vehicle domain')
    if args.output.exists():raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry,Path as Route
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from wildos_nav2_adapter.execution_feedback import ExecutionFeedback

    class Probe(Node,ExecutionFeedback):
        def __init__(self):
            Node.__init__(self,'isolated_execution_braking_contract')
            self.carrier_model='ackermann';self.follow_enabled=True;self.following=True;self.odom=Odometry()
            self.odom.pose.pose.orientation.w=1.;self.motion_queue=[]
            path=Route()
            for x in (0.,4.):
                p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
            self.motion_queue=[path];self.control_callbacks=self.native_callbacks=0
            self.init_execution_feedback()
            self.set_parameters([rclpy.Parameter('stall_timeout_sec',value=.2)])
            self.control=self.create_publisher(String,'/wildos/control_status',10)
            self.native=self.create_publisher(String,'/wildos/native_control_status',10)
        def on_control_status(self,m):
            self.control_callbacks+=1;ExecutionFeedback.on_control_status(self,m)
        def on_native_status(self,m):
            self.native_callbacks+=1;ExecutionFeedback.on_native_status(self,m)
        def spin(self,seconds,done=lambda:False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                rclpy.spin_once(self,timeout_sec=.01)
                if done():return True
            return False
        def send_control(self,active,stamp=None):
            data={'stamp_ns':self.get_clock().now().nanoseconds if stamp is None else stamp,
                'actuating':active,'target_speed':1.2,'target_angular':0.}
            old=self.control_callbacks;self.control.publish(String(data=json.dumps(data)))
            assert self.spin(2,lambda:self.control_callbacks>old),'Control DDS delivery failed'
            return data
        def send_native(self,brake,wall_time=None):
            data={'wall_time':time.time() if wall_time is None else wall_time,'speed':0.,
                'telemetry_ok':True,'telemetry_age_sec':0.,'actuator_ok':True,
                'control_acknowledged':True,'watchdog_stop':False,
                'sent':{'throttle':0. if brake else .3,'brake':brake,'gear':2},
                'actual':{'gear':'D','wheelspeed':0.},'wheel_contact':None}
            old=self.native_callbacks;self.native.publish(String(data=json.dumps(data)))
            assert self.spin(2,lambda:self.native_callbacks>old),'Native DDS delivery failed'
            return data

    rclpy.init();node=Probe();report={'passed':False,'checks':[],
        'scope':'Actual DDS and production feedback callbacks; no Nav2 action, simulator connection or movement',
        'domain':85,'source':str(sys.modules[ExecutionFeedback.__module__].__file__)}
    try:
        assert node.spin(3,lambda:node.control.get_subscription_count()>0 and node.native.get_subscription_count()>0)
        node.send_control(True);assert node.execution_stall() is None
        node.send_control(False);node.spin(.3);node.send_control(True)
        assert node.execution_stall() is None
        report['checks'].append('Control wait between monitor ticks does not age actuation')
        node.send_native(1.);node.spin(.3);node.send_control(True)
        assert node.execution_stall() is None and node.stall_anchor is None
        report['checks'].append('Native full brake vetoes controller intent')
        current_native=node.send_native(0.);assert node.execution_stall() is None
        anchor=node.stall_anchor;current_control=node.send_control(True)
        node.send_control(False,stamp=current_control['stamp_ns']-10000000)
        assert node.control_status['actuating'] and node.stall_anchor==anchor
        report['checks'].append('Late control wait rejected')
        node.send_native(1.,wall_time=current_native['wall_time']-.01)
        assert node.native_status['sent']['brake']==0. and node.stall_anchor==anchor
        report['checks'].append('Late native brake rejected')
        node.send_control(True,stamp=current_control['stamp_ns'])
        assert node.stall_anchor==anchor
        report['checks'].append('Duplicate control does not restart the stall timer')
        node.spin(.25);node.send_control(True);stall=node.execution_stall()
        assert stall is not None and stall['stall_class']=='actuated_no_path_progress'
        report['checks'].append('Real continuous command without motion still detected')
        report['stall']=stall;report['passed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        report['control_callbacks']=node.control_callbacks;report['native_callbacks']=node.native_callbacks
        node.destroy_node();rclpy.shutdown();args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
