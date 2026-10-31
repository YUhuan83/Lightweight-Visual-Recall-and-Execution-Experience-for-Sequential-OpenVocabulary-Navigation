"""End-to-end controller source lifetime through real speed-loop methods.

Controlled clocks and I/O; real ROS messages on the server. No dynamics/DDS.
"""
import json,math,unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from geometry_msgs.msg import Twist,TwistStamped
from beamng_msgs.msg import VehicleControl
from std_msgs.msg import String
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
from wildos_nav2_adapter.source_control import PROTOCOL,COMMAND_TOPIC,SourceControlReceiver

W=1_700_000_000_000_000_000
M=100_000_000_000

class ControllerSourceTests(unittest.TestCase):
    def setUp(self):
        declare=CmdVelToControl.declare_parameter
        def defaults(node,name,value):
            # Real rclpy nodes declare use_sim_time implicitly.
            if name=='require_stamped_command' and hasattr(node,'parameters'):
                node.parameters.setdefault('use_sim_time',S(value=False))
            return declare(node,name,{'require_stamped_command':True,
                'cmd_vel_topic':'/wildos/cmd_vel_stamped',
                'control_protocol':PROTOCOL,'control_topic':COMMAND_TOPIC}.get(name,value))
        with patch.object(CmdVelToControl,'declare_parameter',defaults):self.node=CmdVelToControl()
        self.addCleanup(self.node.destroy_node)
        self.wall=W;self.mono=M;self.ros=W
        for name,value in (('time.time_ns',lambda:self.wall),('time.time',lambda:self.wall/1e9),
                           ('time.monotonic_ns',lambda:self.mono),('time.monotonic',lambda:self.mono/1e9)):
            p=patch(name,side_effect=value);p.start();self.addCleanup(p.stop)
        n=self.node
        n._safety_clock=S(now=lambda:S(nanoseconds=self.mono))
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=self.ros))
        n.pub=Mock();n.control_status_pub=Mock();n.enabled=True

    def at(self,delta):
        self.wall=W+round(delta*1e9);self.ros=self.wall;self.mono=M+round(delta*1e9)

    def command(self,source=0.,speed=1.2,frame='spot1/nav_base_link'):
        msg=TwistStamped();stamp=W+round(source*1e9)
        msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
        msg.header.frame_id=frame;msg.twist.linear.x=speed
        self.node.on_stamped_cmd_vel(msg)
        return msg

    def publish(self,throttle=.2,reason='tracking'):
        control=VehicleControl();control.gear=2;control.throttle=throttle
        control.brake=0. if throttle else 1.
        self.node.publish_control(control,reason)
        return json.loads(self.node.pub.publish.call_args.args[0].data)

    def test_delayed_source_is_charged_before_callback(self):
        self.at(.4);self.command()
        self.assertEqual(self.node.target_speed,1.2)
        self.assertAlmostEqual(self.node.last_command_time,100.4)
        self.assertEqual(self.node.controller_deadline_steady_ns,M+500_000_000)
        self.at(.501);self.node.tick()
        status=json.loads(self.node.control_status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['reason'],'stale_command')
        self.assertEqual(status['brake'],1.)

    def test_duplicate_does_not_refresh_or_change_demand(self):
        self.command();deadline=self.node.controller_deadline_steady_ns
        self.at(.4);self.command(speed=2.)
        self.assertEqual(self.node.controller_deadline_steady_ns,deadline)
        self.assertEqual(self.node.target_speed,1.2)
        self.assertAlmostEqual(self.node.last_command_time,100.)

    def test_reordered_expired_future_and_wrong_frame_do_not_replace_valid_input(self):
        self.at(.1);self.command(.1)
        deadline=self.node.controller_deadline_steady_ns
        for source,frame in ((.05,'spot1/nav_base_link'),(.7,'spot1/nav_base_link'),
                             (-1.,'spot1/nav_base_link'),(.15,'spot1/odom')):
            self.at(.2);self.command(source,2.,frame)
            self.assertEqual(self.node.controller_deadline_steady_ns,deadline)
            self.assertEqual(self.node.target_speed,1.2)

    def test_nonfinite_new_command_revokes_cached_demand(self):
        self.command();self.at(.1);self.command(.1,math.nan)
        self.assertEqual(self.node.target_speed,0.)
        self.assertEqual(self.node.last_command_time,0.)

    def test_unstamped_twist_cannot_drive_physical_stamped_profile(self):
        msg=Twist();msg.linear.x=1.2;self.node.on_cmd_vel(msg)
        self.assertEqual(self.node.target_speed,0.)
        self.assertEqual(self.node.last_command_time,0.)

    def test_terminal_zero_outside_rpp_immediately_brakes_and_fences_late_torque(self):
        self.command();self.at(.05)
        self.node.on_controller_stop(Twist())
        frame=json.loads(self.node.pub.publish.call_args.args[0].data)
        self.assertEqual(frame['control']['brake'],1.)
        self.assertEqual(frame['control']['throttle'],0.)
        self.at(.1);self.command(.04)
        self.assertEqual(self.node.target_speed,0.)
        self.command(.1)
        self.assertEqual(self.node.target_speed,1.2)

    def test_nonzero_legacy_safety_topic_cannot_clear_current_stamped_demand(self):
        self.command();self.at(.1)
        msg=Twist();msg.linear.x=2.
        self.node.on_controller_stop(msg)
        self.assertEqual(self.node.target_speed,1.2)
        self.assertFalse(self.node.controller_stop_active)

    def test_hold_release_fences_late_pre_release_command(self):
        self.command()
        for revision,held,delta in ((0,True,.05),(1,False,.1)):
            self.at(delta)
            self.node.on_execution_hold(String(data=json.dumps(dict(
                owner='bridge',revision=revision,held=held,wall_time=self.wall/1e9))))
        self.at(.11);self.command(.075)
        self.assertEqual(self.node.target_speed,0.)
        self.command(.11)
        self.assertEqual(self.node.target_speed,1.2)

    def test_reenable_requires_source_generated_after_enable(self):
        self.command();self.at(.1)
        self.node.on_parameters([S(name='enable',value=False)])
        self.at(.2);self.node.on_parameters([S(name='enable',value=True)])
        self.at(.21);self.command(.15)
        self.assertEqual(self.node.target_speed,0.)
        self.command(.21)
        self.assertEqual(self.node.target_speed,1.2)

    def test_native_receipt_keeps_original_controller_expiry(self):
        receiver=SourceControlReceiver()
        brake=self.publish(0.,'disabled')
        self.assertIsNone(receiver.accept(json.dumps(brake),self.wall,self.mono,stopped=True))
        self.at(.4);self.command();frame=self.publish()
        self.assertEqual(frame['expires_wall_ns'],W+500_000_000)
        self.assertIsNone(receiver.accept(json.dumps(frame),self.wall,self.mono,stopped=True))
        self.at(.49);self.assertIsNotNone(receiver.current(self.wall,self.mono))
        self.at(.501);self.assertIsNone(receiver.current(self.wall,self.mono))

    def test_expiry_between_tick_and_publish_forces_independent_braking(self):
        self.command();self.at(.501)
        frame=self.publish()
        self.assertEqual(frame['control']['throttle'],0.)
        self.assertEqual(frame['control']['brake'],1.)
        self.assertEqual(frame['expires_wall_ns'],self.wall+500_000_000)

    def test_wall_rollback_cannot_extend_accepted_controller_demand(self):
        self.command();self.at(.2);self.wall=W-100_000_000;self.ros=self.wall
        frame=self.publish()
        self.assertEqual(frame['control']['throttle'],0.)
        self.assertEqual(frame['control']['brake'],1.)

    def test_source_receipt_deadline_can_be_shortened_but_never_extended(self):
        sender=self.node.source_sender
        frame=sender.issue(dict(throttle=0.,brake=1.,steering=0.,parkingbrake=0.,clutch=0.,gear=2),
            self.wall,self.mono,expires_wall_ns=self.wall+100_000_000)
        self.assertEqual(frame['expires_wall_ns'],self.wall+100_000_000)
        with self.assertRaises(ValueError):sender.issue(frame['control'],self.wall,self.mono,
            expires_wall_ns=self.wall+600_000_000)
        with self.assertRaises(ValueError):sender.issue(frame['control'],self.wall,self.mono,
            expires_wall_ns=self.wall)

if __name__=='__main__':unittest.main()
