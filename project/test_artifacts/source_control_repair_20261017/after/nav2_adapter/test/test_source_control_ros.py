"""Real speed-loop/transport/bridge callbacks, with controlled clocks and I/O.

No vehicle dynamics or DDS scheduling is modeled by these tests.
"""
import copy
import json
import time
from contextlib import contextmanager
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
from std_msgs.msg import String
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
from wildos_nav2_adapter.source_control import (
    PROTOCOL, COMMAND_TOPIC, SourceControlReceiver, control_values)
import test_native_vehicle_control_ros as transport
import test_execution_mechanism_ros as bridge_fixture

BRAKE = dict(throttle=0., brake=1., steering=0., parkingbrake=0., clutch=0., gear=2)
DRIVE = dict(BRAKE, throttle=.2, brake=0.)


class SourceControlCallbackTests(unittest.TestCase):
    send = transport.NativeControlIoTests.send
    tick = transport.NativeControlIoTests.tick

    def setUp(self):
        transport.NativeControlIoTests.setUp(self)
        declare = CmdVelToControl.declare_parameter
        def defaults(node, name, value):
            value = {'control_protocol': PROTOCOL, 'control_topic': COMMAND_TOPIC}.get(name, value)
            return declare(node, name, value)
        with patch.object(CmdVelToControl, 'declare_parameter', defaults):
            self.speed = CmdVelToControl()
        self.addCleanup(self.speed.destroy_node)
        self.speed.pub = Mock(); self.speed.control_status_pub = Mock()
        self.speed._safety_clock = S(now=lambda: S(nanoseconds=int(time.monotonic()*1e9)))
        self.speed.get_clock = lambda: S(now=lambda: S(nanoseconds=time.time_ns()))
        self.node.command = None
        self.node.source_receiver = SourceControlReceiver()
        self.sender = self.speed.source_sender

    @contextmanager
    def at(self, now):
        with patch('time.monotonic', return_value=now), \
             patch('time.monotonic_ns', return_value=int(now*1e9)), \
             patch('time.time', return_value=(self.wall_ns+int((now-100)*1e9))*1e-9):
            yield

    def relay(self, frame=None):
        if frame is None:
            message = self.speed.pub.publish.call_args.args[0]
        else:
            message = String(data=json.dumps(frame))
        self.node.on_control(message)
        self.node.poll_tick(); self.node.control_tick()
        status = self.node.telemetry.publish.call_args.args[0]
        self.speed.on_native_motion(status)
        return json.loads(status.data)

    def arm(self):
        self.speed.enabled = True
        self.speed.last_state_time = self.speed.last_command_time = time.monotonic()
        self.speed.target_speed = 1.2
        with self.at(time.monotonic()-.05):
            self.speed.tick()  # Source health missing: brake barrier.
            self.assertEqual(json.loads(self.speed.pub.publish.call_args.args[0].data)['control']['brake'], 1.)
            self.relay()
        self.assertTrue(self.speed.native_control_ok)
        self.speed.tick()
        self.assertGreater(json.loads(self.speed.pub.publish.call_args.args[0].data)['control']['throttle'], 0.)
        return self.relay()

    def test_native_fresh_receipt_cannot_reset_four_hundred_ms_source_age(self):
        with self.at(100.):
            self.send(BRAKE)
            frame = self.sender.issue(DRIVE, time.time_ns(), time.monotonic_ns())
        with self.at(100.4):
            status = self.relay(frame)
            self.assertAlmostEqual(status['command_age'], .4)
            self.assertAlmostEqual(status['command_receipt_age'], 0.)
            self.assertTrue(status['source_receipt']['applied'])
        with self.at(100.501):
            self.node.poll_tick(); self.node.control_tick()
            status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
            self.assertEqual(status['sent']['throttle'], 0.)
            self.assertEqual(status['sent']['brake'], 1.)
            self.assertIsNone(status['source_receipt'])

    def test_newer_brake_cannot_be_overwritten_by_late_old_throttle(self):
        with self.at(100.):
            self.send(BRAKE); old = self.send(DRIVE)
        with self.at(100.1):
            self.send(BRAKE, 100.1)
            status = self.relay(old)
            self.assertEqual(status['source_rejection'], 'source_reordered')
            self.assertEqual(status['sent']['brake'], 1.)
            self.assertIsNotNone(status['source_receipt'])
            self.assertEqual(status['source_receipt']['control']['throttle'], 0.)

    def test_enable_hold_release_require_fresh_generation_ack_and_new_velocity(self):
        with self.at(100.):
            self.arm()
            old = self.node.telemetry.publish.call_args.args[0]
        with self.at(100.05):
            self.speed.on_parameters([S(name='enable', value=True)])
            self.assertFalse(self.speed.native_control_ok)
            self.assertEqual(self.speed.target_speed, 0.)
            self.speed.on_native_motion(old)
            self.assertFalse(self.speed.native_control_ok)
            self.relay()
            self.assertTrue(self.speed.native_control_ok)
        for revision, held, now in ((1, True, 100.1), (2, False, 100.15)):
            with self.at(now):
                generation = self.sender.generation
                self.speed.on_execution_hold(String(data=json.dumps(dict(
                    owner='bridge', revision=revision, held=held, wall_time=time.time()))))
                self.assertGreater(self.sender.generation, generation)
                self.assertFalse(self.speed.native_control_ok)
                self.assertEqual(self.speed.target_speed, 0.)
                self.relay()
                self.assertTrue(self.speed.native_control_ok)
                self.speed.tick()
                self.assertEqual(json.loads(self.speed.pub.publish.call_args.args[0].data)['control']['throttle'], 0.)

    def test_expired_receipt_cannot_arm_speed_loop_even_with_fresh_telemetry(self):
        with self.at(100.):
            status = self.arm()
        with self.at(100.6):
            status.update(wall_time=time.time(), telemetry_age_sec=0.)
            self.speed.on_native_motion(String(data=json.dumps(status)))
            self.assertFalse(self.speed.native_control_ok)
            self.speed.last_state_time = self.speed.last_command_time = 100.6
            self.speed.tick()
            self.assertEqual(json.loads(self.speed.pub.publish.call_args.args[0].data)['control']['brake'], 1.)
        with self.at(100.65):
            self.relay()  # New brake barrier, not cached driving demand.
            self.assertTrue(self.speed.native_control_ok)

    def test_unknown_or_changed_payload_receipts_cannot_arm(self):
        with self.at(100.):
            status = self.arm()
        for change in ({'sequence':999}, {'sender_id':'b'*32}, {'control':BRAKE}):
            with self.at(100.05):
                bad = copy.deepcopy(status)
                bad['wall_time'] = time.time()
                bad['source_receipt'].update(change)
                self.speed.native_source_wall_time = None
                self.speed.on_native_motion(String(data=json.dumps(bad)))
                self.assertFalse(self.speed.native_control_ok)

    def test_late_rpc_ack_does_not_rearm_or_claim_the_brake_fallback_as_source_motion(self):
        with self.at(100.):
            self.arm()
            frame = self.sender.issue(DRIVE, time.time_ns(), time.monotonic_ns())
        with self.at(100.1):
            self.node.on_control(String(data=json.dumps(frame)))
            self.node.poll_tick()
            clock = [100.1]
            self.node.actuator.control.side_effect = lambda **kw: clock.__setitem__(0, 100.6)
            with patch('time.monotonic', side_effect=lambda: clock[0]):
                self.node.control_tick()
            status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
            self.assertTrue(status['control_acknowledged'])
            self.assertFalse(status['source_receipt']['applied'])
        with self.at(100.6):
            self.speed.on_native_motion(String(data=json.dumps(status)))
            self.assertFalse(self.speed.native_control_ok)

    def test_bridge_requires_source_receipt_and_matching_owner_but_allows_one_cycle_topic_skew(self):
        holder = S()
        bridge_fixture.ExecutionMechanismTests.setUp(holder)
        bridge = holder.node
        self.addCleanup(bridge.destroy_node)
        bridge.require_source_control = True
        with self.at(100.):
            self.arm()
            bridge.get_clock = self.speed.get_clock
            bridge.on_native_status(self.node.telemetry.publish.call_args.args[0])
            bridge.on_control_status(self.speed.control_status_pub.publish.call_args.args[0])
            # The first status was emitted before its brake ACK; the next is ready.
            self.assertIsNone(bridge.execution_transport_issue())
            native = copy.deepcopy(bridge.native_status)
            bridge.native_status.pop('source_receipt')
            self.assertIsNotNone(bridge.execution_transport_issue())
            bridge.native_status = native
            bridge.control_status['source_control']['generation'] += 1
            self.assertEqual(bridge.execution_transport_issue(), 'source_owner_unconfirmed')

    def test_malformed_and_headless_messages_do_not_refresh_native_watchdog(self):
        with self.at(100.):
            self.arm(); old_time = self.node.command_at
        with self.at(100.4):
            for value in ('{}', 'not json', json.dumps(dict(control=DRIVE))):
                self.node.on_control(String(data=value))
                self.assertEqual(self.node.command_at, old_time)
        with self.at(100.6):
            self.node.poll_tick(); self.node.control_tick()
            self.assertEqual(json.loads(self.node.telemetry.publish.call_args.args[0].data)['sent']['brake'], 1.)

    def test_actuator_failure_never_produces_a_source_ack_or_resumes_motion(self):
        with self.at(100.):
            self.arm()
        with self.at(100.05):
            self.node.actuator.control.side_effect=TimeoutError('lost ACK')
            self.speed.tick()
            status=self.relay()
            self.assertFalse(status['actuator_ok'])
            self.assertFalse(status['source_receipt']['applied'])
            self.assertFalse(self.speed.native_control_ok)
        with self.at(100.1):
            self.speed.tick()
            self.assertEqual(json.loads(self.speed.pub.publish.call_args.args[0].data)['control']['brake'],1.)

    def test_wrong_topic_protocol_pair_is_rejected_before_creating_actuator_publisher(self):
        original=CmdVelToControl.declare_parameter
        created=[]
        def wrong(node,name,value):
            if not created:created.append(node)
            return original(node,name,COMMAND_TOPIC if name=='control_topic' else value)
        try:
            with patch.object(CmdVelToControl,'declare_parameter',wrong), \
                 patch.object(CmdVelToControl,'create_publisher') as publisher:
                with self.assertRaises(ValueError):CmdVelToControl()
                # Real rclpy creates /parameter_events inside Node.__init__.
                # The forbidden operation is creating a control publisher.
                self.assertFalse(any(len(c.args)>1 and c.args[1] in ('/control',COMMAND_TOPIC)
                                     for c in publisher.call_args_list))
        finally:
            if created:created[0].destroy_node()

    def test_future_native_publication_cannot_poison_recovery_ordering(self):
        with self.at(100.):
            status=self.arm()
            old_stamp=self.speed.native_source_wall_time
            status['wall_time']=time.time()+10.
            self.speed.on_native_motion(String(data=json.dumps(status)))
            self.assertEqual(self.speed.native_source_wall_time,old_stamp)
        with self.at(100.05):
            self.speed.tick();self.relay()
            self.assertTrue(self.speed.native_control_ok)
            self.assertGreater(self.speed.native_source_wall_time,old_stamp)

    def test_missing_source_does_not_prevent_measured_stop_but_still_blocks_new_motion(self):
        holder=S()
        bridge_fixture.ExecutionMechanismTests.setUp(holder)
        bridge=holder.node
        self.addCleanup(bridge.destroy_node)
        bridge.require_source_control=True
        bridge.follow_quarantine={1:dict(terminal=True)}
        bridge.start_segment=Mock()
        with self.at(100.):self.arm()
        for now in (100.6,101.2):
            with self.at(now):
                holder.now=int(now*1e9)
                for value,receive in ((bridge.odom,bridge.on_odom),(bridge.grid,bridge.on_map)):
                    value=copy.deepcopy(value)
                    value.header.stamp=bridge.get_clock().now().to_msg()
                    receive(value)
                self.node.poll_tick();self.node.control_tick()
                bridge.on_native_status(self.node.telemetry.publish.call_args.args[0])
                self.assertIsNone(bridge.execution_transport_issue(for_stop=True))
                self.assertIsNotNone(bridge.execution_transport_issue())
                bridge.monitor_follow_transactions()
        self.assertFalse(bridge.follow_quarantine)
        bridge.start_segment.assert_called_once()

    def test_stop_confirmation_keeps_telemetry_and_rpc_ack_guards(self):
        holder=S()
        bridge_fixture.ExecutionMechanismTests.setUp(holder)
        bridge=holder.node
        self.addCleanup(bridge.destroy_node)
        bridge.require_source_control=True
        with self.at(100.):
            self.arm()
            bridge.on_native_status(self.node.telemetry.publish.call_args.args[0])
            bridge.native_status['control_acknowledged']=False
            self.assertEqual(bridge.execution_transport_issue(for_stop=True),'native_control_unacknowledged')
            bridge.native_status['control_acknowledged']=True
            bridge.native_status['telemetry_ok']=False
            self.assertEqual(bridge.execution_transport_issue(for_stop=True),'native_telemetry_stale')


if __name__ == '__main__':
    unittest.main()
