"""Run the real transport tick with mocked BeamNG I/O; no simulator connection."""
import ast
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import MagicMock, Mock, patch
from beamng_msgs.msg import StateSensor, VehicleControl
from std_msgs.msg import String
from wildos_nav2_adapter.drive_math import speed_exceeds_limit
from wildos_nav2_adapter.wheel_contact import WHEEL_SNAPSHOT_LUA, classify_wheel_contact
from wildos_nav2_adapter.direct_vehicle_control import (
    CONTROL_ACK, install_direct_steering_control)
from wildos_nav2_adapter.vehicle_control_lease import LEASE_VERSION


class NativeControlIoTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parents[1] / 'wildos_nav2_adapter/native_vehicle_control.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                    and n.name in ('poll_tick', 'control_tick', 'stop')]
        namespace = dict(json=json, math=math, time=time, StateSensor=StateSensor, String=String,
            speed_exceeds_limit=speed_exceeds_limit, WHEEL_SNAPSHOT_LUA=WHEEL_SNAPSHOT_LUA,
            classify_wheel_contact=classify_wheel_contact, LEASE_VERSION=LEASE_VERSION)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), 'exec'), namespace)
        self.node = namespace['NativeVehicleControl']()
        self.node.get_parameter = lambda name: S(value=dict(command_timeout=.8, state_timeout=.5, overspeed_limit=0.,
            max_throttle=1., max_steering=.6, vehicle='ego', wheel_probe_interval_sec=1.)[name])
        self.node.get_clock = lambda: S(now=lambda: S(to_msg=lambda: S(sec=100, nanosec=0)))
        # StateSensor and JSON health status are separate ROS topics. Keeping
        # separate publishers makes each interface assertion independent.
        self.node.pub = Mock()
        self.node.telemetry = Mock()
        self.node.vehicle = Mock()
        self.node.vehicle.state = dict(pos=[1., 2., 3.], vel=[0., 0., 0.],
            dir=[0., 1., 0.], up=[0., 0., 1.], time=100.)
        sensors = MagicMock()
        sensors.__getitem__.return_value = {'wheelspeed': .5}
        self.node.vehicle.sensors = sensors
        self.node.actuator = Mock()
        self.node.bng = Mock()
        self.node.control_lease = Mock()
        self.node.command = VehicleControl(throttle=.25, brake=0., gear=2)
        self.node.command_at = self.node.last_tick = 100.
        self.node.command_lock = threading.Lock()
        self.node.state_lock = threading.Lock()
        self.node.latest_state = None
        self.node.latest_electrics = {}
        self.node.last_poll_at = 0.
        self.node.poll_roundtrip_ms = None
        self.node.last_poll_error = None
        self.node.last_poll_warning_at = 0.
        self.node.last_control_error = None
        self.node.actuator_ok = True
        self.node.wheel_probe_at = 0.
        self.node.wheel_snapshot = self.node.stationary_since = None
        self.node.get_logger = Mock()

    def tick(self, now=100.):
        with patch('time.monotonic', return_value=now):
            self.node.poll_tick()
            self.node.control_tick()

    def test_shutdown_failed_brake_releases_own_lease_via_existing_telemetry(self):
        self.node.actuator.control.side_effect = TimeoutError('late brake')
        with self.assertRaises(TimeoutError): self.node.stop()
        self.node.control_lease.release.assert_called_once_with(self.node.vehicle)
        for client in (self.node.actuator, self.node.vehicle, self.node.bng):
            client.disconnect.assert_called_once()

    def test_shutdown_unhealthy_actuator_has_no_reconnect_or_motion(self):
        self.node.actuator_ok = False
        self.node.stop()
        self.node.actuator.control.assert_not_called()
        self.node.actuator.connect.assert_not_called()
        self.node.control_lease.release.assert_called_once_with(self.node.vehicle)

    def test_shutdown_lease_failure_still_disconnects_all_streams(self):
        self.node.control_lease.release.side_effect = ConnectionError('no lease ACK')
        with self.assertRaises(ConnectionError): self.node.stop()
        for client in (self.node.actuator, self.node.vehicle, self.node.bng):
            client.disconnect.assert_called_once()

    def test_transport_does_not_reapply_the_previous_twelve_percent_cap(self):
        self.tick()
        self.assertEqual(self.node.actuator.control.call_args.kwargs['throttle'], .25)
        self.node.vehicle.queue_lua_command.assert_not_called()

    def test_poll_publishes_complete_state_sensor_contract(self):
        self.tick()
        sample = self.node.pub.publish.call_args.args[0]
        self.assertEqual((sample.position.x, sample.position.y, sample.position.z), (1., 2., 3.))
        self.assertEqual((sample.velocity.x, sample.velocity.y, sample.velocity.z), (0., 0., 0.))
        self.assertEqual((sample.dir.x, sample.dir.y, sample.dir.z), (0., 1., 0.))
        self.assertEqual((sample.up.x, sample.up.y, sample.up.z), (0., 0., 1.))

    def test_missing_state_brakes_on_actuator_socket(self):
        with patch('time.monotonic', return_value=100.):
            self.node.control_tick()
        self.assertEqual(self.node.actuator.control.call_args.kwargs['throttle'], 0.)
        self.assertEqual(self.node.actuator.control.call_args.kwargs['brake'], 1.)

    def test_healthy_driving_does_not_poll_wheel_contacts(self):
        self.node.vehicle.state['vel'] = [1.2, 0., 0.]
        self.node.stationary_since = 90.
        self.tick()
        self.node.vehicle.queue_lua_command.assert_not_called()

    def test_sustained_standstill_collects_contact_without_changing_controls(self):
        self.node.stationary_since = 98.
        self.node.vehicle.queue_lua_command.return_value = json.dumps([dict(name='FR',
            wheelSpeed=2., downForceRaw=0., contactMaterialID1=-1, contactMaterialID2=-1)])
        self.tick()
        self.assertEqual(self.node.wheel_snapshot['diagnosis']['classification'], 'wheel_airborne_spin')
        self.assertEqual(self.node.actuator.control.call_args.kwargs['throttle'], .25)

    def test_contact_probe_failure_is_diagnostic_and_stale_commands_still_brake(self):
        self.node.stationary_since = 98.
        self.node.vehicle.queue_lua_command.side_effect = RuntimeError('unsupported')
        self.tick()
        self.assertIsNone(self.node.wheel_snapshot)
        self.tick(102.)
        self.assertEqual(self.node.actuator.control.call_args.kwargs['throttle'], 0.)
        self.assertEqual(self.node.actuator.control.call_args.kwargs['brake'], 1.)

    def test_poll_failure_does_not_publish_fake_state_and_control_still_brakes(self):
        self.node.latest_state = dict(pos=[1., 2., 3.], vel=[0., 0., 0.],
            dir=[0., 1., 0.], up=[0., 0., 1.], time=100.)
        self.node.latest_electrics = {'wheelspeed': .5}
        self.node.last_poll_at = 99.
        self.node.vehicle.sensors.poll.side_effect = TimeoutError('sensor RPC timed out')
        with patch('time.monotonic', return_value=100.):
            self.node.poll_tick()
            self.node.control_tick()
        self.assertEqual(self.node.pub.publish.call_count, 0)
        self.assertEqual(self.node.actuator.control.call_args.kwargs['throttle'], 0.)
        self.assertEqual(self.node.actuator.control.call_args.kwargs['brake'], 1.)

    def test_actuator_failure_is_latched_and_reported_as_unhealthy(self):
        self.node.actuator.control.side_effect = TimeoutError('ack timed out')
        with patch('time.monotonic', return_value=100.):
            self.node.control_tick()
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertFalse(status['actuator_ok'])
        self.assertFalse(status['control_acknowledged'])
        self.assertIsNotNone(status['control_rpc_ms'])
        self.assertEqual(status['actuator_error'], 'TimeoutError: ack timed out')
        self.node.actuator.control.reset_mock()
        with patch('time.monotonic', return_value=100.1):
            self.node.control_tick()
        self.node.actuator.control.assert_not_called()
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertFalse(status['actuator_ok'])
        self.assertTrue(status['watchdog_stop'])
        self.assertFalse(status['control_acknowledged'])
        self.assertIsNone(status['control_rpc_ms'])

    def test_direct_control_ack_mismatch_latches_failure_without_retrying_motion(self):
        self.node.actuator.queue_lua_command.return_value = 'wrong_ack'
        install_direct_steering_control(self.node.actuator)
        self.tick()
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertFalse(status['actuator_ok'])
        self.assertFalse(status['control_acknowledged'])
        self.node.actuator.disconnect.assert_called_once()
        self.node.actuator.queue_lua_command.reset_mock()
        self.tick(100.1)
        self.node.actuator.queue_lua_command.assert_not_called()

    def test_direct_control_preserves_watchdog_braking_and_gear(self):
        self.node.actuator.queue_lua_command.return_value = CONTROL_ACK
        install_direct_steering_control(self.node.actuator)
        self.tick(102.)  # The command is older than the existing .8 s watchdog.
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertTrue(status['control_acknowledged'])
        self.assertTrue(status['watchdog_stop'])
        self.assertEqual(status['sent']['gear'], 2)
        self.assertEqual(status['sent']['throttle'], 0.)
        self.assertEqual(status['sent']['brake'], 1.)
        chunk = self.node.actuator.queue_lua_command.call_args.args[0]
        self.assertIn("input.event('brake', 1.0, FILTER_PAD)", chunk)
        self.assertIn('drivetrain.shiftToGear(2)', chunk)

    def test_slow_ack_does_not_renew_the_cached_state_age(self):
        self.node.latest_state = dict(self.node.vehicle.state)
        self.node.last_poll_at = 99.9
        clock = [100.]
        def delayed_ack(**control):
            clock[0] = 100.7
        self.node.actuator.control.side_effect = delayed_ack
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            self.node.control_tick()
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertTrue(status['actuator_ok'])
        self.assertTrue(status['control_acknowledged'])
        self.assertAlmostEqual(status['control_rpc_ms'], 700.)
        self.assertFalse(status['telemetry_ok'])
        self.assertAlmostEqual(status['telemetry_age_sec'], .8)
        self.assertAlmostEqual(status['command_age'], .7)

    def test_slow_poll_is_not_published_with_a_new_state_timestamp(self):
        clock = [100.]
        self.node.vehicle.sensors.poll.side_effect = lambda: clock.__setitem__(0, 100.7)
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            self.node.poll_tick()
            self.node.control_tick()
        self.node.pub.publish.assert_not_called()
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertFalse(status['telemetry_ok'])
        self.assertEqual(self.node.actuator.control.call_args.kwargs['brake'], 1.)

    def test_new_fast_poll_recovers_from_aged_sample(self):
        clock = [100.]
        self.node.vehicle.sensors.poll.side_effect = lambda: clock.__setitem__(0, 100.7)
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            self.node.poll_tick()
            self.node.control_tick()
            self.node.vehicle.sensors.poll.side_effect = None
            self.node.command_at = clock[0]
            self.node.poll_tick()
            self.node.control_tick()
        self.assertEqual(self.node.pub.publish.call_count, 1)
        status = json.loads(self.node.telemetry.publish.call_args.args[0].data)
        self.assertTrue(status['telemetry_ok'])
        self.assertTrue(status['control_acknowledged'])
        self.assertEqual(self.node.actuator.control.call_args.kwargs['throttle'], .25)
