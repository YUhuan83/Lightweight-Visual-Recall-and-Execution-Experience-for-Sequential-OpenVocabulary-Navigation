"""Low latency vehicle controls on a socket separate from camera/LiDAR polling.

Use /wildos/native_control instead of /control, with exactly one actuator.
The simulation is already loaded; this node never loads or starts a scenario.
"""
import json
import math
import time
import threading

from beamngpy import BeamNGpy
from beamngpy.connection import Connection
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from beamngpy.sensors import Electrics
from beamngpy.vehicle import Vehicle
from beamng_msgs.msg import StateSensor, VehicleControl
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from .drive_math import speed_exceeds_limit
from .fail_closed_transport import install_fail_closed_reads
from .wheel_contact import WHEEL_SNAPSHOT_LUA, classify_wheel_contact
from .direct_vehicle_control import install_direct_steering_control
from .vehicle_control_lease import VehicleControlLease, LEASE_VERSION


class NativeVehicleControl(Node):
    def __init__(self):
        super().__init__('wildos_native_vehicle_control')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 25252)
        self.declare_parameter('vehicle', 'ego')
        self.declare_parameter('vehicle_port', 0)
        # BeamNG polling is synchronous and can take longer than one 20 Hz
        # tick. A small margin prevents false watchdog braking while retaining
        # a bounded stop when the command stream really disappears.
        self.declare_parameter('command_timeout', .8)
        self.declare_parameter('state_timeout', .5)
        self.declare_parameter('overspeed_limit', 0.0)
        # The speed-loop node owns throttle policy; this transport only clamps
        # to BeamNG's input range, unless an explicit transport limit is set.
        self.declare_parameter('max_throttle', 1.0)
        self.declare_parameter('max_steering', .6)
        self.declare_parameter('wheel_probe_interval_sec', 1.0)
        self.wheel_probe_at = 0.
        self.wheel_snapshot = None
        self.stationary_since = None
        for name in ('max_throttle', 'max_steering'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0 < value <= 1.:
                raise ValueError(name + ' must be in (0, 1]')
        interval = float(self.get_parameter('wheel_probe_interval_sec').value)
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError('wheel_probe_interval_sec must be finite and positive')
        install_fail_closed_reads(PrefixedLengthSocket)
        self.bng = BeamNGpy(self.get_parameter('host').value, self.get_parameter('port').value,
                           socket_timeout=2)
        self.bng.open(launch=False)
        self.vehicle = self.bng.vehicles.get_current()[self.get_parameter('vehicle').value]
        vehicle_port = self.get_parameter('vehicle_port').value
        if vehicle_port:
            if not 1 <= vehicle_port <= 65535:
                raise ValueError('vehicle_port must be 0 or a valid TCP port')
            # Use the measured listener after a respawn. BeamNG.tech 0.39.4
            # sometimes omits the port from a repeated StartVehicleConnection.
            self.vehicle.connection = Connection(
                self.get_parameter('host').value, vehicle_port, timeout=2)
        self.vehicle.connect(self.bng)
        # Keep slow state/electrics polling on its own vehicle socket. A
        # blocked Sensors.poll must not prevent the command watchdog from
        # sending a brake command on the actuator socket.
        self.actuator = Vehicle(self.vehicle.vid, self.vehicle.model)
        try:
            # Request the current endpoint, then open a separate TCP stream.
            # BeamNG may reuse the listener port; stream separation does not
            # isolate either client from an engine/render stall.
            self.actuator.connection = Connection(
                self.get_parameter('host').value, timeout=2)
            response = self.bng.vehicles.start_connection(
                self.actuator, self.actuator.extensions)
            if response.get('vid') != self.actuator.vid:
                raise RuntimeError('BeamNG allocated actuator port for another vehicle')
            actuator_port = int(response['result'])
            if not 1 <= actuator_port <= 65535:
                raise RuntimeError('BeamNG returned an invalid actuator vehicle port')
            self.actuator.connection.port = actuator_port
            self.actuator.connect(self.bng, tries=1)
            # Claim before changing AI/shift mode: a second owner must not
            # mutate an already controlled vehicle before being rejected.
            self.control_lease = VehicleControlLease(self.bng,self.actuator,
                float(self.get_parameter('command_timeout').value))
            self.actuator.ai.set_mode('disabled')
            self.actuator.set_shift_mode('realistic_automatic')
            # SDK Control uses FILTER_PAD for steering, adding gamepad
            # smoothing/assistance after our Ackermann command calculation.
            # Only this actuator uses direct input; vehicle physics remain.
            install_direct_steering_control(self.actuator,self.control_lease.owner)
        except BaseException:
            # A failed second socket must not leave the telemetry client or
            # simulator connection open and prevent the node from restarting.
            for resource in (self.actuator, self.vehicle, self.bng):
                try:
                    resource.disconnect()
                except Exception:
                    pass
            raise
        self.actuator_ok = True
        self.vehicle.sensors.attach('wildos_control_electrics', Electrics())
        self.command = None
        self.command_at = 0.
        self.command_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.latest_state = None
        self.latest_electrics = {}
        self.last_poll_at = 0.
        self.poll_roundtrip_ms = None
        self.last_poll_error = None
        self.last_poll_warning_at = 0.
        self.last_control_error = None
        self.command_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.io_group = MutuallyExclusiveCallbackGroup()
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(StateSensor, '/wildos/native_state', QoSProfile(depth=1))
        self.telemetry = self.create_publisher(String, '/wildos/native_control_status', 10)
        self.create_subscription(VehicleControl, '/wildos/native_control', self.on_control,
                                 QoSProfile(depth=1), callback_group=self.command_group)
        self.create_timer(.05, self.poll_tick, callback_group=self.io_group)
        self.create_timer(.05, self.control_tick, callback_group=self.control_group)
        self.get_logger().info('Independent actuator connected; camera/LiDAR socket retained')

    def on_control(self, command):
        values = (command.throttle, command.brake, command.steering,
                  command.parkingbrake, command.clutch)
        if not all(math.isfinite(v) for v in values):
            with self.command_lock:
                self.command = None
            return
        with self.command_lock:
            self.command = command
            self.command_at = time.monotonic()

    def poll_tick(self):
        started = time.monotonic()
        try:
            self.vehicle.sensors.poll()
            state = dict(self.vehicle.state)
            electrics = dict(self.vehicle.sensors['wildos_control_electrics'])
            if not all(math.isfinite(float(v)) for v in (
                    *state['pos'], *state['vel'], *state['dir'], *state['up'])):
                raise ValueError('BeamNG returned non-finite vehicle state')
        except Exception as exc:
            self.last_poll_error = f'{type(exc).__name__}: {exc}'[:240]
            now = time.monotonic()
            if now - self.last_poll_warning_at >= 2.:
                self.last_poll_warning_at = now
                self.get_logger().warning('BeamNG telemetry poll failed: ' + self.last_poll_error)
            return

        with self.state_lock:
            self.latest_state = state
            self.latest_electrics = electrics
            # The response has no independent acquisition timestamp. The
            # request start is the conservative bound for sample freshness;
            # receipt after a slow RPC must not make the old sample young.
            self.last_poll_at = started
            self.poll_roundtrip_ms = (time.monotonic() - started) * 1000
            self.last_poll_error = None

        if time.monotonic() - started > float(self.get_parameter('state_timeout').value):
            return  # Do not stamp an aged sample as a new StateSensor.

        msg = StateSensor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('vehicle').value
        msg.position.x, msg.position.y, msg.position.z = state['pos']
        msg.velocity.x, msg.velocity.y, msg.velocity.z = state['vel']
        msg.dir.x, msg.dir.y, msg.dir.z = state['dir']
        msg.up.x, msg.up.y, msg.up.z = state['up']
        self.pub.publish(msg)

        speed = math.hypot(*state['vel'][:2])
        with self.command_lock:
            command = self.command
            command_at = self.command_at
        age = max(0., time.monotonic() - command_at)
        # Read detailed contact only after a sustained commanded standstill.
        # This optional query stays on the telemetry socket, never the actuator.
        if (command is not None and age <= self.get_parameter('command_timeout').value
                and command.throttle > .05 and speed < .1):
            if self.stationary_since is None:
                self.stationary_since = started
            if (started - self.stationary_since >= 1.
                    and started - self.wheel_probe_at >= self.get_parameter('wheel_probe_interval_sec').value):
                self.wheel_probe_at = started
                try:
                    wheels = json.loads(self.vehicle.queue_lua_command(WHEEL_SNAPSHOT_LUA, response=True))
                    self.wheel_snapshot = dict(wall_time=time.time(), wheels=wheels,
                        diagnosis=classify_wheel_contact(wheels, speed))
                except Exception as exc:
                    self.wheel_snapshot = None
                    self.get_logger().warning('Wheel contact probe unavailable: ' + str(exc)[:120])
        else:
            self.stationary_since = None
            self.wheel_snapshot = None

    def control_tick(self):
        started = time.monotonic()
        with self.state_lock:
            state = dict(self.latest_state) if self.latest_state is not None else None
            electrics = dict(self.latest_electrics)
            last_poll_at = self.last_poll_at
            poll_error = self.last_poll_error
            poll_roundtrip_ms = self.poll_roundtrip_ms
        telemetry_age = max(0., started - last_poll_at) if last_poll_at else None
        state_fresh = state is not None and telemetry_age <= float(
            self.get_parameter('state_timeout').value)
        speed = math.hypot(*state['vel'][:2]) if state is not None else 0.
        with self.command_lock:
            command = self.command
            command_at = self.command_at
        age = max(0., time.monotonic() - command_at)
        stop = (not state_fresh or command is None
                or age > self.get_parameter('command_timeout').value
                or speed_exceeds_limit(speed, self.get_parameter('overspeed_limit').value))
        if stop or not self.actuator_ok:
            # Keep the requested gear while braking. Forcing drive here turns a
            # delayed reverse command into an unnecessary D/R oscillation.
            gear = 2 if command is None else int(command.gear)
            control = dict(throttle=0., brake=1., parkingbrake=0., steering=0., clutch=0., gear=gear)
        else:
            control = dict(throttle=min(max(command.throttle, 0.), self.get_parameter('max_throttle').value),
                           brake=min(max(command.brake, 0.), 1.),
                           parkingbrake=min(max(command.parkingbrake, 0.), 1.),
                           clutch=min(max(command.clutch, 0.), 1.),
                           steering=max(-self.get_parameter('max_steering').value,
                                        min(command.steering, self.get_parameter('max_steering').value)),
                           gear=int(command.gear))
        acknowledged = False
        rpc_attempted = self.actuator_ok
        rpc_started = time.monotonic()
        if self.actuator_ok:
            try:
                self.actuator.control(**control)
                acknowledged = True
                self.last_control_error = None
            except Exception as exc:
                self.last_control_error = f'{type(exc).__name__}: {exc}'[:240]
                self.actuator_ok = False
                self.get_logger().error('BeamNG actuator failed closed: ' + self.last_control_error)
                try:
                    self.actuator.disconnect()
                except Exception:
                    pass
        finished = time.monotonic()
        # Keep the sample and its age paired across the synchronous control
        # RPC. A newly polled state belongs to the next tick, not this record.
        telemetry_age = max(0., finished - last_poll_at) if last_poll_at else None
        state_fresh = state is not None and telemetry_age <= float(
            self.get_parameter('state_timeout').value)
        age = max(0., finished - command_at)
        self.telemetry.publish(String(data=json.dumps({
            'wall_time': time.time(), 'sim_time': state.get('time') if state else None,
            'position': state.get('pos') if state else None,
            'direction': state.get('dir') if state else None,
            'up': state.get('up') if state else None,
            'velocity': state.get('vel') if state else None,
            'speed': speed, 'telemetry_age_sec': telemetry_age,
            'telemetry_ok': state_fresh, 'telemetry_error': poll_error,
            'actuator_ok': self.actuator_ok,
            'actuator_error': self.last_control_error,
            'control_error': self.last_control_error,
            'control_acknowledged': acknowledged,
            'steering_control_interface': 'beamng_direct_input_v1',
            'vehicle_control_lease_version': LEASE_VERSION,
            'control_rpc_ms': (finished - rpc_started) * 1000 if rpc_attempted else None,
            'poll_roundtrip_ms': poll_roundtrip_ms,
            'period': started - self.last_tick,
            'roundtrip_ms': (finished - started) * 1000, 'command_age': age,
            'watchdog_stop': bool(stop or not self.actuator_ok), 'sent': control,
            'wheel_contact': self.wheel_snapshot,
            'actual': {k: electrics.get(k) for k in ('throttle', 'brake', 'steering',
                       'steering_input', 'steeringUnassisted', 'gear', 'gear_index', 'gearboxMode',
                       'rpm', 'engineRunning', 'fuel', 'wheelspeed', 'clutchRatio1',
                       'wildosControlLeaseTripped')}
        })))
        self.last_tick = started

    def stop(self):
        try:
            control_error = None
            if self.actuator_ok:
                try:
                    self.actuator.control(throttle=0, brake=1, parkingbrake=1,
                                          steering=0, clutch=0, gear=0)
                except Exception as exc:
                    control_error = exc
            # A failed actuator is not reconnected. Cleanup may only release
            # its own lease through the existing telemetry stream, after the
            # vehicle-local guard confirms standstill; otherwise brakes stay held.
            self.control_lease.release(self.actuator if self.actuator_ok and control_error is None else self.vehicle)
            if control_error is not None:
                raise control_error
        finally:
            try:
                self.actuator.disconnect()
            finally:
                self.vehicle.disconnect()
                self.bng.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = NativeVehicleControl()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        node.bng.control.pause()
        raise
    finally:
        executor.shutdown()
        try:
            node.stop()
        except Exception as exc:
            node.get_logger().error(f'Actuator shutdown control failed: {exc!r}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
