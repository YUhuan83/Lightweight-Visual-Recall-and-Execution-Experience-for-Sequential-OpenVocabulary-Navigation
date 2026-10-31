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
from beamngpy.sensors import Electrics
from beamng_msgs.msg import StateSensor, VehicleControl
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from .drive_math import speed_exceeds_limit


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
        self.declare_parameter('overspeed_limit', 0.0)
        self.declare_parameter('max_throttle', .12)
        self.declare_parameter('max_steering', .6)
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
        self.vehicle.ai.set_mode('disabled')
        self.vehicle.set_shift_mode('realistic_automatic')
        self.vehicle.sensors.attach('wildos_control_electrics', Electrics())
        self.command = None
        self.command_at = 0.
        self.command_lock = threading.Lock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.io_group = MutuallyExclusiveCallbackGroup()
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(StateSensor, '/wildos/native_state', QoSProfile(depth=1))
        self.telemetry = self.create_publisher(String, '/wildos/native_control_status', 10)
        self.create_subscription(VehicleControl, '/wildos/native_control', self.on_control,
                                 QoSProfile(depth=1), callback_group=self.control_group)
        self.create_timer(.05, self.tick, callback_group=self.io_group)
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

    def tick(self):
        started = time.monotonic()
        self.vehicle.sensors.poll()
        state = dict(self.vehicle.state)
        speed = math.hypot(*state['vel'][:2])
        msg = StateSensor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('vehicle').value
        msg.velocity.x, msg.velocity.y, msg.velocity.z = state['vel']
        self.pub.publish(msg)
        with self.command_lock:
            command = self.command
            command_at = self.command_at
        age = max(0., time.monotonic() - command_at)
        stop = (command is None or age > self.get_parameter('command_timeout').value
                or speed_exceeds_limit(speed, self.get_parameter('overspeed_limit').value))
        if stop:
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
        self.vehicle.control(**control)
        electrics = dict(self.vehicle.sensors['wildos_control_electrics'])
        self.telemetry.publish(String(data=json.dumps({
            'wall_time': time.time(), 'sim_time': state['time'], 'position': state['pos'],
            'direction': state.get('dir'), 'up': state.get('up'),
            'velocity': state['vel'], 'speed': speed, 'period': started - self.last_tick,
            'roundtrip_ms': (time.monotonic() - started) * 1000, 'command_age': age,
            'watchdog_stop': stop, 'sent': control,
            'actual': {k: electrics.get(k) for k in ('throttle', 'brake', 'steering',
                       'steering_input', 'gear', 'gear_index', 'gearboxMode',
                       'rpm', 'engineRunning', 'fuel')}
        })))
        self.last_tick = started

    def stop(self):
        try:
            self.vehicle.control(throttle=0, brake=1, parkingbrake=1, steering=0, clutch=0, gear=0)
        finally:
            self.vehicle.disconnect()
            self.bng.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = NativeVehicleControl()
    executor = MultiThreadedExecutor(num_threads=2)
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
