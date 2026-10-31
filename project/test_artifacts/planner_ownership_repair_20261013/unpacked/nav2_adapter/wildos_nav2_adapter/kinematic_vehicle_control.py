"""ROS velocity commands -> fixed-node BeamNG pose, for physics isolation.

Must replace both native_vehicle_control and cmd_vel_to_control. Remap the
official adapter StateSensor input to /wildos/kinematic_state. Measured engine
positions produce velocity; commanded motion is never published as odometry.
The current diagnostic backend holds initial height and pitch/roll.
"""
import json
import math
import threading
import time
from copy import deepcopy

from beamngpy import BeamNGpy
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from beamng_msgs.msg import StateSensor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from .drive_math import bounded_cmd_velocity
import rclpy
from rclpy.node import Node
from .vehicle_geometry import SBR_BASE_OFFSET
from .kinematic_rpc_worker import KinematicRpcWorker
from .fail_closed_transport import install_fail_closed_reads


class KinematicVehicleControl(Node):
    def __init__(self):
        super().__init__('wildos_kinematic_vehicle_control')
        self.declare_parameter('enable', False)
        self.declare_parameter('carrier_model', 'ideal_point')
        self.declare_parameter('max_angular_rate', .8)
        self.declare_parameter('minimum_turning_radius', 0.0)
        self.carrier_model = str(self.get_parameter('carrier_model').value).strip().lower()
        self.max_angular_rate = float(self.get_parameter('max_angular_rate').value)
        self.minimum_turning_radius = float(self.get_parameter('minimum_turning_radius').value)
        if self.carrier_model not in ('ideal_point', 'ackermann'):
            raise ValueError('carrier_model must be ideal_point or ackermann')
        if not math.isfinite(self.max_angular_rate) or self.max_angular_rate <= 0:
            raise ValueError('max_angular_rate must be a positive finite number')
        if (self.carrier_model == 'ackermann'
                and (not math.isfinite(self.minimum_turning_radius)
                     or self.minimum_turning_radius <= 0)):
            raise ValueError('Ackermann carrier requires a positive minimum_turning_radius')
        self.enabled = bool(self.get_parameter('enable').value)
        self.add_on_set_parameters_callback(self.on_parameters)
        # A BeamNG timeout can leave a partial length-prefixed response. Close
        # that socket instead of letting BeamNGpy reuse it for another request.
        install_fail_closed_reads(PrefixedLengthSocket)
        # Establish the GE/TCom connection on the RPC worker. Connecting or
        # initializing Lua in __init__ used to prevent the ROS node from
        # starting whenever BeamNG was paused or its command queue was busy.
        self.bng = None
        self.lua_started = False
        self.command, self.command_at, self.previous = (0., 0.), 0., None
        self.measured_velocity = [0.,0.,0.]
        self.command_lock = threading.Lock()
        self.last_state_received_at = None
        self.last_state_stamp_ns = None
        self.pub = self.create_publisher(StateSensor, '/wildos/kinematic_state', 1)
        self.native = self.create_publisher(String, '/wildos/native_control_status', 10)
        self.control = self.create_publisher(String, '/wildos/control_status', 10)
        self.create_subscription(Twist, '/wildos/cmd_vel', self.on_command, 1)
        self.create_subscription(StateSensor, '/vehicles/ego/sensors/state', self.on_state, 1)
        self.create_timer(.05, self.tick)
        self.rpc_worker = KinematicRpcWorker(self.exchange, self.stop_rpc)
        # Do not touch BeamNG TCom until a live source-state stream exists.
        self.rpc_worker.set_active(False)
        self.rpc_worker.start()

    def on_state(self, source):
        # Keep exactly the sensor bridge's source clock. ROS wall time here
        # would make the official adapter reject LiDAR and camera frames.
        source_ns = source.header.stamp.sec*10**9 + source.header.stamp.nanosec
        position = [source.position.x,source.position.y,source.position.z]
        velocity = [0.,0.,0.]
        if self.previous:
            previous_stamp, previous_position = self.previous
            if source_ns <= previous_stamp:
                if source_ns < previous_stamp - 500_000_000:
                    # BeamNG restarted its sensor clock. Publish this sample
                    # as a new epoch, with zero derivative across the reset.
                    self.previous = None
                else:
                    return
            if self.previous is not None:
                previous_stamp, previous_position = self.previous
                dt = (source_ns-previous_stamp)*1e-9
                if 0. < dt < .5:
                    velocity = [(p-q)/dt for p,q in zip(position,previous_position)]
        self.previous = source_ns, position
        self.measured_velocity = velocity
        self.last_state_received_at = time.monotonic()
        self.last_state_stamp_ns = source_ns
        msg = deepcopy(source)
        msg.velocity.x,msg.velocity.y,msg.velocity.z = velocity
        self.pub.publish(msg)

    def on_parameters(self, parameters):
        # Validate the whole batch before mutating the actuator. These profile
        # values are cached at startup and also initialize the Lua backend;
        # reporting a successful ROS parameter change without changing either
        # implementation makes runtime preflight describe the wrong carrier.
        for parameter in parameters:
            if parameter.name == 'enable':
                if type(parameter.value) is not bool:
                    return SetParametersResult(successful=False, reason='enable must be boolean')
            elif parameter.name == 'use_sim_time':
                if parameter.value != self.get_parameter('use_sim_time').value:
                    return SetParametersResult(successful=False,
                        reason='restart the coherent navigation stack to change use_sim_time')
            else:
                return SetParametersResult(successful=False,
                    reason='restart kinematic controller to change ' + parameter.name)
        for parameter in parameters:
            if parameter.name == 'enable':
                was_enabled = self.enabled
                self.enabled = parameter.value
                if self.enabled and not was_enabled:
                    # A false-to-true transition is an explicit retry after
                    # the worker opened its circuit on a BeamNG timeout.
                    self.rpc_worker.retry()
                # Enabling never replays a command from before the change.
                with self.command_lock:
                    self.command, self.command_at = (0., 0.), 0.
                self.rpc_worker.set_command(0., 0.)
        return SetParametersResult(successful=True)

    def on_command(self, msg):
        raw_v, raw_w = msg.linear.x, msg.angular.z
        if not all(math.isfinite(x) for x in (raw_v, raw_w)):
            with self.command_lock:
                self.command, self.command_at = (0., 0.), 0.
            return
        # The ideal point carrier accepts independent linear and angular
        # velocity, including rotation at zero translation. Keep the legacy
        # Ackermann constraint available only when that model is selected.
        radius = (self.minimum_turning_radius
                  if self.carrier_model == 'ackermann' else None)
        v, w = bounded_cmd_velocity(raw_v, raw_w, 0., self.max_angular_rate, True, radius)
        with self.command_lock:
            self.command, self.command_at = (v, w), time.monotonic()

    def exchange(self, v, w):
        try:
            if self.bng is None:
                self.bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=2).open(launch=False)
                self.lua_started = False
            if not self.lua_started:
                start = (
                    "local v=scenetree.findObject('ego'); assert(v); "
                    f"extensions.wildosKinematic.start(v:getID(),{SBR_BASE_OFFSET[0]},"
                    f"{SBR_BASE_OFFSET[1]},{self.carrier_model!r},"
                    f"{self.minimum_turning_radius}); "
                )
            else:
                start = ''
            response = self.bng.control.queue_lua_command(
                start + f'extensions.wildosKinematic.command({v},{w}); '
                'return jsonEncode(extensions.wildosKinematic.status())', response=True)
            data = json.loads(response)
            if not isinstance(data, dict):
                raise ValueError('BeamNG returned a non-object kinematic status')
            for name in ('position', 'direction', 'up'):
                vector = data.get(name)
                if (not isinstance(vector, (list, tuple)) or len(vector) != 3
                        or not all(math.isfinite(float(value)) for value in vector)):
                    raise ValueError(f'BeamNG returned invalid {name}')
            self.lua_started = True
            return data
        except Exception:
            # A timeout makes request/response framing ambiguous. Discard the
            # connection and let the worker retry with a fresh BeamNG session.
            self.disconnect_beamng()
            raise

    def disconnect_beamng(self):
        bng, self.bng = self.bng, None
        self.lua_started = False
        if bng is not None:
            try:
                bng.disconnect()
            except Exception as exc:
                self.get_logger().warning(f'BeamNG reconnect cleanup failed: {exc!r}')

    def stop_rpc(self):
        if self.bng is not None and self.lua_started:
            self.bng.control.queue_lua_command(
                'extensions.wildosKinematic.stop(); return true', response=True)

    def tick(self):
        now = time.monotonic()
        with self.command_lock:
            command, command_at = self.command, self.command_at
        state_age = (None if self.last_state_received_at is None else
                     max(0., now-self.last_state_received_at))
        state_ok = state_age is not None and state_age <= .5
        valid = self.enabled and now-command_at <= .5 and state_ok
        v, w = command if valid else (0., 0.)
        self.rpc_worker.set_active(state_ok)
        self.rpc_worker.set_command(v, w)
        rpc = self.rpc_worker.snapshot(now)
        data = rpc['data'] or {}
        telemetry_ok = bool(rpc['telemetry_ok'])
        velocity = list(self.measured_velocity)
        stamp = self.get_clock().now()
        active = valid and telemetry_ok and (abs(v) > .01 or abs(w) > .01)
        self.native.publish(String(data=json.dumps(dict(
            wall_time=time.time(), rpc_wall_time=rpc['rpc_wall_time'],
            telemetry_age_sec=rpc['telemetry_age_sec'], telemetry_ok=telemetry_ok,
            telemetry_error=rpc['error'], state_ok=state_ok, state_age_sec=state_age,
            state_stamp_ns=self.last_state_stamp_ns,
            position=data.get('position'), direction=data.get('direction'), up=data.get('up'),
            velocity=velocity, speed=math.hypot(*velocity[:2]),
            actuator_ok=rpc['actuator_ok'], control_error=rpc['error'],
            watchdog_stop=not active, wheel_contact=None, actual={},
            physics_bypassed=True, profile=data.get('profile', 'unavailable'), backend='kinematic',
            carrier_model=self.carrier_model,
            minimum_turning_radius_m=(self.minimum_turning_radius
                if self.carrier_model == 'ackermann' else 0.0),
            beamng_carrier_model=data.get('carrierModel'),
            beamng_minimum_turning_radius_m=data.get('minimumTurningRadius'),
            beamng_ros_yaw_to_cluster_sign=data.get('rosYawToClusterSign'),
            beamng_rotation_convention=data.get('rotationConvention'),
            beamng_visual_meshes_hidden=data.get('visualMeshesHidden'),
            sensors={'cameras': data.get('cameras', []), 'lidars': data.get('lidars', [])},
            visuals={'meshes_hidden': data.get('visualMeshesHidden', False),
                     'mesh_count': data.get('visualMeshCount', 0),
                     'hidden_mesh_count': data.get('hiddenMeshCount', 0)},
            dropped_dt_updates=data.get('droppedDtUpdates'),
            stale_command_updates=data.get('staleCommandUpdates'),
            last_dt_sim=data.get('lastDtSim')))))
        self.control.publish(String(data=json.dumps(dict(
            stamp_ns=stamp.nanoseconds, actuating=active,
            target_speed=v, target_angular=w, backend='kinematic',
            telemetry_ok=telemetry_ok, telemetry_age_sec=rpc['telemetry_age_sec'],
            state_ok=state_ok, state_age_sec=state_age,
            carrier_model=self.carrier_model,
            minimum_turning_radius_m=(self.minimum_turning_radius
                if self.carrier_model == 'ackermann' else 0.0)))))

    def stop(self):
        try:
            if not self.rpc_worker.close(timeout_sec=4.5):
                self.get_logger().error('BeamNG kinematic worker did not stop within 4.5 seconds')
        finally:
            self.command, self.command_at = (0., 0.), 0.
            self.disconnect_beamng()


def main(args=None):
    rclpy.init(args=args)
    node = KinematicVehicleControl()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
