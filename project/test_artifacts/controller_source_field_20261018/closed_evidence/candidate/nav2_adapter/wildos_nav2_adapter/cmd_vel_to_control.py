"""Safe /wildos/cmd_vel -> beamng_msgs/VehicleControl bridge.

Safety rules (all enforced here, not in the vehicle):
  * disabled by default; requires -p enable:=true to actuate anything
  * optional speed ceiling (zero disables it), bounded steering and angular rate
  * zero-command timeout forces brakes
  * reverse is opt-in via -p allow_reverse:=true
  * direction changes wait for the vehicle to nearly stop
  * publishes a fixed-rate control stream so the vehicle never keeps stale input
"""
from __future__ import annotations

import math
import json
import time
from std_msgs.msg import String

import rclpy
from rclpy.clock import Clock, ClockType
from beamng_msgs.msg import StateSensor, VehicleControl
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from .planning import update_direction_shift
from .execution_hold import ExecutionHoldReceiver
from .source_control import (SourceControlSender, PROTOCOL, LEGACY_PROTOCOL, COMMAND_TOPIC,
                             CONTROLLER_TOPIC, CONTROLLER_FRAME, IDENTITY, LIFETIME_NS)


from .drive_math import clamp, steering_from_angular, drive_outputs, bounded_cmd_velocity, speed_exceeds_limit, ThrottleIntegrator, longitudinal_ground_speed


class CmdVelToControl(Node):
    def __init__(self):
        super().__init__("wildos_cmd_vel_to_control")
        self.declare_parameter("cmd_vel_topic", "/wildos/cmd_vel")
        self.declare_parameter("state_topic", "/vehicles/ego/sensors/state")
        self.declare_parameter("control_topic", "/control")
        self.declare_parameter('control_protocol', 'vehicle_control')
        self.control_protocol = self.get_parameter('control_protocol').value
        self.declare_parameter('require_stamped_command', False)
        self.declare_parameter('cmd_vel_frame', CONTROLLER_FRAME)
        self.require_stamped_command = bool(self.get_parameter('require_stamped_command').value)
        if self.require_stamped_command and (
                self.get_parameter('cmd_vel_topic').value != CONTROLLER_TOPIC
                or self.get_parameter('cmd_vel_frame').value != CONTROLLER_FRAME
                or self.control_protocol != PROTOCOL
                or bool(self.get_parameter('use_sim_time').value)):
            raise ValueError('Stamped physical commands require the bound topic/frame/source protocol and system ROS clock')
        if (self.control_protocol not in (PROTOCOL, LEGACY_PROTOCOL)
                or (self.control_protocol == PROTOCOL
                    and self.get_parameter('control_topic').value != COMMAND_TOPIC)
                or (self.control_protocol == LEGACY_PROTOCOL
                    and self.get_parameter('control_topic').value in (COMMAND_TOPIC, '/wildos/native_control'))):
            raise ValueError('control topic/protocol mismatch; restart the complete control chain')
        self.source_sender = SourceControlSender()
        self.source_receipt = None
        self.declare_parameter("enable", False)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("max_speed", 0.0)
        self.declare_parameter("max_steering", 0.6)
        # The Town ego vehicle turns opposite to the ROS yaw convention when
        # BeamNG receives positive steering, so invert the adapter output.
        self.declare_parameter("steer_sign", -1.0)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("minimum_turning_radius", 8.0)
        # Load compensation is bounded by the speed-loop policy and a throttle
        # rise limit. The transport must not silently add a second 12% cap.
        self.declare_parameter("throttle_gain", 0.20)
        self.declare_parameter("max_throttle", 0.35)
        self.declare_parameter("throttle_integral_gain", .05)
        self.declare_parameter("throttle_rise_rate", .3)
        self.declare_parameter("min_throttle", 0.06)
        self.declare_parameter("brake_gain", 0.5)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("state_timeout", 0.5)
        # SBR wheel hubs measure 2.501 m. Saturated 0.6 input gives a rear-axle
        # radius of 7.407 m, or about 0.544 rad per normalized steering input.
        self.declare_parameter("wheelbase", 2.501)
        self.declare_parameter("max_steer_angle", 0.54)
        self.declare_parameter("overspeed_limit", 0.0)
        self.declare_parameter("direction_change_speed", 0.15)
        self.declare_parameter("gear_change_confirmation_time", 2.0)
        # BeamNG realistic gearbox mode: reverse=-1, neutral=0, park=1, drive=2.
        self.declare_parameter("gear_forward", 2)
        self.declare_parameter("gear_reverse", -1)

        self.enabled = bool(self.get_parameter("enable").value)
        self.execution_hold = ExecutionHoldReceiver()
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self._safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.current_gear = int(self.get_parameter("gear_forward").value)
        self.pending_gear = None
        self.pending_gear_since = None
        self.raw_curvature = 0.0
        self.last_source_state_ns = 0
        self.last_source_state_frame = ''
        self.target_speed = 0.0
        self.target_angular = 0.0
        self.speed = 0.0
        self.longitudinal_speed = 0.0
        self.last_command_time = 0.0
        self.controller_stamp_ros_ns = 0
        self.controller_minimum_stamp_ns = 0
        self.controller_deadline_steady_ns = None
        self.controller_rejection = None
        self.controller_stop_active = False
        self.last_state_time = 0.0
        self.throttle_loop = ThrottleIntegrator()
        self.last_control_time = None
        self.native_motion = None
        self.native_motion_at = 0.
        self.native_control_ok = False
        self.native_control_at = 0.
        self.native_sample_age_at_receipt = 0.
        self.native_source_wall_time = None
        finite_parameters = (
            "max_speed", "max_steering", "steer_sign", "max_angular", "minimum_turning_radius",
            "throttle_gain", "max_throttle", "min_throttle", "brake_gain",
            "command_timeout", "state_timeout", "wheelbase",
            "max_steer_angle", "overspeed_limit", "direction_change_speed",
            "gear_change_confirmation_time",
            "throttle_integral_gain", "throttle_rise_rate",
        )
        for name in finite_parameters:
            if not math.isfinite(float(self.get_parameter(name).value)):
                raise ValueError(f"{name} must be finite")
        if float(self.get_parameter("max_speed").value) < 0:
            raise ValueError("max_speed must be non-negative; zero disables the ceiling")
        radius = float(self.get_parameter('minimum_turning_radius').value)
        physical_radius = float(self.get_parameter('wheelbase').value) / math.tan(
            float(self.get_parameter('max_steering').value) * float(self.get_parameter('max_steer_angle').value))
        self.physical_turning_radius = physical_radius
        if radius < physical_radius:
            raise ValueError('minimum_turning_radius exceeds vehicle steering capability')
        if float(self.get_parameter("max_angular").value) <= 0:
            raise ValueError("max_angular must be positive")
        if not 0 < float(self.get_parameter("max_steering").value) <= 1:
            raise ValueError("max_steering must be in (0, 1]")
        if abs(float(self.get_parameter("steer_sign").value)) != 1.0:
            raise ValueError("steer_sign must be either -1 or 1")
        if float(self.get_parameter("wheelbase").value) <= 0:
            raise ValueError("wheelbase must be positive")
        if not 0 < float(self.get_parameter("max_steer_angle").value) < math.pi / 2:
            raise ValueError("max_steer_angle must be in (0, pi/2)")
        if float(self.get_parameter("overspeed_limit").value) < 0:
            raise ValueError("overspeed_limit must be non-negative; zero disables the cutoff")
        if float(self.get_parameter("throttle_gain").value) < 0:
            raise ValueError("throttle_gain must be non-negative")
        if (float(self.get_parameter('throttle_integral_gain').value) < 0
                or float(self.get_parameter('throttle_rise_rate').value) <= 0):
            raise ValueError('invalid throttle integral gain or rise rate')
        if float(self.get_parameter("brake_gain").value) < 0:
            raise ValueError("brake_gain must be non-negative")
        if float(self.get_parameter("command_timeout").value) <= 0:
            raise ValueError("command_timeout must be positive")
        if float(self.get_parameter("state_timeout").value) <= 0:
            raise ValueError("state_timeout must be positive")
        if not 0.0 <= float(self.get_parameter("min_throttle").value) \
                <= float(self.get_parameter("max_throttle").value) <= 1.0:
            raise ValueError("throttle bounds must satisfy 0 <= min <= max <= 1")
        if float(self.get_parameter("direction_change_speed").value) < 0:
            raise ValueError("direction_change_speed must be non-negative")
        if float(self.get_parameter("gear_change_confirmation_time").value) < 0:
            raise ValueError("gear_change_confirmation_time must be non-negative")
        if self.current_gear == int(self.get_parameter("gear_reverse").value):
            raise ValueError("gear_forward and gear_reverse must differ")

        self.control_status_pub = self.create_publisher(String, "/wildos/control_status", 10)
        self.pub = self.create_publisher(
            String if self.control_protocol == PROTOCOL else VehicleControl,
            str(self.get_parameter("control_topic").value), 10)
        self.create_subscription(
            TwistStamped if self.require_stamped_command else Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.on_stamped_cmd_vel if self.require_stamped_command else self.on_cmd_vel, 1)
        if self.require_stamped_command:
            # Humble controller_server also emits terminal zero velocity
            # outside its controller plugin. Preserve this brake-only channel;
            # it can never supply or refresh a nonzero motion demand.
            self.create_subscription(Twist, '/wildos/cmd_vel', self.on_controller_stop, 1)
        self.create_subscription(
            StateSensor, str(self.get_parameter("state_topic").value), self.on_state, 10)
        self.create_subscription(String, '/wildos/native_control_status', self.on_native_motion, 10)
        self.create_subscription(String, '/wildos/execution_hold', self.on_execution_hold, 10)
        self.create_timer(0.05, self.tick, clock=self._safety_clock)
        self.add_on_set_parameters_callback(self.on_parameters)
        level = "ENABLED" if self.enabled else "DISABLED (dry run)"
        self.get_logger().warn(
            f"cmd_vel->control adapter {level}; reverse "
            f"{'enabled' if self.allow_reverse else 'disabled'}; publish to "
            f"{self.get_parameter('cmd_vel_topic').value}")

    def on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'enable':
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(successful=False, reason='enable must be bool')
            elif parameter.name == 'use_sim_time':
                if self.require_stamped_command and parameter.value is not False:
                    return SetParametersResult(successful=False, reason='Physical controller source time requires system ROS time')
            else:
                return SetParametersResult(successful=False, reason='restart to change ' + parameter.name)
        for parameter in parameters:
            if parameter.name == 'enable':
                self.enabled = parameter.value
                self.invalidate_controller_command()
                if self.control_protocol == PROTOCOL:
                    self.source_sender.new_generation()
                    self.source_receipt = None
                    self.native_control_ok = False
                    self.tick()  # Every new ownership generation begins with brakes.
                elif not self.enabled:
                    self.tick()  # Publish brakes immediately on disable.
        return SetParametersResult(successful=True)

    def on_cmd_vel(self, msg: Twist):
        if self.require_stamped_command:
            return  # A legacy Twist must not become a freshness fallback.
        self.apply_cmd_vel(msg)

    def invalidate_controller_command(self):
        self.target_speed = self.target_angular = self.last_command_time = 0.
        self.controller_deadline_steady_ns = None
        self.controller_stop_active = False
        if self.require_stamped_command:
            self.controller_minimum_stamp_ns = self.get_clock().now().nanoseconds

    def on_controller_stop(self, msg: Twist):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if (not self.require_stamped_command or not all(math.isfinite(v) and v == 0. for v in values)):
            return
        self.invalidate_controller_command()
        self.last_command_time=self._safety_clock.now().nanoseconds*1e-9
        self.controller_stop_active=True
        self.controller_rejection='controller_safety_stop'
        self.tick()

    def controller_remaining_ns(self, steady_ns=None, ros_ns=None):
        if self.controller_deadline_steady_ns is None or not self.controller_stamp_ros_ns:
            return 0
        steady_ns = self._safety_clock.now().nanoseconds if steady_ns is None else steady_ns
        ros_ns = self.get_clock().now().nanoseconds if ros_ns is None else ros_ns
        age = ros_ns-self.controller_stamp_ros_ns
        if age < 0 or self.controller_stamp_ros_ns <= self.controller_minimum_stamp_ns:
            return 0
        return max(0, min(self.controller_deadline_steady_ns-steady_ns,
            min(LIFETIME_NS, round(float(self.get_parameter('command_timeout').value)*1e9))-age))

    def on_stamped_cmd_vel(self, msg: TwistStamped):
        if not self.require_stamped_command or self.execution_hold.held:
            return
        stamp = int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
        if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 10**9:
            self.controller_rejection = 'controller_source_time'
            return
        now_ros = self.get_clock().now().nanoseconds
        lifetime = min(LIFETIME_NS, round(float(self.get_parameter('command_timeout').value)*1e9))
        age = now_ros-stamp
        if msg.header.frame_id != self.get_parameter('cmd_vel_frame').value:
            self.controller_rejection = 'controller_frame'
            return
        if (stamp <= self.controller_minimum_stamp_ns or stamp <= self.controller_stamp_ros_ns
                or not 0 <= age < lifetime):
            self.controller_rejection = 'controller_source_time'
            return
        received = self._safety_clock.now().nanoseconds
        if not self.apply_cmd_vel(msg.twist):
            self.controller_deadline_steady_ns = None
            self.controller_rejection = 'controller_nonfinite'
            self.controller_stop_active = False
            return
        self.controller_stamp_ros_ns = stamp
        self.controller_stop_active = False
        self.controller_deadline_steady_ns = received+lifetime-age
        # Preserve receipt-age diagnostics; the separate source deadline is
        # what charges pre-callback delay and stops motion at original expiry.
        self.last_command_time = received*1e-9
        self.controller_rejection = None

    def apply_cmd_vel(self, msg: Twist):
        if self.execution_hold.held:
            return False
        max_speed = float(self.get_parameter("max_speed").value)
        max_angular = float(self.get_parameter("max_angular").value)
        raw_speed, raw_angular = float(msg.linear.x), float(msg.angular.z)
        if not math.isfinite(raw_speed) or not math.isfinite(raw_angular):
            self.target_speed = 0.0
            self.target_angular = 0.0
            self.last_command_time = 0.0
            self.get_logger().error("Rejected non-finite cmd_vel")
            return False
        self.raw_curvature = raw_angular / raw_speed if abs(raw_speed) > .01 else 0.
        speed, angular = bounded_cmd_velocity(
            raw_speed, raw_angular, max_speed, max_angular, self.allow_reverse,
            # A conservative planner radius is not the vehicle's steering
            # limit. RPP needs the remaining physical authority to correct
            # cross-track error after a cusp or a disturbed wheel contact.
            self.physical_turning_radius)
        self.target_speed = speed
        self.target_angular = angular
        self.last_command_time = self._safety_clock.now().nanoseconds * 1e-9
        return True

    def on_state(self, msg: StateSensor):
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if stamp <= self.last_source_state_ns:
            return
        self.last_source_state_ns = stamp
        self.last_source_state_frame = msg.header.frame_id
        # Control speed along the drivable ground plane. Vertical suspension
        # motion should not trigger longitudinal braking.
        self.speed = math.hypot(
            float(msg.velocity.x), float(msg.velocity.y))
        self.longitudinal_speed = longitudinal_ground_speed(
            float(msg.velocity.x), float(msg.velocity.y),
            float(msg.dir.x), float(msg.dir.y))
        if not math.isfinite(self.speed):
            self.speed = float("inf")
        self.last_state_time = self._safety_clock.now().nanoseconds * 1e-9

    def on_native_motion(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or not isinstance(value.get('actual'), dict):
                return
            age = time.time() - float(value['wall_time'])
            if not math.isfinite(age) or not 0. <= age <= .5:
                return  # Invalid source time must not poison later valid samples.
            if (self.native_source_wall_time is not None
                    and float(value['wall_time']) <= self.native_source_wall_time):
                return
            self.native_source_wall_time = float(value['wall_time'])
            telemetry_age = value.get('telemetry_age_sec')
            sample_age = age + float(telemetry_age) if telemetry_age is not None else float('inf')
            self.native_sample_age_at_receipt = sample_age
            self.native_control_ok = (
                value.get('actuator_ok') is True
                and value.get('control_acknowledged', True) is True
                and value.get('telemetry_ok') is True
                and not value.get('control_error')
                and age >= 0.
                and age <= .5
                and telemetry_age is not None
                and math.isfinite(float(telemetry_age))
                and float(telemetry_age) >= 0.
                and math.isfinite(sample_age) and sample_age <= .5
            )
            if self.control_protocol == PROTOCOL:
                self.source_receipt = value.get('source_receipt')
                self.native_control_ok = bool(self.native_control_ok
                    and value.get('control_protocol') == PROTOCOL
                    and self.source_sender.ack_issue(self.source_receipt,
                        time.time_ns(), time.monotonic_ns()) is None)
            self.native_control_at = self._safety_clock.now().nanoseconds * 1e-9
            if (not self.native_control_ok or value.get('control_error')
                    or value.get('telemetry_ok') is False
                    or (telemetry_age is not None and float(telemetry_age) > .5)):
                self.native_motion = None
                self.native_motion_at = 0.
                return
            speed, wheel_speed = float(value['speed']), float(value['actual']['wheelspeed'])
            if not 0 <= age <= .5 or not all(map(math.isfinite, (speed, wheel_speed))):
                return
            self.native_motion = dict(speed=speed, wheelspeed=wheel_speed)
            self.native_motion_at = self._safety_clock.now().nanoseconds * 1e-9
        except (ValueError, KeyError, TypeError):
            return

    def tick(self):
        control = VehicleControl()
        control.parkingbrake = 0.0
        control.clutch = 0.0
        control.gear = self.current_gear
        now = self._safety_clock.now().nanoseconds * 1e-9
        dt = .05 if self.last_control_time is None else now - self.last_control_time
        self.last_control_time = now
        stale = (now - self.last_command_time) > float(
            self.get_parameter("command_timeout").value)
        if self.require_stamped_command and not self.controller_stop_active:
            stale = stale or self.controller_remaining_ns() <= 0
        state_stale = (now - self.last_state_time) > float(
            self.get_parameter("state_timeout").value)
        native_control_stale = (
            not self.native_control_ok
            or self.native_sample_age_at_receipt + max(0., now - self.native_control_at) > .5)
        if self.control_protocol == PROTOCOL:
            native_control_stale = bool(native_control_stale or self.source_sender.ack_issue(
                self.source_receipt, time.time_ns(), time.monotonic_ns()) is not None)
        # A disabled mission is a session boundary. Once the measured vehicle
        # is stopped and its state is fresh, normalize an old reverse gear so
        # the next mission never inherits a prior maneuver's direction.
        if (not self.enabled and not state_stale
                and self.speed <= float(self.get_parameter("direction_change_speed").value)):
            self.current_gear = int(self.get_parameter("gear_forward").value)
            control.gear = self.current_gear
        rotating = (abs(self.target_angular) > 0.1
                    and abs(self.target_speed) <= 0.01)
        overspeed = speed_exceeds_limit(self.speed, float(self.get_parameter("overspeed_limit").value))
        invalid_direction = not math.isfinite(self.longitudinal_speed)
        progress_speed = (math.copysign(1., self.target_speed) * self.longitudinal_speed
                          if not invalid_direction else 0.)
        opposite_motion = (not invalid_direction and abs(self.target_speed) > .01
                           and progress_speed < -float(self.get_parameter('direction_change_speed').value))

        if self.execution_hold.held or not self.enabled or stale or state_stale or native_control_stale or overspeed or invalid_direction:
            self.throttle_loop.reset()
            self.pending_gear = None
            self.pending_gear_since = None
            control.throttle = 0.0
            control.brake = 1.0
            control.steering = 0.0
        else:
            desired_gear, drive_throttle, drive_brake = drive_outputs(
                self.target_speed,
                max(0., progress_speed),
                float(self.get_parameter("throttle_gain").value),
                int(self.get_parameter("gear_forward").value),
                int(self.get_parameter("gear_reverse").value),
                self.allow_reverse,
                current_gear=self.current_gear,
                brake_gain=float(self.get_parameter("brake_gain").value),
                max_throttle=float(self.get_parameter("max_throttle").value),
                min_throttle=float(self.get_parameter("min_throttle").value),
            )
            shift_speed = float(self.get_parameter("direction_change_speed").value)
            (self.current_gear, self.pending_gear, self.pending_gear_since,
             shifting) = update_direction_shift(
                self.current_gear, desired_gear, self.pending_gear,
                self.pending_gear_since, self.speed, now, shift_speed,
                float(self.get_parameter("gear_change_confirmation_time").value))
            if shifting:
                self.throttle_loop.reset()
                control.throttle = 0.0
                control.brake = 1.0
                control.steering = 0.0
            else:
                control.gear = self.current_gear
                steering = 0.0
                if opposite_motion:
                    # R/D gear alone does not prove actual travel direction.
                    # Brake a rollback before requesting driving torque again.
                    self.throttle_loop.reset()
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.steering = 0.0
                elif rotating or abs(self.target_speed) <= 0.01:
                    self.throttle_loop.reset()
                    # An Ackermann vehicle cannot rotate in place. A yaw-only
                    # command must stop safely instead of adding hidden throttle.
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.steering = 0.0
                    self.publish_control(control, 'ackermann_stop')
                    return
                else:
                    native = self.native_motion
                    slip = bool(native is not None and now - self.native_motion_at <= .5
                        and native['speed'] < .2 and abs(native['wheelspeed']) > max(.3, native['speed'] + .3))
                    control.throttle = self.throttle_loop.update(drive_throttle,
                        abs(self.target_speed) - progress_speed, drive_brake, dt,
                        gain=float(self.get_parameter('throttle_integral_gain').value),
                        maximum=float(self.get_parameter('max_throttle').value),
                        rise_rate=float(self.get_parameter('throttle_rise_rate').value),
                        slip=slip, slip_limit=float(self.get_parameter('min_throttle').value))
                    control.brake = drive_brake
                    steering = steering_from_angular(
                        self.target_angular, self.target_speed,
                        float(self.get_parameter("wheelbase").value),
                        float(self.get_parameter("max_steer_angle").value))
                control.steering = clamp(
                    steering, -float(self.get_parameter("max_steering").value),
                    float(self.get_parameter("max_steering").value)) * float(
                        self.get_parameter("steer_sign").value)
        reason = ('execution_hold' if self.execution_hold.held else
                  'disabled' if not self.enabled else 'stale_command' if stale else
                  'stale_state' if state_stale else
                  'native_control_unavailable' if native_control_stale else
                  'overspeed' if overspeed else
                  'invalid_motion_direction' if invalid_direction else
                  'gear_change' if self.pending_gear is not None else
                  'opposite_motion_braking' if opposite_motion else 'tracking')
        self.publish_control(control, reason)

    def on_execution_hold(self, message):
        if self.execution_hold.receive(message):
            # Both hold and release discard cached velocity. Resume requires a
            # new controller command, not the pre-timeout throttle demand.
            self.invalidate_controller_command()
            if self.control_protocol == PROTOCOL:
                self.source_sender.new_generation()
                self.source_receipt = None
                self.native_control_ok = False
            self.tick()

    def publish_control(self, control, reason):
        source = None
        source_ack_ready = False
        if self.control_protocol == PROTOCOL:
            expiry = None
            if self.require_stamped_command and reason == 'tracking':
                remaining = self.controller_remaining_ns()
                if remaining <= 0:
                    # Input may expire between the tick guard and this send.
                    # A fresh safety brake has its own lifetime; stale torque
                    # must never be wrapped in a new source generation.
                    control.throttle = 0.
                    control.brake = 1.
                    control.steering = 0.
                    self.throttle_loop.reset()
                    reason = 'stale_command'
                else:
                    expiry = min(self.controller_stamp_ros_ns+min(LIFETIME_NS,
                        round(float(self.get_parameter('command_timeout').value)*1e9)),
                        time.time_ns()+remaining)
            source = self.source_sender.issue(control, time.time_ns(), time.monotonic_ns(),
                                               expires_wall_ns=expiry)
            source_ack_ready = self.source_sender.ack_issue(
                self.source_receipt, time.time_ns(), time.monotonic_ns()) is None
            self.pub.publish(String(data=json.dumps(source, separators=(',', ':'))))
        else:
            self.pub.publish(control)
        radius = float(self.get_parameter('minimum_turning_radius').value)
        ros_now_ns = self.get_clock().now().nanoseconds
        steady_now = self._safety_clock.now().nanoseconds * 1e-9
        # Diagnosis only. Receipt ages use the existing safety clock; source
        # ages retain their ROS/wall clock identity and do not change guards.
        receipt_age = lambda stamp: steady_now-stamp if stamp > 0 else None
        native_sample_age = (self.native_sample_age_at_receipt + max(0., steady_now-self.native_control_at)
                             if self.native_control_at else None)
        # Actual published control distinguishes execution from waiting or a
        # disabled test. It also exposes impossible controller commands.
        self.control_status_pub.publish(String(data=json.dumps(dict(
            stamp_ns=ros_now_ns, enabled=self.enabled,
            actuating=bool(self.enabled and reason == 'tracking' and abs(self.target_speed) > .01
                           and control.brake < .9), reason=reason, target_speed=self.target_speed,
            target_angular=self.target_angular,
            measured_speed=self.speed, moving=bool(math.isfinite(self.speed) and self.speed > .05),
            measured_longitudinal_speed=(self.longitudinal_speed
                                         if math.isfinite(self.longitudinal_speed) else None),
            throttle=control.throttle, brake=control.brake,
            control_protocol=self.control_protocol, source_ack_ready=source_ack_ready,
            source_control=({key: source[key] for key in IDENTITY} if source else None),
            command_receipt_age_steady_sec=receipt_age(self.last_command_time),
            requires_stamped_command=self.require_stamped_command,
            controller_stamp_ros_ns=self.controller_stamp_ros_ns,
            controller_source_age_ros_sec=((ros_now_ns-self.controller_stamp_ros_ns)*1e-9
                                           if self.controller_stamp_ros_ns else None),
            controller_deadline_remaining_sec=(self.controller_remaining_ns()*1e-9
                                               if self.require_stamped_command else None),
            controller_rejection=self.controller_rejection,
            state_receipt_age_steady_sec=receipt_age(self.last_state_time),
            native_receipt_age_steady_sec=receipt_age(self.native_control_at),
            state_source_stamp_ros_ns=self.last_source_state_ns,
            state_source_frame=self.last_source_state_frame,
            state_source_age_ros_sec=((ros_now_ns-self.last_source_state_ns)*1e-9
                                      if self.last_source_state_ns else None),
            native_source_wall_time=self.native_source_wall_time,
            native_health_ok=bool(self.native_control_ok and native_sample_age is not None
                                  and native_sample_age <= .5),
            native_sample_age_steady_sec=native_sample_age,
            current_gear=self.current_gear,pending_gear=self.pending_gear,
            throttle_saturated=bool(control.throttle >= float(self.get_parameter('max_throttle').value) - 1e-6),
            command_minimum_turning_radius=radius,
            physical_minimum_turning_radius=self.physical_turning_radius,
            raw_curvature=self.raw_curvature,
            applied_curvature=(self.target_angular/self.target_speed if abs(self.target_speed) > .01 else 0.),
            curvature_infeasible=bool(abs(self.raw_curvature) > 1./self.physical_turning_radius + .01)))))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = VehicleControl()
        stop.brake = 1.0
        stop.throttle = 0.0
        stop.parkingbrake = 0.0
        stop.gear = node.current_gear
        try:
            node.publish_control(stop, 'shutdown')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
