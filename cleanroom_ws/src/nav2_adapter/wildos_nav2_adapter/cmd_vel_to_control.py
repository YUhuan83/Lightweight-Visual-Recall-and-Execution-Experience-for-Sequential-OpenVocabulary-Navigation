"""Signed speed PI and calibrated Ackermann steering; no transport protocol."""

import math
import time

import rclpy
from beamng_msgs.msg import StateSensor, VehicleControl
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from .drive_math import (
    ThrottleIntegrator,
    bounded_cmd_velocity,
    drive_outputs,
    steering_from_angular,
)


class CmdVelToControl(Node):
    def __init__(self):
        super().__init__("wildos_cmd_vel_to_control")
        for name, default in dict(
            wheelbase=2.501,
            steer_sign=-1.0,
            max_steering=0.6,
            max_steer_angle=0.54,
            minimum_turning_radius=8.0,
            command_timeout=0.5,
            direction_change_speed=0.15,
        ).items():
            self.declare_parameter(name, default)
        self.integrator = ThrottleIntegrator()
        self.command = None
        self.state = None
        self.gear = 2
        self.gear_changed = 0.0
        self.last_tick = time.monotonic()
        self.publisher = self.create_publisher(
            VehicleControl, "/wildos/native_control", 1
        )
        self.create_subscription(
            TwistStamped, "/wildos/cmd_vel_stamped", self.on_command, 1
        )
        self.create_subscription(StateSensor, "/wildos/native_state", self.on_state, 1)
        self.create_timer(0.05, self.tick)

    def on_command(self, message):
        if message.header.frame_id == "spot1/nav_base_link":
            age = (
                self.get_clock().now().nanoseconds
                - message.header.stamp.sec * 10**9
                - message.header.stamp.nanosec
            ) * 1e-9
            if 0 <= age < self.get_parameter("command_timeout").value:
                self.command = (message.twist, time.monotonic() - age)

    def on_state(self, message):
        self.state = message, time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        output = VehicleControl(gear=self.gear, brake=1.0)
        if (
            self.command is None
            or self.state is None
            or now - self.command[1] > self.get_parameter("command_timeout").value
            or now - self.state[1] > 0.5
        ):
            self.integrator.reset()
            self.publisher.publish(output)
            return
        command, _ = self.command
        state, _ = self.state
        target, angular = bounded_cmd_velocity(
            command.linear.x,
            command.angular.z,
            0.0,
            0.8,
            True,
            self.get_parameter("minimum_turning_radius").value,
        )
        values = (
            target,
            angular,
            state.velocity.x,
            state.velocity.y,
            state.dir.x,
            state.dir.y,
        )
        if not all(math.isfinite(v) for v in values):
            self.publisher.publish(output)
            return
        norm = math.hypot(state.dir.x, state.dir.y)
        speed = (
            (state.velocity.x * state.dir.x + state.velocity.y * state.dir.y) / norm
            if norm > 0.001
            else 0.0
        )
        ground_speed = math.hypot(state.velocity.x, state.velocity.y)
        desired_gear = -1 if target < -0.01 else 2 if target > 0.01 else self.gear
        if desired_gear != self.gear:
            self.integrator.reset()
            if ground_speed < self.get_parameter("direction_change_speed").value:
                self.gear = desired_gear
                self.gear_changed = now
            output.gear = self.gear
        elif now - self.gear_changed >= 0.3:
            direction = -1 if self.gear == -1 else 1
            if direction * speed >= -0.05:
                gear, throttle, brake = drive_outputs(
                    target,
                    direction * speed,
                    0.20,
                    2,
                    -1,
                    True,
                    current_gear=self.gear,
                    max_throttle=0.35,
                )
                output.gear, output.brake = gear, brake
                output.throttle = self.integrator.update(
                    throttle,
                    abs(target) - direction * speed,
                    brake,
                    dt,
                    gain=0.05,
                    maximum=0.35,
                    rise_rate=0.3,
                )
                output.steering = self.get_parameter(
                    "steer_sign"
                ).value * steering_from_angular(
                    angular,
                    target,
                    self.get_parameter("wheelbase").value,
                    self.get_parameter("max_steer_angle").value,
                )
                limit = self.get_parameter("max_steering").value
                output.steering = max(-limit, min(limit, output.steering))
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(VehicleControl(gear=node.gear, brake=1.0))
        node.destroy_node()
        rclpy.shutdown()
