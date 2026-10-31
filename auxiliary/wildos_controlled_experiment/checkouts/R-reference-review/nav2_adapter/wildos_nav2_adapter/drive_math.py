"""ROS-independent control calculations shared by runtime and offline checks."""
from __future__ import annotations

import math

def clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))


class ThrottleIntegrator:
    """Load compensation with anti-windup and a bounded throttle rise."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = self.output = 0.

    def update(self, base, error, brake, dt, *, gain, maximum, rise_rate, slip=False, slip_limit=.06):
        if not 0 < dt <= .5 or brake > 0:
            self.reset()
            return 0.
        if slip:
            self.reset()
            self.output = min(base, slip_limit, maximum)
            return self.output
        increment = gain * error * dt
        proposed = clamp(self.integral + increment, 0., maximum)
        # Do not accumulate positive error while the actuator is saturated.
        if base + proposed <= maximum or increment < 0:
            self.integral = proposed
        desired = clamp(base + self.integral, 0., maximum)
        self.output = min(desired, self.output + rise_rate * dt)
        return self.output


def steering_from_angular(angular_z: float, speed: float, wheelbase: float,
                          max_steer_angle: float, min_speed: float = 0.01) -> float:
    """Convert signed velocity/yaw rate without changing requested curvature.

    ``min_speed`` is a stop threshold, not a denominator floor. RPP requests
    angular_z = speed * curvature, including during its low-speed approach.
    Raising only the denominator to 0.5 m/s silently flattened that curve.
    """
    if abs(speed) <= min_speed:
        return 0.0
    delta = math.atan(wheelbase * angular_z / speed)
    return clamp(delta / max_steer_angle, -1.0, 1.0)


def drive_outputs(target_speed: float, speed: float, throttle_gain: float,
                  gear_forward: int, gear_reverse: int,
                  allow_reverse: bool,
                  current_gear: int | None = None,
                  brake_gain: float = 0.5,
                  max_throttle: float = 0.12,
                  min_throttle: float = 0.06) -> tuple[int, float, float]:
    """Map signed target speed to (gear, positive throttle, brake).

    ``speed`` is the measured speed magnitude from StateSensor. Reverse uses
    BeamNG's -1 gear and positive throttle; a negative throttle is never sent.
    """
    if not all(math.isfinite(v) for v in
               (target_speed, speed, throttle_gain, brake_gain,
                max_throttle, min_throttle)):
        return gear_forward, 0.0, 1.0
    if not 0.0 <= min_throttle <= max_throttle <= 1.0:
        return gear_forward, 0.0, 1.0
    if abs(target_speed) <= 0.01:
        gear = current_gear if current_gear in (gear_forward, gear_reverse) \
            else gear_forward
        return gear, 0.0, 1.0
    reversing = target_speed < 0.0
    if reversing and not allow_reverse:
        return gear_forward, 0.0, 1.0

    gear = gear_reverse if reversing else gear_forward
    error = abs(target_speed) - max(0.0, speed)
    # Apply proportional braking as soon as the measured speed exceeds the
    # target. Waiting for a fixed 0.2 m/s overshoot let the Town SBR coast up
    # past 2 m/s before the global overspeed cutoff could intervene.
    if error > 0.03:
        throttle = clamp(max(throttle_gain * error, min_throttle),
                         0.0, max_throttle)
        brake = 0.0
    elif error < -0.03:
        throttle = 0.0
        brake = clamp(-brake_gain * error, 0.0, 0.8)
    else:
        throttle = 0.0
        brake = 0.0
    return gear, throttle, brake


def bounded_cmd_velocity(raw_speed: float, raw_angular: float, max_speed: float,
                         max_angular: float, allow_reverse: bool,
                         minimum_turning_radius: float | None = None) -> tuple[float, float]:
    """Zero max_speed disables the speed ceiling; reverse remains opt-in."""
    speed = clamp(raw_speed, -max_speed, max_speed) if max_speed > 0 else raw_speed
    # Scaling speed must scale yaw rate too, preserving the requested curve.
    angular = raw_angular * (speed / raw_speed) if abs(raw_speed) > .01 else raw_angular
    limit = max_angular
    if minimum_turning_radius is not None:
        limit = min(limit, abs(speed) / minimum_turning_radius)
    angular = clamp(angular, -limit, limit)
    if speed < 0.0 and not allow_reverse:
        return 0.0, 0.0
    return speed, angular


def speed_exceeds_limit(speed: float, limit: float) -> bool:
    """Zero disables the fixed cutoff; invalid telemetry always stops."""
    return not math.isfinite(speed) or (limit > 0 and speed > limit)


