"""Direct physical steering, preserving the measured actuator calibration."""

import math


def control_lua(
    steering=None,
    throttle=None,
    brake=None,
    parkingbrake=None,
    clutch=None,
    gear=None,
    is_adas=False,
):
    if is_adas:
        raise ValueError("Robot control uses physical steering input")
    lines = []
    for name, value in dict(
        steering=steering,
        throttle=throttle,
        brake=brake,
        parkingbrake=parkingbrake,
        clutch=clutch,
    ).items():
        if value is not None:
            if (
                not math.isfinite(value)
                or not (-1 if name == "steering" else 0) <= value <= 1
            ):
                raise ValueError("Input outside actuator range")
            filtering = "FILTER_DIRECT" if name == "steering" else "FILTER_PAD"
            lines.append(f"input.event('{name}', {float(value)}, {filtering})")
    if gear is not None:
        lines.append(f"drivetrain.shiftToGear({int(gear)})")
    lines.append("return 'wildos_direct_control'")
    return "\n".join(lines)


def install_direct_steering_control(vehicle):
    def control(**values):
        result = vehicle.queue_lua_command(control_lua(**values), response=True)
        if result != "wildos_direct_control":
            raise ConnectionError("Direct input RPC failed")

    vehicle.control = control
