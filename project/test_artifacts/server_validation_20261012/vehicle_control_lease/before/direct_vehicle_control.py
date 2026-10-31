"""Robot steering input without BeamNG's gamepad filter.

Use one acknowledged vehicle Lua request, keeping native hydros, tires and
body physics. Other inputs retain the SDK Control handler's PAD filter.
Direct-input stabilization settings remain owned by BeamNG; this adapter
does not modify assistance or physical steering actuator rates.
"""
import math

CONTROL_ACK = 'wildos_direct_control_v1'


def control_lua(steering=None, throttle=None, brake=None, parkingbrake=None,
                clutch=None, gear=None, is_adas=False):
    if is_adas is not False:
        raise ValueError('Native robot control does not use the ADAS input arbiter')
    fields = dict(steering=steering, throttle=throttle, brake=brake,
                  parkingbrake=parkingbrake, clutch=clutch)
    for name, value in fields.items():
        if value is None:
            continue
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not (-1. if name == 'steering' else 0.) <= value <= 1.):
            raise ValueError('Invalid vehicle input: ' + name)
    if gear is not None and (isinstance(gear, bool) or not isinstance(gear, int)):
        raise ValueError('Vehicle gear must be an integer')
    # Check constants before any mutation. No arbitrary strings enter Lua.
    lines = ["assert(FILTER_DIRECT == 2 and FILTER_PAD == 1, 'Unsupported input filter contract')"]
    for name, value in fields.items():
        if value is not None:
            filter_name = 'FILTER_DIRECT' if name == 'steering' else 'FILTER_PAD'
            lines.append(f"input.event('{name}', {value!r}, {filter_name})")
    if gear is not None:
        lines.append(f'drivetrain.shiftToGear({gear})')
    lines.append(f"return '{CONTROL_ACK}'")
    return '\n'.join(lines)


def install_direct_steering_control(vehicle):
    """Replace only this actuator's control method; never patch SDK globally."""
    def control(steering=None, throttle=None, brake=None, parkingbrake=None,
                clutch=None, gear=None, is_adas=False):
        chunk = control_lua(steering, throttle, brake, parkingbrake, clutch, gear, is_adas)
        result = vehicle.queue_lua_command(chunk, response=True)
        if result != CONTROL_ACK:
            raise ConnectionError('Direct vehicle input acknowledgement mismatch')
    vehicle.control = control
