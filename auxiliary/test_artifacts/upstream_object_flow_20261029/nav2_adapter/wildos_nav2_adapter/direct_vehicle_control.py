"""Robot steering input without BeamNG's gamepad filter.

Use one acknowledged vehicle Lua request, keeping native hydros, tires and
body physics. Other inputs retain the SDK Control handler's PAD filter.
Direct-input stabilization settings remain owned by BeamNG; this adapter
does not modify assistance or physical steering actuator rates.
"""
import math
from .vehicle_control_lease import valid_owner

CONTROL_ACK = 'wildos_direct_control_v1'


def control_lua(steering=None, throttle=None, brake=None, parkingbrake=None,
                clutch=None, gear=None, is_adas=False, *, lease_owner=None, sequence=None):
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
    if lease_owner is not None:
        if (not valid_owner(lease_owner) or type(sequence) is not int
                or not 0 < sequence <= 9007199254740991):
            raise ValueError('Invalid vehicle lease identity or sequence')
        lines.append("local lease=assert(extensions.wildosControlLease,'Vehicle lease missing')")
        lines.append(f"lease.feed('{lease_owner}',{sequence})")
    for name, value in fields.items():
        if value is not None:
            filter_name = 'FILTER_DIRECT' if name == 'steering' else 'FILTER_PAD'
            lines.append(f"input.event('{name}', {value!r}, {filter_name})")
    if gear is not None:
        lines.append(f'drivetrain.shiftToGear({gear})')
    ack = CONTROL_ACK if lease_owner is None else f'wildos_direct_control_v2:{lease_owner}:{sequence}'
    lines.append(f"return '{ack}'")
    return '\n'.join(lines)


def install_direct_steering_control(vehicle, lease_owner=None):
    """Replace only this actuator's control method; never patch SDK globally."""
    if lease_owner is not None and not valid_owner(lease_owner):
        raise ValueError('Invalid vehicle lease owner')
    sequence = 0
    def control(steering=None, throttle=None, brake=None, parkingbrake=None,
                clutch=None, gear=None, is_adas=False):
        nonlocal sequence
        sequence += 1
        chunk = control_lua(steering, throttle, brake, parkingbrake, clutch, gear, is_adas,
                            lease_owner=lease_owner,sequence=sequence)
        ack = CONTROL_ACK if lease_owner is None else f'wildos_direct_control_v2:{lease_owner}:{sequence}'
        result = vehicle.queue_lua_command(chunk, response=True)
        if result != ack:
            raise ConnectionError('Direct vehicle input acknowledgement mismatch')
    vehicle.control = control
