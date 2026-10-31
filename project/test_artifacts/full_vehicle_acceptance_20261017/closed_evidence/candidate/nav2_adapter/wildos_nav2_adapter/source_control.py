"""Bounded source-to-actuator command lifetime on a shared UNIX wall clock.

ROS receipt time never renews a demand. Local monotonic deadlines also bound
commands/receipts across wall-clock rollback. An RPC ACK is not a vehicle stop.
"""
import copy
import json
import math
import re
import uuid
from collections import OrderedDict

PROTOCOL = 'wildos_source_control_v1'
LEGACY_PROTOCOL = 'vehicle_control'
COMMAND_TOPIC = '/wildos/native_control_command'
LIFETIME_NS = 500_000_000
FIELDS = ('throttle', 'brake', 'steering', 'parkingbrake', 'clutch', 'gear')
IDENTITY = ('version', 'clock', 'sender_id', 'generation', 'sequence',
            'issued_wall_ns', 'expires_wall_ns')


def control_values(control):
    value = {k: control[k] if isinstance(control, dict) else getattr(control, k) for k in FIELDS}
    for name in FIELDS[:-1]:
        v = value[name]
        if type(v) not in (int, float) or not math.isfinite(v):
            raise ValueError('nonfinite_control')
        if not (-1. <= v <= 1. if name == 'steering' else 0. <= v <= 1.):
            raise ValueError('control_range')
        value[name] = float(v)
    if type(value['gear']) is not int or not -1 <= value['gear'] <= 8:
        raise ValueError('gear_range')
    return value


def _envelope_issue(value, wall_ns):
    if not isinstance(value, dict) or value.get('version') != PROTOCOL or value.get('clock') != 'unix_ns':
        return 'source_protocol'
    if not isinstance(value.get('sender_id'), str) or not re.fullmatch('[0-9a-f]{32}', value['sender_id']):
        return 'source_identity'
    for key in ('generation', 'sequence', 'issued_wall_ns', 'expires_wall_ns'):
        if type(value.get(key)) is not int or not 0 <= value[key] < 2**63:
            return 'source_integer'
    if value['sequence'] == 0 or value['issued_wall_ns'] == 0:
        return 'source_integer'
    lifetime = value['expires_wall_ns'] - value['issued_wall_ns']
    if not 0 < lifetime <= LIFETIME_NS:
        return 'source_lifetime'
    if wall_ns < value['issued_wall_ns']:
        return 'source_future'
    if wall_ns >= value['expires_wall_ns']:
        return 'source_expired'
    try:
        control_values(value['control'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return 'source_control'
    return None


def receipt_issue(receipt, wall_ns):
    issue = _envelope_issue(receipt, wall_ns)
    if issue:
        return issue
    applied = receipt.get('applied_wall_ns')
    if (receipt.get('applied') is not True or type(applied) is not int
            or not receipt['issued_wall_ns'] <= applied <= wall_ns
            or applied >= receipt['expires_wall_ns']):
        return 'source_not_applied'
    return None


def _braking(control):
    return control['throttle'] == 0. and control['brake'] >= .9


class SourceControlSender:
    def __init__(self, sender_id=None):
        self.sender_id = sender_id or uuid.uuid4().hex
        self.generation = 0
        self.sequence = 0
        self.sent = OrderedDict()
        self.last_ack_sequence = 0

    def new_generation(self):
        self.generation += 1
        self.sent.clear()
        self.last_ack_sequence = 0

    def issue(self, control, wall_ns, mono_ns):
        self.sequence += 1
        frame = dict(version=PROTOCOL, clock='unix_ns', sender_id=self.sender_id,
                     generation=self.generation, sequence=self.sequence,
                     issued_wall_ns=wall_ns, expires_wall_ns=wall_ns + LIFETIME_NS,
                     control=control_values(control))
        error = _envelope_issue(frame, wall_ns)
        if error:
            raise ValueError(error)
        self.sent[self.sequence] = (copy.deepcopy(frame), mono_ns, mono_ns + LIFETIME_NS)
        while len(self.sent) > 64:
            self.sent.popitem(last=False)
        return frame

    def ack_issue(self, receipt, wall_ns, mono_ns):
        error = receipt_issue(receipt, wall_ns)
        if error:
            return error
        if receipt['sender_id'] != self.sender_id or receipt['generation'] != self.generation:
            return 'source_ack_owner'
        record = self.sent.get(receipt['sequence'])
        if record is None:
            return 'source_ack_unknown'
        frame, issued, deadline = record
        if not issued <= mono_ns < deadline:
            return 'source_ack_expired'
        if any(receipt[k] != frame[k] for k in (*IDENTITY, 'control')):
            return 'source_ack_payload'
        if receipt['sequence'] < self.last_ack_sequence:
            return 'source_ack_reordered'
        self.last_ack_sequence = receipt['sequence']
        return None


class SourceControlReceiver:
    def __init__(self):
        self.frame = None
        self.deadline_mono_ns = 0
        self.expired = False
        self.retired = set()
        self.deadlines = OrderedDict()

    def current(self, wall_ns, mono_ns):
        if self.frame is None or self.expired:
            return None
        if mono_ns >= self.deadline_mono_ns or _envelope_issue(self.frame, wall_ns):
            self.expired = True  # A later wall rollback cannot revive old torque.
            return None
        return copy.deepcopy(self.frame)

    def accept(self, payload, wall_ns, mono_ns, *, stopped):
        try:
            if not isinstance(payload, str) or len(payload) > 4096:
                return 'source_packet_size'
            frame = json.loads(payload)
            error = _envelope_issue(frame, wall_ns)
            if error:
                return error
            frame = {k: frame[k] for k in IDENTITY} | {'control': control_values(frame['control'])}
        except (ValueError, KeyError, TypeError, OverflowError, RecursionError):
            return 'source_packet'
        old = self.frame
        live = self.current(wall_ns, mono_ns) is not None
        barrier = _braking(frame['control'])
        if old is None:
            if not barrier:
                return 'source_brake_barrier'
        elif frame['sender_id'] != old['sender_id']:
            if (frame['sender_id'] in self.retired or live or stopped is not True
                    or not barrier or len(self.retired) >= 64):
                return 'source_foreign_owner'
            self.retired.add(old['sender_id'])
        else:
            if frame['generation'] < old['generation'] or frame['sequence'] <= old['sequence']:
                return 'source_reordered'
            if (frame['generation'] > old['generation'] or not live) and not barrier:
                return 'source_brake_barrier'
            if frame['issued_wall_ns'] < old['issued_wall_ns']:
                return 'source_clock_rollback'
        self.frame = frame
        self.expired = False
        self.deadline_mono_ns = mono_ns + frame['expires_wall_ns'] - wall_ns
        key = (frame['sender_id'], frame['generation'], frame['sequence'])
        self.deadlines[key] = self.deadline_mono_ns
        while len(self.deadlines) > 64:
            self.deadlines.popitem(last=False)
        return None

    def receipt(self, frame, actual_control, acknowledged, wall_ns, mono_ns):
        if frame is None:
            return None
        result = {k: frame[k] for k in IDENTITY}
        result['control'] = control_values(actual_control)
        key = (frame['sender_id'], frame['generation'], frame['sequence'])
        result['applied_wall_ns'] = wall_ns
        current = self.current(wall_ns, mono_ns)
        result['applied'] = bool(acknowledged and current is not None
            and frame['sender_id'] == self.frame['sender_id']
            and frame['generation'] == self.frame['generation']
            and mono_ns < self.deadlines.get(key, 0)
            and _envelope_issue(frame, wall_ns) is None
            and result['control'] == frame['control'])
        return result
