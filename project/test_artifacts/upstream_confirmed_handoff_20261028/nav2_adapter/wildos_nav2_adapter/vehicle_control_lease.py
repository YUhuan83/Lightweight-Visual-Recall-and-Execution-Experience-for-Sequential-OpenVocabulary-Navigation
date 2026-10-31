"""Install a source-pinned vehicle-local lease before enabling native control."""
import hashlib
from importlib import resources
import json
import math
import re
import uuid

LEASE_VERSION = 'wildos_control_lease_v1'


def valid_owner(owner):
    return isinstance(owner,str) and re.fullmatch(r'[0-9a-f]{32}',owner) is not None


class VehicleControlLease:
    def __init__(self, bng, vehicle, timeout_sec, owner=None):
        if (isinstance(timeout_sec,bool) or not isinstance(timeout_sec,(int,float))
                or not math.isfinite(timeout_sec) or not 0 < timeout_sec <= 2):
            raise ValueError('Vehicle lease timeout must be in (0, 2] seconds')
        self.owner = uuid.uuid4().hex if owner is None else owner
        if not valid_owner(self.owner): raise ValueError('Invalid lease owner')
        self.timeout_sec = float(timeout_sec)
        source = resources.files(__package__).joinpath('vehicle_control_lease.lua').read_text(encoding='utf8')
        if not source.isascii(): raise ValueError('Vehicle lease source must be ASCII Lua')
        self.source_sha256 = hashlib.sha256(source.encode()).hexdigest()
        path = 'lua/vehicle/extensions/wildosControlLease.lua'
        # Refuse an unknown file instead of overwriting an active deployment.
        install = (f'local source={json.dumps(source)}; assert(loadstring(source)); '
            f'local path={json.dumps(path)}; local old=readFile(path); '
            "assert(old == nil or old == source, 'Vehicle lease source mismatch'); "
            "if old == nil then FS:directoryCreate('lua/vehicle/extensions'); assert(writeFile(path,source)) end; "
            f"return '{self.source_sha256}'")
        if bng.control.queue_lua_command(install,response=True) != self.source_sha256:
            raise ConnectionError('Vehicle lease source installation acknowledgement mismatch')
        claim = ("local loaded=rawget(extensions,'wildosControlLease'); "
            "if not loaded then extensions.load('wildosControlLease') end; "
            "local m=assert(extensions.wildosControlLease,'Vehicle lease not loaded'); "
            f"assert(m.version == '{LEASE_VERSION}', 'Vehicle lease version mismatch'); "
            f"assert(not loaded or m.source_sha256 == '{self.source_sha256}', 'Loaded vehicle lease source mismatch'); "
            f"m.source_sha256='{self.source_sha256}'; return m.claim('{self.owner}',{self.timeout_sec!r})")
        if vehicle.queue_lua_command(claim,response=True) != 'wildos_lease_claimed_v1':
            raise ConnectionError('Vehicle lease claim acknowledgement mismatch')

    def release(self, vehicle):
        chunk = ("local m=assert(extensions.wildosControlLease,'Vehicle lease missing'); "
                 f"return m.release('{self.owner}')")
        result = vehicle.queue_lua_command(chunk,response=True)
        if result not in ('wildos_lease_released_v1','wildos_lease_brake_held_v1'):
            raise ConnectionError('Vehicle lease release acknowledgement mismatch')
        return result
