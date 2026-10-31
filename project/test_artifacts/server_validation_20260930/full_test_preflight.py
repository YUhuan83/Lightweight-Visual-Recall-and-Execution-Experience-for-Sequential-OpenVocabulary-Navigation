import json
from pathlib import Path
import time
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics

out = Path('/home/ly/full_town_memory_20260930')
out.mkdir(exist_ok=True)
b = BeamNGpy('127.0.0.1', 25252)
b.open(launch=False)
v = b.vehicles.get_current()['ego']
v.connect(b)
v.sensors.attach('full_test_preflight', Electrics())
records = []
try:
    b.control.pause()
    v.ai.set_mode('disabled')
    v.set_shift_mode('realistic_automatic')
    v.control(throttle=0, brake=1, parkingbrake=0, steering=0, clutch=0, gear=2)
    b.control.resume()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        t = time.monotonic()
        v.sensors.poll()
        records.append({'wall': t, 'poll_ms': (time.monotonic() - t) * 1000,
                        'state': dict(v.state), 'electrics': dict(v.sensors['full_test_preflight'])})
        time.sleep(.05)
finally:
    b.control.pause()
    v.control(throttle=0, brake=1, parkingbrake=1, steering=0, clutch=0, gear=0)
    b.disconnect()
report = {'samples': records, 'wall_elapsed': records[-1]['wall'] - records[0]['wall'],
          'sim_elapsed': records[-1]['state']['time'] - records[0]['state']['time']}
out.joinpath('preflight.json').write_text(json.dumps(report, indent=2))
print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2))
print('maximum polling ms:', max(s['poll_ms'] for s in records), 'final state:', records[-1]['state'])
