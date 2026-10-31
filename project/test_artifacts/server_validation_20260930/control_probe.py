import json
from pathlib import Path
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics
b=BeamNGpy('127.0.0.1',25252);b.open(launch=False)
v=None
try:
    b.control.pause()
    v=b.vehicles.get_current()['ego'];v.connect(b)
    v.sensors.attach('validation_electrics',Electrics())
    v.sensors.poll()
    e=dict(v.sensors['validation_electrics'])
    report={'state':dict(v.state),'electrics':{k:val for k,val in e.items() if any(x in k.casefold() for x in ('gear','mode','throttle','brake','steer','wheel','rpm'))}}
    Path('/home/ly/navigation_validation_20260930_logs/control_probe.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    # Hold the vehicle independent of longitudinal pedal interpretation.
    v.ai.set_mode('disabled')
    v.set_shift_mode('realistic_automatic')
    v.control(throttle=0,brake=1,parkingbrake=1,steering=0,clutch=0,gear=0)
    print('DIRECT_BRAKES_NEUTRAL; simulation remains paused',flush=True)
finally:
    if v:v.disconnect()
    b.disconnect()
