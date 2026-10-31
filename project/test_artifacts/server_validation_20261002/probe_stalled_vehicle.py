"""Read physical vehicle diagnostics with the finished simulation kept paused."""
import json
from pathlib import Path
import sys
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT
from beamngpy import BeamNGpy
from beamngpy.sensors import Damage,Electrics
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5);b.open(launch=False)
v=b.vehicles.get_current()['ego'];v.connect(b)
try:
    v.sensors.attach('stall_damage_probe',Damage());v.sensors.attach('stall_electrics_probe',Electrics())
    v.sensors.poll()
    damage=dict(v.sensors['stall_damage_probe']);electrics=dict(v.sensors['stall_electrics_probe'])
    report={'state':dict(v.state),'damage':damage,
        'electrics':{k:electrics.get(k) for k in ('wheelspeed','airspeed','rpm','gear','gear_index','throttle','brake','parkingbrake','clutch','engineRunning')},
        'limitation':'Paused end state; damage is cumulative and has no pre-trial baseline.'}
    (ROOT/sys.argv[1]/'stalled_vehicle_probe.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'damage':damage.get('damage'),'electrics':report['electrics']}),flush=True)
finally:
    v.sensors.detach('stall_damage_probe');v.sensors.detach('stall_electrics_probe');v.disconnect();b.disconnect()
