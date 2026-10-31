"""Read vehicle powertrain state without modifying simulation or controls."""
import json
import math
from pathlib import Path
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Damage
b = BeamNGpy('127.0.0.1',25252,socket_timeout=5)
b.open(launch=False)
v = b.vehicles.get_current()['ego']
v.connect(b)
v.sensors.attach('town_diagnostic_electrics',Electrics())
v.sensors.attach('town_diagnostic_damage',Damage())
try:
    v.sensors.poll()
    electric=dict(v.sensors['town_diagnostic_electrics'])
    keys=['rpm','fuel','fuel_capacity','engine_running','engineRunning','engineLoad',
          'throttle','brake','parkingbrake','gear','gear_index','gear_A','gear_M',
          'gearboxMode','airspeed','wheelspeed','ignition','ignition_level','check_engine',
          'oil_temperature','water_temperature']
    value={'position':v.state['pos'],'electrics':{k:electric[k] for k in keys if k in electric},
           'damage':dict(v.sensors['town_diagnostic_damage'])}
    pos=v.state['pos']
    h=math.atan2(v.state['dir'][1],v.state['dir'][0])
    corners={k:[math.cos(h)*(p[0]-pos[0])+math.sin(h)*(p[1]-pos[1]),
                -math.sin(h)*(p[0]-pos[0])+math.cos(h)*(p[1]-pos[1]),p[2]-pos[2]]
             for k,p in v.get_bbox().items()}
    value['body_local_bbox']=corners
    value['body_extents']={'x':[min(p[0] for p in corners.values()),max(p[0] for p in corners.values())],
                           'y':[min(p[1] for p in corners.values()),max(p[1] for p in corners.values())]}
    Path('/home/ly/full_town_memory_20261002/native_health.json').write_text(json.dumps(value,indent=2))
    print(json.dumps(value,indent=2),flush=True)
finally:
    v.sensors.detach('town_diagnostic_electrics')
    v.sensors.detach('town_diagnostic_damage')
    v.disconnect()
    b.disconnect()
