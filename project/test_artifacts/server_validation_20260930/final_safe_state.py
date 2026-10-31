import json
from pathlib import Path
from beamngpy import BeamNGpy
b=BeamNGpy('127.0.0.1',25252);b.open(launch=False);v=None
try:
    b.control.pause()
    scenario=json.loads(Path('/home/ly/wildos-sim/east-coast-test-20260927/town.json').read_text())
    spec=scenario['vehicles'][0]
    if str(b.scenario.get_current().level)!=scenario['level']:raise RuntimeError('Map changed')
    reset=b.vehicles.teleport('ego',tuple(spec['pos']),tuple(spec['rot']),reset=True)
    v=b.vehicles.get_current()['ego'];v.connect(b)
    v.ai.set_mode('disabled')
    v.set_shift_mode('realistic_automatic')
    v.control(throttle=0,brake=1,parkingbrake=1,steering=0,clutch=0,gear=0)
    v.sensors.poll()
    report={'vehicle_reset':reset,'simulation_paused':True,'map':'east_coast_usa','vehicle_state':dict(v.state),'wildos_original_pid_alive':Path('/proc/1575639').exists()}
    Path('/home/ly/navigation_validation_20260930_logs/final_safe_state.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
finally:
    if v:v.disconnect()
    b.disconnect()
