"""Generate an isolated movement-component owner from the tested brake-only owner."""
from pathlib import Path
import hashlib
import json

root=Path(__file__).resolve().parent
source=(root/'run_braked_component.py').read_text()
replacements={
    'Real brake-only diagnostic-controller lifecycle plus raw/mapped sensor recording.':
        'Recorded short-section execution/contact diagnostic; no object search or video.',
    "ROOT = Path('/home/ly/contact_observer_20261018')":
        "ROOT = Path('/home/ly/recorded_section_replay_20261018')",
    "OUT = ROOT/('braked_'": "OUT = ROOT/('recorded_'",
    "'/home/ly/planning_phase_physical_20261018/field/scene_left90.py'": "str(ROOT/'scene_recorded.py')",
    "str(ROOT/'contact_client.py')": "str(ROOT/'execute_recorded.py')",
    "'/wildos/control_status', '/wildos/diagnostic/contact_snapshot']":
        "'/wildos/control_status', '/wildos/diagnostic/contact_snapshot',\n              '/spot1/measurement_timing', '/vehicles/ego/sensors/state',\n              '/wildos/cmd_vel_stamped', '/wildos/cmd_vel', '/received_global_plan', '/lookahead_point']",
    'deadline = time.monotonic()+75': 'deadline = time.monotonic()+100',
    "    report['passed'] = True":
        "    report['component_closed'] = bool(report['contact_result']['protocol_closed'])\n"
        "    report['passed'] = bool(report['contact_result']['execution_passed'])\n"
        "    report['motion_goals'] = int(report['contact_result']['motion_goals'])\n"
        "    report['outcome'] = report['contact_result']['outcome']",
    "raise SystemExit(0 if report['passed'] else 1)":
        "raise SystemExit(0 if report.get('component_closed') and not report['errors'] else 1)"
}
for old,new in replacements.items():
    assert source.count(old)==1,(old,source.count(old))
    source=source.replace(old,new)
# Parent closes actuation before attempting SDK stop, even if the probe exits
# unexpectedly. Failure to confirm the gate pauses the owned simulation and
# remains an error; it never becomes a successful standstill measurement.
marker='    try:\n        from beamngpy import BeamNGpy\n'
assert source.count(marker)==1
source=source.replace(marker,"""    gate_ok=False
    try:
        gate=subprocess.run(['ros2','param','set','/wildos_cmd_vel_to_control','enable','false'],
            stdout=(OUT/'parent_stop_gate.log').open('w'),stderr=subprocess.STDOUT,timeout=6)
        gate_ok=gate.returncode==0
        if not gate_ok:report['errors'].append('Parent speed gate not confirmed')
    except Exception as exc:report['errors'].append('Parent speed gate: '+repr(exc))
    try:
        from beamngpy import BeamNGpy
""")
needle="        state, samples = settle(lambda: read_physics_snapshot(bng), time.sleep, timeout=12)"
assert source.count(needle)==1
source=source.replace(needle,"""        if not gate_ok:
            bng.control.pause()
            raise RuntimeError('Actuation gate not confirmed; paused, no stop acceptance')
        state, samples = settle(lambda: read_physics_snapshot(bng), time.sleep, timeout=12)""")
out=root/'run_recorded_component.py';out.write_text(source,newline='\n')
launcher=(root/'run_braked_component.sh').read_text().replace('/home/ly/contact_observer_20261018','/home/ly/recorded_section_replay_20261018').replace('run_braked_component.py','run_recorded_component.py')
(root/'run_recorded_component.sh').write_text(launcher,newline='\n')
assert b'\r' not in (root/'run_recorded_component.sh').read_bytes()
print(json.dumps(dict(driver_sha256=hashlib.sha256(source.encode()).hexdigest(),
                      recording_measurement_timing=True,parent_actuation_close=True)))
