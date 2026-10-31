"""Retire only this verified test stack and sensors; keep engine/map."""
import json,os,signal,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
session=BASE/'resume_session3'
def state(pid):
    try:
        raw=(Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()
        return {'pid':pid,'state':raw[0],'starttime':raw[19],'pgid':os.getpgid(pid),
            'command':(Path('/proc')/str(pid)/'cmdline').read_bytes().decode().replace('\0',' ').strip()}
    except (FileNotFoundError,ProcessLookupError):return None
def alive(old):
    now=state(old['pid']);return now and now['starttime']==old['starttime'] and now['state'] not in ('Z','X')
def stop(rows,sig,grace):
    for row in rows:
        if alive(row):os.kill(row['pid'],sig)
    until=time.monotonic()+grace
    while any(alive(r) for r in rows) and time.monotonic()<until:time.sleep(.1)
stack_pid=int((session/'stack.pid').read_text());sensor_pid=int((session/'sensor.pid').read_text())
stack=state(stack_pid);sensor=state(sensor_pid)
assert stack and 'physical_debug.launch.py' in stack['command'] and stack['pgid']==stack_pid
assert sensor and 'attach_current_vehicle_sensors.py' in sensor['command'] and str(session) in sensor['command']
owned=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:
        row=state(int(entry.name))
        if row and row['pgid']==stack_pid:owned.append(row)
    except PermissionError:continue
assert all(r['pid']==stack_pid or str(BASE/'install') in r['command'] or '/opt/ros/humble/lib/nav2_' in r['command'] or str(BASE/'component_tools/controller_debug_prefix.py') in r['command'] for r in owned)
from beamngpy import BeamNGpy
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
try:b.control.pause()
finally:b.disconnect()
stop(owned,signal.SIGINT,8);stop(owned,signal.SIGTERM,3);stop(owned,signal.SIGKILL,1)
stop([sensor],signal.SIGTERM,6);stop([sensor],signal.SIGKILL,1)
report={'scope':'Test component stack cleanup; engine retained','passed':not any(alive(r) for r in owned+[sensor]),'owned':owned+[sensor]}
(session/'retired_stack.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
assert report['passed']
