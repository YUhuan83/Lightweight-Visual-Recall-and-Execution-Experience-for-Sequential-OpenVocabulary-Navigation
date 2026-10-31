"""User requested stop: terminate only verified current project owners."""
import json,os,signal,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
report={'passed':False,'requested_by_user':True,'stopped':[],'errors':[]}

def state(pid):
    try:
        raw=(Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()
        command=(Path('/proc')/str(pid)/'cmdline').read_bytes().decode().replace('\0',' ').strip()
        return {'pid':pid,'state':raw[0],'starttime':raw[19],
            'pgid':os.getpgid(pid),'command':command}
    except (FileNotFoundError,ProcessLookupError):return None

def alive(old):
    now=state(old['pid'])
    return bool(now and now['starttime']==old['starttime'] and now['state'] not in ('Z','X'))

def stop(rows,sig,grace):
    for row in rows:
        if alive(row):os.kill(row['pid'],sig)
    deadline=time.monotonic()+grace
    while any(alive(r) for r in rows) and time.monotonic()<deadline:time.sleep(.1)

session=BASE/'physical_component_session'
stack_pid=int((session/'stack.pid').read_text())
sensor_pid=int((session/'sensor.pid').read_text())
stack=state(stack_pid);sensor=state(sensor_pid);engine=state(1202233)
assert stack and 'physical_vehicle_control.launch.py' in stack['command'] and stack['pgid']==stack_pid
assert sensor and 'attach_current_vehicle_sensors.py' in sensor['command'] and str(session) in sensor['command']
assert engine and 'BeamNG.tech.x64' in engine['command'] and '-tport 25252' in engine['command']
owned=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:
        row=state(int(entry.name))
        if row and row['pgid']==stack_pid:owned.append(row)
    except PermissionError:continue
assert all(r['pid']==stack_pid or str(BASE/'install') in r['command']
    or '/opt/ros/humble/lib/nav2_' in r['command'] for r in owned)
report['owned_before']=owned+[sensor,engine]
try:
    from beamngpy import BeamNGpy
    b=BeamNGpy('127.0.0.1',25252,socket_timeout=3).open(launch=False)
    try:b.control.pause();report['pause_acknowledged']=True
    finally:b.disconnect()
except Exception as exc:report['pause_error']=repr(exc)
stop(owned,signal.SIGINT,8)
stop(owned,signal.SIGTERM,3)
stop(owned,signal.SIGKILL,1)
stop([sensor],signal.SIGTERM,5)
stop([sensor],signal.SIGKILL,1)
stop([engine],signal.SIGTERM,5)
stop([engine],signal.SIGKILL,1)
for row in report['owned_before']:
    done=not alive(row)
    report['stopped'].append({'pid':row['pid'],'stopped':done,'command':row['command']})
    if not done:report['errors'].append('Still running: '+str(row['pid']))
report['passed']=not report['errors']
(BASE/'user_stop_20261014.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'passed':report['passed'],'stopped_pids':[r['pid'] for r in report['stopped']
    if r['stopped']],'errors':report['errors'],'pause_acknowledged':report.get('pause_acknowledged')},indent=2))
raise SystemExit(0 if report['passed'] else 1)
