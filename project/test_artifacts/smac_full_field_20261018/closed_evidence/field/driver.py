"""420s frozen Smac repair full search and Native timing investigation; no video."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
root=Path('/home/ly/native_delay_20261018')
out=Path('/home/ly/smoothing_boundary_repair_20261018')/('field_native_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
config=json.loads(Path('/home/ly/smoothing_boundary_repair_20261018/field_native_investigation/config.json').read_text())
config.update(output_root=str(out/'sessions'),goal_timeout_s=420)
(out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
child=None;session=None;report=dict(scope=__doc__,passed=False,video=False)
def send(command):
    child.stdin.write(command+'\n');child.stdin.flush()
def wait_for(check,timeout):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if check():return
        if child.poll() is not None:raise RuntimeError('console exit '+str(child.returncode))
        time.sleep(.2)
    raise TimeoutError('bounded Native diagnosis')
try:
    subprocess.run(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,stdout=(out/'prestart.log').open('w'))
    log=out/'console.log'
    child=subprocess.Popen(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python','/home/ly/smoothing_boundary_repair_20261018/field_native_investigation/timing_console.py','--runtime','--config',str(out/'config.json')],stdin=subprocess.PIPE,stdout=log.open('w'),stderr=subprocess.STDOUT,text=True,start_new_session=True)
    (out/'console.pid').write_text(str(child.pid))
    wait_for(lambda:'输入目标（建议英文）' in log.read_text(),300)
    sessions=list((out/'sessions').glob('*/console_identity.json'));assert len(sessions)==1
    session=sessions[0].parent
    if '--idle' in sys.argv:
        until=time.monotonic()+60
        wait_for(lambda:time.monotonic()>=until,70)
        report['brake_only_idle']=True
    else:
        send('green residential house')
        wait_for(lambda:(session/'goal_001/result.json').exists(),540)
        report['goal']=json.loads((session/'goal_001/result.json').read_text())
    send('/quit');child.wait(timeout=100)
    report['component_closed']=child.returncode==0;report['passed']=False
except BaseException as exc:report['error']=repr(exc)
finally:
    if child and child.poll() is None:
        try:send('/quit');child.wait(timeout=100)
        except Exception:
            child.send_signal(signal.SIGTERM)
            try:child.wait(timeout=100)
            except Exception as exc:report['shutdown_error']=repr(exc)
    if session:
        report['session']=str(session)
        result=session/'session_result.json'
        if result.exists():report['session_result']=json.loads(result.read_text())
    else:
        sessions=list((out/'sessions').glob('*/session_result.json'))
        if len(sessions)==1:report['session_result']=json.loads(sessions[0].read_text());report['session']=str(sessions[0].parent)
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),**report),indent=2))
raise SystemExit(0 if report.get('component_closed') else 1)
