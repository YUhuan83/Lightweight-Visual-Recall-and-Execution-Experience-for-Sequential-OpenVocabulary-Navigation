"""One full current-feature green-house field trial, with original physics and evidence review required."""
from pathlib import Path
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
OUT=ROOT/('trial_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
child=None;session=None;halt=False
report=dict(scope=__doc__,passed=False,errors=[],complete_navigation_passed=False,video=False)
def interrupt(*_):
    global halt
    halt=True
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
def send(command):
    child.stdin.write(command+'\n');child.stdin.flush()
def wait(predicate,seconds):
    until=time.monotonic()+seconds
    while not predicate():
        if halt:raise InterruptedError('Field interrupted')
        if child and child.poll() is not None:raise RuntimeError('Console exited '+str(child.returncode))
        if time.monotonic()>until:raise TimeoutError('Field readiness/result deadline')
        time.sleep(.2)
try:
    result=subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],
                          capture_output=True,text=True,check=True,timeout=20)
    (OUT/'prestart.json').write_text(result.stdout);assert not json.loads(result.stdout)['processes']
    manifest=json.loads((ROOT/'frozen_source_manifest.json').read_text())
    assert all(hashlib.sha256((ROOT/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
    config=json.loads((ROOT/'config.json').read_text());config['output_root']=str(OUT/'sessions')
    (OUT/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    log=OUT/'console.log'
    child=subprocess.Popen([sys.executable,str(ROOT/'console_full.py'),'--runtime','--config',str(OUT/'config.json')],
        stdin=subprocess.PIPE,stdout=log.open('w'),stderr=subprocess.STDOUT,text=True,start_new_session=True)
    (OUT/'console.pid').write_text(str(child.pid))
    wait(lambda:'输入目标（建议英文）' in log.read_text(),300)
    sessions=list((OUT/'sessions').glob('*/console_identity.json'));assert len(sessions)==1;session=sessions[0].parent
    identity=json.loads((session/'model_identity.json').read_text())
    assert identity['sha256']==manifest['members']['wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py']
    assert all(identity['stores'].values()) and identity['confirmed_views_at_start']==0
    expected=config['library_sha256'];libraries={};stack=int((session/'stack.pid').read_text())
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if os.getpgid(int(proc.name))!=stack:continue
            for line in (proc/'maps').read_text().splitlines():
                path=Path(line.split()[-1])
                if path.is_absolute() and path.name in expected:
                    digest=hashlib.sha256(path.read_bytes()).hexdigest();assert digest==expected[path.name],str(path)
                    libraries[path.name]=dict(path=str(path),sha256=digest,pid=int(proc.name))
        except (FileNotFoundError,ProcessLookupError):continue
    assert set(libraries)==set(expected),libraries
    (OUT/'runtime_identity.json').write_text(json.dumps(dict(libraries=libraries,model=identity),indent=2)+'\n')
    send('green residential house')
    wait(lambda:(session/'goal_001/result.json').exists(),540)
    report['goal_result']=json.loads((session/'goal_001/result.json').read_text())
    # Harness arrival remains subject to physical/visual evidence review.
    report['harness_arrival']=bool(report['goal_result'].get('passed'))
except BaseException as error:report['errors'].append(repr(error))
finally:
    halt=False
    if child and child.poll() is None:
        try:
            send('/quit');until=time.monotonic()+100.
            while child.poll() is None and time.monotonic()<until:time.sleep(.2)
            if child.poll() is None:
                child.send_signal(signal.SIGTERM);until=time.monotonic()+50.
                while child.poll() is None and time.monotonic()<until:time.sleep(.2)
            if child.poll() is None:report['errors'].append('Console still live; inspect owned processes before further action')
        except Exception as error:report['errors'].append('Shutdown: '+repr(error))
    if session is None:
        found=list((OUT/'sessions').glob('*/console_identity.json'))
        if len(found)==1:session=found[0].parent
    if session:
        report['session']=str(session)
        if (session/'session_result.json').exists():report['session_result']=json.loads((session/'session_result.json').read_text())
    report['console_exit']=child.poll() if child else None
    report['closed']=child is not None and child.poll() is not None
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),**report),indent=2))
raise SystemExit(0 if report['closed'] and not report['errors'] else 1)
