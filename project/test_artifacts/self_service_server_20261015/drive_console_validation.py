"""Bounded field validation of the actual console; no video or waypoint injection."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path

root=Path('/home/ly/wildos-console')
parser=argparse.ArgumentParser()
parser.add_argument('stage',choices=['lifecycle','search'])
args=parser.parse_args()
out=root/('field_'+args.stage+'_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir(exist_ok=False)
console=root/'bundle/tools/wildos_console.py'
logpath=out/'console.log'
report={'passed':False,'stage':args.stage,'video':False,'started_wall':time.time()}
commands=[]; child=None; session=None
def send(text):
    child.stdin.write(text+'\n');child.stdin.flush()
    commands.append({'text':text,'wall':time.time()})
def wait_for(test,timeout):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if test():return
        if child.poll() is not None:raise RuntimeError('console exited code '+str(child.returncode))
        time.sleep(.2)
    raise TimeoutError('field driver bounded wait expired')
try:
    with logpath.open('x') as log:
        child=subprocess.Popen(['/usr/bin/python3',str(console),'--config',str(root/'console.json')],
            stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    (out/'driver_child.pid').write_text(str(child.pid))
    wait_for(lambda:'输入目标（建议英文）' in logpath.read_text(),300)
    sessions=[p.parent for p in (root/'results').glob('*/console_identity.json')
              if json.loads(p.read_text())['pid']==child.pid]
    assert len(sessions)==1,sessions
    session=sessions[0];report['session']=str(session)
    send('/status')
    if args.stage=='lifecycle':
        send('/stop')
        wait_for(lambda:'已停车并暂停，可输入下一个目标。' in logpath.read_text(),45)
        send('/status')
        send('/quit')
        child.wait(timeout=100)
    else:
        send('green residential house')
        wait_for(lambda:(session/'goal_001/result.json').exists(),540)
        report['goal']=json.loads((session/'goal_001/result.json').read_text())
        send('/status')
        send('/quit')
        child.wait(timeout=100)
    result=json.loads((session/'session_result.json').read_text())
    report.update(console_exit=child.returncode,session_result=result,
        passed=child.returncode==0 and not result['errors'])
    if args.stage=='search':
        report['automatic_arrival']=report['goal']['state']=='automatic_arrival_pending_visual_review'
        report['passed']=False  # no automatic visual/identity acceptance
        report['visual_review_pending']=report['automatic_arrival']
except BaseException as exc:
    report['error']=repr(exc)
finally:
    if child and child.poll() is None:
        try:
            send('/quit');child.wait(timeout=100)
        except Exception as exc:
            report['shutdown_error']=repr(exc)
            # Signal the console, never blindly terminate vehicle-owning descendants.
            child.send_signal(signal.SIGTERM)
            try:child.wait(timeout=100)
            except Exception as final:report['shutdown_timeout']=repr(final)
    if session:
        report['session']=str(session)
        path=session/'session_result.json'
        if path.exists():report['session_result']=json.loads(path.read_text())
    report.update(commands=commands,elapsed_s=time.time()-report['started_wall'])
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(out),**report},indent=2))
