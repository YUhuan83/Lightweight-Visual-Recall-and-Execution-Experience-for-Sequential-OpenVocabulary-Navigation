"""Bounded field validation of the actual console; no video or waypoint injection."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path

root=Path('/home/ly/visible_route_yaw90_20261017')
parser=argparse.ArgumentParser()
parser.add_argument('stage',choices=['lifecycle','search'])
args=parser.parse_args()
out=root/('field_yaw_left90_'+args.stage+'_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir(exist_ok=False)
console=root/'candidate/tools/wildos_console.py'
logpath=out/'console.log'
report={'passed':False,'stage':args.stage,'video':True,'started_wall':time.time()}
commands=[]; child=None; session=None; screen=None
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
        child=subprocess.Popen(['bash','/home/ly/visible_route_yaw90_20261017/field/sensor_full_console.sh'],
            stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    (out/'driver_child.pid').write_text(str(child.pid))
    wait_for(lambda:'输入目标（建议英文）' in logpath.read_text(),300)
    sessions=[p.parent for p in (root/'field_results').glob('*/console_identity.json')
              if json.loads(p.read_text())['pid']==child.pid]
    assert len(sessions)==1,sessions
    session=sessions[0];report['session']=str(session)
    send('/status')
    component=subprocess.run(['bash',str(root/'field/probe_source.sh'),str(out/'source_lifecycle.json')],stdout=(out/'source_probe.log').open('x'),stderr=subprocess.STDOUT,timeout=40)
    assert component.returncode==0, 'source lifecycle component failed; see source_probe.log'
    report['source_lifecycle']=json.loads((out/'source_lifecycle.json').read_text())
    directions=[r['data']['direction'] for r in report['source_lifecycle']['rows'] if r['kind']=='native']
    headings=[math.atan2(d[1],d[0]) for d in directions]
    expected=-1.7453292519943295
    assert headings and all(abs(math.remainder(h-expected,2*math.pi))<math.radians(3) for h in headings), 'left-90 spawn heading not confirmed'
    report['spawn_heading_check']={'passed':True,'expected_degrees':-100,'observed_degrees':[math.degrees(h) for h in headings]}
    if args.stage=='lifecycle':
        send('/stop')
        wait_for(lambda:'已停车并暂停，可输入下一个目标。' in logpath.read_text(),45)
        send('/status')
        send('/quit')
        child.wait(timeout=100)
    else:
        screenlog=(out/'screen.log').open('xb')
        screen=subprocess.Popen(['ffmpeg','-nostdin','-n','-f','x11grab','-framerate','8','-video_size','1920x1080','-i',':148','-c:v','libx264','-preset','veryfast','-crf','23','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart','-progress',str(out/'screen_progress.log'),str(out/'rviz_live.mp4')],stdout=screenlog,stderr=subprocess.STDOUT,start_new_session=True)
        (out/'screen.pid').write_text(str(screen.pid))
        wait_for(lambda:(out/'screen_progress.log').exists() and 'frame=' in (out/'screen_progress.log').read_text(),15)
        assert screen.poll() is None
        send('green residential house')
        wait_for(lambda:(session/'goal_001/result.json').exists(),540)
        report['goal']=json.loads((session/'goal_001/result.json').read_text())
        screen.send_signal(signal.SIGINT);screen.wait(timeout=20);screenlog.close()
        report['rviz_video']=str(out/'rviz_live.mp4')
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
    if screen and screen.poll() is None:
        screen.send_signal(signal.SIGINT)
        try:screen.wait(timeout=20)
        except subprocess.TimeoutExpired:screen.terminate();screen.wait(timeout=10)
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
