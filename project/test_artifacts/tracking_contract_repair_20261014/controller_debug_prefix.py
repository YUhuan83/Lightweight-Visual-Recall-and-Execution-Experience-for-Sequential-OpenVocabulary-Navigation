"""Parent debugger for one owned controller; one fixed stack snapshot."""
import json,os,queue,signal,subprocess,sys,threading,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
OUT=BASE/'resume_session3';OUT.mkdir(exist_ok=True)
assert sys.argv[1]=='/opt/ros/humble/lib/nav2_controller/controller_server'
log=(OUT/'controller_gdb.log').open('x',buffering=1)
p=subprocess.Popen(['/usr/bin/gdb','-q','-nx','--interpreter=mi2','--args',*sys.argv[1:]],
    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
events=queue.Queue()
def reader():
    for line in p.stdout:
        log.write(line);events.put(line)
        if line.startswith('=thread-group-started'):
            import re
            match=re.search(r'pid="(\d+)"',line)
            if match:(OUT/'debugged_controller.json').write_text(json.dumps({
                'controller_pid':int(match.group(1)),'debugger_pid':p.pid,'prefix_pid':os.getpid()}))
threading.Thread(target=reader,daemon=True).start()
def send(command):p.stdin.write(command+'\n');p.stdin.flush()
send('1-gdb-set pagination off');send('2-gdb-set mi-async on');send('3-exec-run')
requested=interrupted=finished=False
try:
    while p.poll() is None:
        if not requested and (OUT/'snapshot.request').exists():
            requested=True;send('4-exec-interrupt --all')
        try:line=events.get(timeout=.1)
        except queue.Empty:continue
        if requested and not interrupted and line.startswith('*stopped'):
            interrupted=True;send('5-interpreter-exec console "thread apply all bt 18"')
        if interrupted and not finished and line.startswith('5^done'):
            finished=True;(OUT/'snapshot.completed').write_text(str(time.time_ns()))
            send('6-exec-continue')
        if line.startswith('=thread-group-exited'):break
except KeyboardInterrupt:pass
finally:
    if p.poll() is None:
        try:send('-gdb-exit');p.wait(timeout=3)
        except Exception:p.terminate()
    log.close()
