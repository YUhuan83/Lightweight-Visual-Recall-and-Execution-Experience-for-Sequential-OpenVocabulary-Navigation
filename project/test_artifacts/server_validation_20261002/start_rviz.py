"""Open the RViz viewer on the existing user's server display, domain 40."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import signal
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env

env=runtime_env(); env.update(DISPLAY=':99',LIBGL_ALWAYS_SOFTWARE='1')
env.pop('QT_QPA_PLATFORM_PLUGIN_PATH',None)
args={
 'visualization':[str(WS/'.venv/bin/python'),str(WS/'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/visualization_bridge')],
 'rviz':['/opt/ros/humble/lib/rviz2/rviz2','-d',str(WS/'install/wildos_nav2_adapter/share/wildos_nav2_adapter/config/beamng_navigation.rviz')],
}
for name,command in args.items():
    pidfile=ROOT/(name+'.pid')
    if pidfile.exists() and Path('/proc',pidfile.read_text().strip()).exists():
        if name=='visualization':continue
        pid=int(pidfile.read_text())
        if Path('/proc',str(pid),'comm').read_text().strip()!='rviz2':raise RuntimeError('Unexpected viewer process')
        os.kill(pid,signal.SIGINT)
        deadline=time.monotonic()+5
        while Path('/proc',str(pid)).exists() and time.monotonic()<deadline:time.sleep(.1)
        if Path('/proc',str(pid)).exists() and Path('/proc',str(pid),'stat').read_text().split()[2]!='Z':raise RuntimeError('Viewer did not stop')
    with ROOT.joinpath(name+'.log').open('ab') as log:
        p=subprocess.Popen(command,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    pidfile.write_text(str(p.pid))
    time.sleep(3)
    if p.poll() is not None:raise RuntimeError(name+' exited: '+ROOT.joinpath(name+'.log').read_text()[-1500:])
    print(name,p.pid,flush=True)
tree=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,text=True,timeout=5)
ROOT.joinpath('rviz_window_tree.txt').write_text(tree.stdout+tree.stderr)
print(tree.stdout[-2000:],flush=True)
