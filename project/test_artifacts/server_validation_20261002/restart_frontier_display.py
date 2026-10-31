"""Refresh only visualization nodes; keep the map and memory process loaded."""
import os, signal, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env

env=runtime_env(); env.update(DISPLAY=':99',LIBGL_ALWAYS_SOFTWARE='1')
env.pop('QT_QPA_PLATFORM_PLUGIN_PATH',None)
package=WS/'src/wildos_nav2_adapter'
for name in ('model_visualization.py','visualization_bridge.py'):
    (package/'wildos_nav2_adapter'/name).write_bytes(Path('/home/ly',name).read_bytes())
config=Path('/home/ly/beamng_navigation_frontier.rviz').read_bytes()
for target in (package/'config/beamng_navigation.rviz',WS/'install/wildos_nav2_adapter/share/wildos_nav2_adapter/config/beamng_navigation.rviz'):
    target.write_bytes(config)
commands={
 'visualization':[str(WS/'.venv/bin/python'),str(WS/'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/visualization_bridge')],
 'rviz':['/opt/ros/humble/lib/rviz2/rviz2','-d',str(WS/'install/wildos_nav2_adapter/share/wildos_nav2_adapter/config/beamng_navigation.rviz')],
}
for name,command in commands.items():
    pidfile=ROOT/(name+'.pid')
    if pidfile.exists():
        pid=pidfile.read_text().strip(); proc=Path('/proc',pid)
        if proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z':
            cmd=proc.joinpath('cmdline').read_bytes().decode()
            expected='visualization_bridge' if name=='visualization' else '/rviz2'
            if expected not in cmd: raise RuntimeError('Unexpected process: '+cmd)
            os.kill(int(pid),signal.SIGINT)
            deadline=time.monotonic()+5
            while proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z' and time.monotonic()<deadline:time.sleep(.1)
            if proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z':raise RuntimeError('Display did not stop')
    with ROOT.joinpath(name+'.log').open('ab') as log:
        p=subprocess.Popen(command,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    pidfile.write_text(str(p.pid))
    time.sleep(3)
    if p.poll() is not None:raise RuntimeError(name+' exited: '+ROOT.joinpath(name+'.log').read_text()[-1800:])
    print(name,p.pid,flush=True)
print('vision PID retained',ROOT.joinpath('vision.pid').read_text(),flush=True)

