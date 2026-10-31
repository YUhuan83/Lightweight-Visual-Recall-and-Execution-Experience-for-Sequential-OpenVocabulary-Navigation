import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
out=Path('/home/ly/full_town_memory_20260930')
pid=int(out.joinpath('triangulation.pid').read_text())
args=Path('/proc',str(pid),'cmdline').read_bytes().decode().split('\0')[:-1]
env=dict(e.split('=',1) for e in Path('/proc',str(pid),'environ').read_bytes().decode().split('\0') if '=' in e)
config=Path(args[args.index('--config')+1])
if not config.is_absolute():
    config=Path('/home/ly/beamng_ros2_0394_ws/install/visual_navigation/share/visual_navigation/configs')/config
value=yaml.safe_load(config.read_text())
value['syncsub_queue_size']=50
config.write_text(yaml.safe_dump(value))
out.joinpath('triangulation_config.yaml').write_text(yaml.safe_dump(value))
os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+10
while Path('/proc',str(pid)).exists() and time.monotonic()<deadline:time.sleep(.1)
if Path('/proc',str(pid)).exists():raise RuntimeError('Old triangulator did not stop')
with out.joinpath('triangulation.log').open('ab') as f:
    p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
out.joinpath('triangulation.pid').write_text(str(p.pid))
print('Triangulation restarted with cache=50, same slop=0.2; vision PID retained',p.pid)
