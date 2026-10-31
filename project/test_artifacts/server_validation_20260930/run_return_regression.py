import os
from pathlib import Path
import subprocess
out=Path('/home/ly/full_town_memory_20260930')
pid=int(out.joinpath('graph_bridge.pid').read_text())
env=dict(e.split('=',1) for e in Path('/proc',str(pid),'environ').read_bytes().decode().split('\0') if '=' in e)
env.update(ROS_DOMAIN_ID='49',ROS_LOCALHOST_ONLY='1')
with out.joinpath('graph_bridge_regression.log').open('w') as f:
    p=subprocess.run(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python','-m','unittest','discover',
                      '-s','test','-p','test_graph_path_bridge_routing.py','-v'],
                     cwd='/home/ly/beamng_ros2_0394_ws/src/wildos_nav2_adapter',env=env,stdout=f,stderr=subprocess.STDOUT)
print('\n'.join(out.joinpath('graph_bridge_regression.log').read_text().splitlines()[-7:]))
raise SystemExit(p.returncode)
