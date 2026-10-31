"""Run unchanged physical probe with a bounded read-only DDS observer."""
import argparse,json,os,subprocess,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/curvature_contract_candidate1')
PY='/home/ly/beamng_ros2_0394_ws/.venv/bin/python'
os.environ['PYTHONPATH']=str(BASE/'bundle/tools')+':'+os.environ.get('PYTHONPATH','')
import sys
sys.path.insert(0,str(BASE/'bundle/tools'))
parser=argparse.ArgumentParser();parser.add_argument('--trial',type=int,default=4)
parser.add_argument('--debug-snapshot',action='store_true')
args=parser.parse_args();trial=args.trial;assert trial>0
name='physical_cusp_component'+str(trial)
OUT=BASE/(name+'_observer')
with (BASE/(name+'_observer.log')).open('x') as log:
    observer=subprocess.Popen([PY,str(BASE/'record_execution_contract.py'),str(OUT)]+
        (['--debug-snapshot'] if args.debug_snapshot else []),
        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
deadline=time.monotonic()+10
while time.monotonic()<deadline:
    assert observer.poll() is None,'Observer exited'
    if (OUT/'ready.json').exists():break
    time.sleep(.2)
else:raise RuntimeError('Observer not ready')
from beamngpy import BeamNGpy
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
install_fail_closed_reads(PrefixedLengthSocket)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
snapshot=read_physics_snapshot(b)
assert not physical_snapshot_errors(snapshot) and snapshot['speed']<.05
b.control.resume();b.disconnect()
with (BASE/(name+'.log')).open('x') as log:
    p=subprocess.run([PY,str(BASE/'bundle/tools/check_ackermann_cusp_execution.py'),
        '--output',str(BASE/name),'--arm',str(BASE/'bundle')],
        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,timeout=140)
print(json.dumps({'probe_exit':p.returncode,'observer_pid':observer.pid,
    'observer_alive':observer.poll() is None}),flush=True)
