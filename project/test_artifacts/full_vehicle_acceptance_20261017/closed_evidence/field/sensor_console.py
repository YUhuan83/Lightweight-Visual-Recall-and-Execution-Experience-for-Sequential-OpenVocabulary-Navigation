import os,time,sys
from pathlib import Path
sys.path.insert(0,'/home/ly/full_vehicle_acceptance_20261017/candidate/tools')
import wildos_console as m
original=m.OwnedProcesses.spawn

def spawn(self,role,argv,env=None,cwd=None):
 if role=='engine':
  if Path('/tmp/.X148-lock').exists():raise RuntimeError('Display148 already in use')
  original(self,'audit_display',['Xvfb',':148','-screen','0','1920x1080x24','-nolisten','tcp'])
  deadline=time.monotonic()+5
  while not Path('/tmp/.X11-unix/X148').exists() and time.monotonic()<deadline:time.sleep(.1)
  if not Path('/tmp/.X11-unix/X148').exists():raise RuntimeError('Audit display not ready')
 if role=='scene':
  argv=list(argv)
  argv[1]='/home/ly/full_vehicle_acceptance_20261017/field/scene_left90.py'
 if role=='vision':
  argv=list(argv)
  argv[argv.index('--config')+1]='/home/ly/full_vehicle_acceptance_20261017/field/baseline_no_memory.yaml'
  original(self,'execution_observer',[argv[0],'/home/ly/full_vehicle_acceptance_20261017/field/record_execution_contract.py',str(self.output/'execution_observer')],env,cwd)
  argv=[argv[0],'/home/ly/full_vehicle_acceptance_20261017/field/audit_vision.py',*argv[3:]]
  env=dict(env or os.environ,WILDOS_AUDIT_FILE=str(self.output/'vision_participation.jsonl'))
 if role=='preflight':
  argv=[argv[0],'/home/ly/full_vehicle_acceptance_20261017/field/audit_preflight.py',*argv[2:],'--require-visualization']
 if role=='rviz':env=dict(env or os.environ,LIBGL_ALWAYS_SOFTWARE='1',OMP_NUM_THREADS='4')
 return original(self,role,argv,env,cwd)
m.OwnedProcesses.spawn=spawn
original_begin=m.PhysicalBackend.begin
def begin(self,query,folder):
 if not any(role=='sensor_bag' for role,_,_ in self.owned.children):
  child=original(self.owned,'sensor_bag',['ros2','bag','record','-o',str(self.output/'sensor_interfaces'),'/wildos/full_planned_path','/wildos/points','/wildos/clearing_points','/spot1/ouster/front/points_filtered','/tf','/tf_static','/spot1/odom','/spot1/nav_graph','/spot1/scored_nav_graph'])
  deadline=time.monotonic()+10
  while time.monotonic()<deadline:
   if child.poll() is not None:raise RuntimeError('Diagnostic sensor recorder exited')
   if 'All requested topics are subscribed' in (self.output/'sensor_bag.log').read_text():break
   time.sleep(.1)
  else:raise RuntimeError('Diagnostic sensor recorder discovery timeout')
 return original_begin(self,query,folder)
m.PhysicalBackend.begin=begin
raise SystemExit(m.main())
