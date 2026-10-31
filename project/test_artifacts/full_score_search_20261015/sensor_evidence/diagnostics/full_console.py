import os,time,sys
from pathlib import Path
sys.path.insert(0,'/home/ly/wildos-console/bundle/tools')
import wildos_console as m
original=m.OwnedProcesses.spawn

def spawn(self,role,argv,env=None,cwd=None):
 if role=='engine':
  if Path('/tmp/.X148-lock').exists():raise RuntimeError('Display148 already in use')
  original(self,'audit_display',['Xvfb',':148','-screen','0','1920x1080x24','-nolisten','tcp'])
  deadline=time.monotonic()+5
  while not Path('/tmp/.X11-unix/X148').exists() and time.monotonic()<deadline:time.sleep(.1)
  if not Path('/tmp/.X11-unix/X148').exists():raise RuntimeError('Audit display not ready')
 if role=='vision':
  original(self,'execution_observer',[argv[0],'/home/ly/full_score_search_20261015/record_execution_contract.py',str(self.output/'execution_observer')],env,cwd)
  argv=[argv[0],'/home/ly/search_mechanism_audit_20261015/audit_vision.py',*argv[3:]]
  env=dict(env or os.environ,WILDOS_AUDIT_FILE=str(self.output/'vision_participation.jsonl'))
 if role=='preflight':
  argv=[argv[0],'/home/ly/search_mechanism_audit_20261015/audit_preflight.py',*argv[2:],'--require-visualization']
 if role=='rviz':env=dict(env or os.environ,LIBGL_ALWAYS_SOFTWARE='1',OMP_NUM_THREADS='4')
 return original(self,role,argv,env,cwd)
m.OwnedProcesses.spawn=spawn
raise SystemExit(m.main())
