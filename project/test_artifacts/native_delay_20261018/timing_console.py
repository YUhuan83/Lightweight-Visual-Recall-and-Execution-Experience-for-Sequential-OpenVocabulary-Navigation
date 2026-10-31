"""Frozen original field harness with opt-in Native timing only; no MP4 recorder."""
import os
from pathlib import Path
import sys,time
sys.path.insert(0,'/home/ly/planning_phase_physical_20261018/candidate/tools')
import wildos_console as m
original=m.OwnedProcesses.spawn
root=Path('/home/ly/native_delay_20261018')
def spawn(self,role,argv,env=None,cwd=None):
    if role=='engine':
        if Path('/tmp/.X148-lock').exists():raise RuntimeError('Display148 already in use')
        original(self,'audit_display',['Xvfb',':148','-screen','0','1920x1080x24','-nolisten','tcp'])
        deadline=time.monotonic()+5
        while not Path('/tmp/.X11-unix/X148').exists() and time.monotonic()<deadline:time.sleep(.1)
        assert Path('/tmp/.X11-unix/X148').exists()
    if role=='scene':argv=[argv[0],'/home/ly/planning_phase_physical_20261018/field/scene_left90.py',*argv[2:]]
    if role=='stack':
        env=dict(env or os.environ)
        env['PYTHONPATH']=str(root/'instrumentation')+os.pathsep+env.get('PYTHONPATH','')
        env['WILDOS_NATIVE_TIMING_FILE']=str(self.output/'native_timing.jsonl')
        if os.environ.get('WILDOS_NATIVE_KERNEL_TRACE')=='1':
            env['LD_PRELOAD']=str(root/'libnative_rx_trace.so')
            env['WILDOS_NATIVE_RX_TRACE_FILE']=str(self.output/'native_udp.jsonl')
    if role=='vision':
        argv=list(argv)
        argv[argv.index('--config')+1]='/home/ly/planning_phase_physical_20261018/field/baseline_no_memory.yaml'
        original(self,'execution_observer',[argv[0],'/home/ly/planning_phase_physical_20261018/field/record_execution_contract.py',str(self.output/'execution_observer')],env,cwd)
        argv=[argv[0],'/home/ly/planning_phase_physical_20261018/field/audit_vision.py',*argv[3:]]
        env=dict(env or os.environ,WILDOS_AUDIT_FILE=str(self.output/'vision_participation.jsonl'))
    if role=='preflight':argv=[argv[0],'/home/ly/planning_phase_physical_20261018/field/audit_preflight.py',*argv[2:],'--require-visualization']
    if role=='rviz':env=dict(env or os.environ,LIBGL_ALWAYS_SOFTWARE='1',OMP_NUM_THREADS='4')
    return original(self,role,argv,env,cwd)
m.OwnedProcesses.spawn=spawn
raise SystemExit(m.main())
