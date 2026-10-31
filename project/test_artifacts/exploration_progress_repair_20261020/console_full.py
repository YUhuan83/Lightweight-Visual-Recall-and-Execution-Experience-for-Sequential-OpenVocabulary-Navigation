"""Current production console with field-only display, timing and identity observers."""
from pathlib import Path
import os
import sys
import time

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/tools'))
import wildos_console as console
original=console.OwnedProcesses.spawn
def spawn(self,role,argv,env=None,cwd=None):
    if role=='engine':
        assert not Path('/tmp/.X148-lock').exists()
        original(self,'field_display',['Xvfb',':148','-screen','0','1920x1080x24','-nolisten','tcp'])
        until=time.monotonic()+5.
        while not Path('/tmp/.X11-unix/X148').exists() and time.monotonic()<until:time.sleep(.1)
        assert Path('/tmp/.X11-unix/X148').exists()
    if role=='scene':argv=[argv[0],str(ROOT/'scene_left90.py'),*argv[2:]]
    if role=='stack':
        env=dict(env or os.environ)
        diag=Path('/home/ly/native_delay_20261018')
        env['PYTHONPATH']=str(diag/'instrumentation')+os.pathsep+env.get('PYTHONPATH','')
        env['WILDOS_NATIVE_TIMING_FILE']=str(self.output/'native_timing.jsonl')
        env['LD_PRELOAD']=str(diag/'libnative_rx_trace.so')
        env['WILDOS_NATIVE_RX_TRACE_FILE']=str(self.output/'native_udp.jsonl')
        env.pop('WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT',None)
    if role=='vision':
        env=dict(env or os.environ,WILDOS_FIELD_MODEL_IDENTITY=str(self.output/'model_identity.json'))
        argv=[argv[0],str(ROOT/'vision_identity.py'),*argv[3:]]
        original(self,'score_observer',[argv[0],str(ROOT/'score_observer.py'),str(self.output/'decisions.jsonl')],env,cwd)
    if role=='preflight':argv=[argv[0],str(ROOT/'preflight_full.py'),*argv[2:],'--require-visualization']
    if role=='rviz':env=dict(env or os.environ,LIBGL_ALWAYS_SOFTWARE='1',OMP_NUM_THREADS='4')
    return original(self,role,argv,env,cwd)
console.OwnedProcesses.spawn=spawn
raise SystemExit(console.main())
