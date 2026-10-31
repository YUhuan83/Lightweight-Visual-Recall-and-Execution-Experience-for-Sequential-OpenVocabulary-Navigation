from pathlib import Path
import os,sys,signal,time,json,subprocess
out=Path(__file__).parent
assert json.loads((out/'preflight.json').read_text())['passed']
procs=[]
try:
 for name,args in [('camera',[sys.executable,str(out/'record_front.py'),str(out)]),('rviz',['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab','-filter_threads','1','-video_size','1280x1024','-framerate','10','-i',':137','-t','65','-c:v','libx264','-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',str(out/'rviz_live.mp4')])]:
  f=(out/(name+'_recording.log')).open('w');p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p)
 until=time.monotonic()+10
 while not (out/'camera_ready').exists() and time.monotonic()<until:
  assert all(p.poll() is None for p in procs),'recorder stopped'
  time.sleep(.1)
 assert (out/'camera_ready').exists(),'no camera frame'
 subprocess.run([sys.executable,str(out/'followpath_probe.py')],stdout=(out/'followpath_probe.log').open('w'),stderr=subprocess.STDOUT,check=True,timeout=50)
finally:
 for p in procs:
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for p in procs:
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
print((out/'followpath_probe.json').read_text()[:800])
