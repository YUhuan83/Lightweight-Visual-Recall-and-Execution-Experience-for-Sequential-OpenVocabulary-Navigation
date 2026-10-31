from pathlib import Path
import hashlib,json,os,signal,subprocess,sys,time,inspect,shutil,re
ROOT=Path('/home/ly/town_memory_validation_20261008')
ARM=Path('/home/ly/wildos_ab_20261008_B_fix')
name=sys.argv[1] if len(sys.argv)>1 else 'B_green_house_station_trial1'
assert name.replace('_','').isalnum()
out=ROOT/name
assert not out.exists(), 'Trial directory already exists'
out.mkdir()
subprocess.run([sys.executable,str(ROOT/'server_test_preflight.py'),
    '--session-dir',str(ROOT),'--arm',str(ARM),'--output',str(out/'server_preflight.json')],check=True)
preflight_report=json.loads((out/'server_preflight.json').read_text())
rviz_display=preflight_report['roles']['rviz'][0]['display']
screen=subprocess.check_output(['xrandr','--display',rviz_display,'--current'],text=True)
resolution=re.search(r'current (\d+) x (\d+)',screen)
assert resolution, 'Cannot determine live RViz display resolution'
video_size=resolution.group(1)+'x'+resolution.group(2)
for artifact in ('server_test_preflight.py','full_trial.py','run_trial.py','server_cleanup_before_trial11.json'):
    path=ROOT/artifact
    if path.exists(): shutil.copyfile(path,out/path.name)
import wildos_nav2_adapter.graph_path_bridge as bridge
import wildos_nav2_adapter.kinematic_vehicle_control as kinetic
import beamng_wildos_adapter.goal_standoff as standoff
provenance={'arm':'B_visual_memory_semantic_instance_fix','commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ARM,text=True).strip(),'dirty':subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=ARM,text=True).strip(),'runtime':{}}
for module in (bridge,kinetic,standoff):
 p=Path(inspect.getfile(module)).resolve()
 assert ARM in p.parents, str(p)
 provenance['runtime'][module.__name__]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
for label,p in [('vision',ARM/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'),('vision_config',ARM/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/configs/beamng_ablation.yaml'),('controller_config',ARM/'nav2_adapter/config/nav2_official_control.yaml'),('lua',Path('/home/ly/beamng-east-coast-user/current/current/lua/ge/extensions/wildoskinematic.lua'))]:
 provenance['runtime'][label]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
(out/'provenance.json').write_text(json.dumps(provenance,indent=2))
rec=None;trial=None
try:
 with (out/'rviz_recording.log').open('w') as log:
  rec=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab','-filter_threads','1','-video_size',video_size,'-framerate','10','-i',rviz_display,'-t','1050','-c:v','libx264','-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',str(out/'rviz_live.mp4')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(.5)
 assert rec.poll() is None, 'RViz recorder failed'
 env=dict(os.environ,WILDOS_TEST_SESSION_DIR=str(ROOT))
 with (out/'trial.log').open('w') as log:
  trial=subprocess.Popen([sys.executable,str(ROOT/'full_trial.py'),name],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  (out/'trial.pid').write_text(str(trial.pid))
  print(json.dumps({'trial':name,'trial_pid':trial.pid,'recorder_pid':rec.pid}),flush=True)
  result=trial.wait(timeout=1000)
  print(json.dumps({'trial_exit':result}),flush=True)
finally:
 for proc in (trial,rec):
  if proc and proc.poll() is None:
   proc.send_signal(signal.SIGINT)
   try: proc.wait(timeout=20)
   except subprocess.TimeoutExpired: proc.terminate();proc.wait(timeout=5)
