"""Stationary coverage A/B only: recorded camera/masks, static LiDAR, no navigation."""
import collections,json,math,os,signal,socket,sqlite3,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamngpy import BeamNGpy
from beamngpy.sensors import Lidar
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth
from visual_navigation.utils.semantic_mask import largest_mask_component
from beamng_wildos_adapter.pointcloud import limit_points
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v48_range_candidate'
out=base/'depth_range_probe_20261027';out.mkdir(exist_ok=False)
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
pre=json.loads(subprocess.run([sys.executable,'/home/ly/planning_phase_physical_20261018/prestart.py'],capture_output=True,text=True,check=True).stdout)
assert not pre['processes'] and not pre['display148_lock'],pre
(out/'precheck.json').write_text(json.dumps(pre))
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
i,c=topics['/spot1/object_mask'];masks=[deserialize_message(b,c) for b, in db.execute('select data from messages where topic_id=? order by timestamp',(i,))];db.close()
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def area(m):return sum(m.object_mask.data)
chosen=[]
for m in sorted(masks,key=area,reverse=True):
 if all(abs(stamp(m)-stamp(o))>8e9 for o in chosen):chosen.append(m)
 if len(chosen)==6:break
config=json.loads((root/'full/config.json').read_text());home=Path(config['beamng_home'])
children=[];bng=None;report=dict(scope='Same recorded camera geometry and mask; static world LiDAR at calibrated ray origin. No visual model/navigation, no complete success claim.',rows=[],errors=[])
def spawn(argv,name,cwd=None):
 log=(out/(name+'.log')).open('wb');p=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=cwd);children.append((p,log));return p
try:
 env=dict(os.environ,DISPLAY=':148');os.environ['DISPLAY']=':148'
 xvfb=spawn(['Xvfb',':148','-screen','0','1280x800x24','-nolisten','tcp'],'display');time.sleep(1.)
 engine=spawn([str(home/'BinLinux/BeamNG.tech.x64'),'-headless','-nosfx','-userpath',config['beamng_user'],'-tcom','-tport','25252','-tcom-listen-ip','127.0.0.1','-nosteam'],'engine',str(home))
 deadline=time.monotonic()+90
 while True:
  if engine.poll() is not None:raise RuntimeError('Owned engine exited')
  try:
   with socket.create_connection(('127.0.0.1',25252),timeout=.3):break
  except OSError:
   if time.monotonic()>deadline:raise TimeoutError('engine startup')
   time.sleep(.2)
 scene=subprocess.run([sys.executable,str(root/'candidate/tools/wildos_console_scene.py'),'--home',str(home),'--user',config['beamng_user'],'--output',str(out/'scene.json')],capture_output=True,text=True,timeout=120)
 (out/'scene.log').write_text(scene.stdout+scene.stderr);assert scene.returncode==0,(scene.returncode,scene.stderr[-500:])
 bng=BeamNGpy('127.0.0.1',25252,quit_on_close=False,socket_timeout=10).open(launch=False);bng.control.resume()
 for index,m in enumerate(chosen):
  tf=m.cam_transforms.transforms[0];q=tf.transform.rotation;p=tf.transform.translation;camrot=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix();origin=np.array([p.x,p.y,p.z])
  info=m.cam_infos[0];k=np.asarray(info.k).reshape(3,3);shape=[d.size for d in m.object_mask.layout.dim]
  mask=largest_mask_component(np.asarray(m.object_mask.data,dtype=np.uint8).reshape(shape).reshape(info.height,info.width).astype(bool))
  pose=m.odom.pose.pose;p=pose.position;q=pose.orientation;body=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix();nav=np.array([p.x,p.y,p.z]);ray_origin=nav+body@np.array([1.252,0.,1.560])
  direction=body@np.array([1.,0.,0.]);up=body@np.array([0.,0.,1.])
  for run in (1,2):
   for lines,density,max_range in ((16,30.,40.),(16,30.,80.),(64,60.,80.)):
    sensor=None;started=time.monotonic()
    try:
     sensor=Lidar(f'range_{index}_{run}_{lines}_{int(max_range)}',bng,None,pos=tuple(ray_origin),dir=tuple(direction),up=tuple(up),vertical_resolution=lines,vertical_angle=60.,max_distance=max_range,density=density,requested_update_time=.2,is_static=True,is_dir_world_space=True,is_360_mode=True,is_using_shared_memory=False,is_visualised=False)
     time.sleep(.7);readings=sensor.poll();world=np.asarray(readings['pointCloud']).reshape(-1,3);world=world[np.all(np.isfinite(world),axis=1)]
     sampled=limit_points(world,30000);sampled=sampled[np.linalg.norm(sampled-nav,axis=1)<=max_range];camera=(sampled-origin)@camrot
     valid=camera[:,2]>.1;uv=camera[valid]@k.T;uv=uv[:,:2]/uv[:,2:];inside=(uv[:,0]>=0)&(uv[:,0]<info.width)&(uv[:,1]>=0)&(uv[:,1]<info.height);pix=uv[inside].astype(int);inmask=mask[pix[:,1],pix[:,0]]
     result=foreground_depth(camera,k,mask)
     row=dict(index=index,stamp_ns=stamp(m),run=run,lines=lines,density=density,max_range_m=max_range,mask_pixels=int(mask.sum()),raw_points=len(world),sampled_points=len(sampled),masked_points=int(inmask.sum()),independent_pixels=len(np.unique(pix[inmask],axis=0)),elapsed_s=time.monotonic()-started,located=result is not None and result['association_validated'])
     if result:row.update(position=(camrot@result['position']+origin).tolist(),confidence=result['confidence'],support=result['points'],association_status=result['association_status'])
     report['rows'].append(row)
     if run==1:np.savez_compressed(out/f'frame{index}_{lines}_{int(max_range)}.npz',points_cam=camera,k=k,mask=mask)
     (out/'progress.json').write_text(json.dumps(report,indent=2));print(json.dumps(row),flush=True)
    finally:
     if sensor is not None:sensor.remove()
except BaseException as e:
 report['errors'].append(repr(e));raise
finally:
 if bng is not None:
  try:bng.control.pause();bng.disconnect()
  except Exception as e:report['errors'].append('cleanup '+repr(e))
 for p,log in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
  log.close()
 post=json.loads(subprocess.run([sys.executable,'/home/ly/planning_phase_physical_20261018/prestart.py'],capture_output=True,text=True,check=True).stdout)
 report['postcheck']=post;(out/'report.json').write_text(json.dumps(report,indent=2))
