"""Preserve three closed-trial comparisons and original timing counterexamples, no live experiment."""
from compare import OUT,BASE,NEW,sha,q
import json,pathlib,collections,shutil,zipfile,sqlite3,math
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Path as RosPath
reports=[json.load(open(OUT/('trial_20261020_'+x+'.json'))) for x in ('215010','224538','225946')]
expected={}
for file,key in ((BASE/'sealed/manifest.json','server_references'),(NEW/'sealed/evidence_manifest.json','references')):
 v=json.load(open(file));expected.update({r['path']:r['sha256'] for r in v[key]})
expected.update({r['path']:r['sha256'] for r in [json.load(open(NEW/'recording_delivery_manifest.json'))['source_bag']]})
raws=[];refs=[];initial=[]
for r in reports:
 x=pathlib.Path(r['trial']).name.split('_')[-1];s=pathlib.Path(r['session']);assert expected[r['source_bag']['path']]==r['source_bag']['sha256']
 for key in ('scene.json','verified_sources.json','goal_001/stop.json'):
  p=s/key;dest=OUT/'provenance'/x/key;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 for name in ('result.json','runtime_identity.json','prestart.json'):
  p=pathlib.Path(r['trial'])/name;dest=OUT/'provenance'/x/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 layers=json.load(open(OUT/('native_layers_'+x+'.json')))
 chosen={v['sequence'] for v in layers['slowest_callbacks'][:3]};trace=[];wire=[]
 for name,target in (('native_timing.jsonl',trace),('native_udp.jsonl',wire)):
  p=s/name;digest=sha(p);refs.append(dict(path=str(p),bytes=p.stat().st_size,sha256=digest))
  if str(p) in expected:assert expected[str(p)]==digest,name
  for line in p.open():
   v=json.loads(line)
   if v.get('sequence') in chosen:target.append(v)
 raws.append(dict(trial=x,selected_sequences=sorted(chosen),timing=trace,udp=wire))
 c=sqlite3.connect('file:'+r['source_bag']['path']+'?mode=ro',uri=True)
 topics={n:i for i,n in c.execute('select id,name from topics')};first=r['planned'][0]; cutoff=first['stamp_ns'];snap=dict(trial=x,query_wall_start=r['query_wall_start'],first_planned=first,maps={})
 for topic in ('/global_costmap/costmap','/local_costmap/costmap','/wildos/planned_path','/spot1/graphnav_planner/path'):
  if topic not in topics:continue
  if 'costmap' in topic or topic=='/spot1/graphnav_planner/path':
   row=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(topics[topic],cutoff)).fetchone()
  else:row=c.execute('select timestamp,data from messages where topic_id=? and timestamp>=? order by timestamp limit 1',(topics[topic],cutoff-100000000)).fetchone()
  if not row:continue
  t,b=row;dest=OUT/'initial_snapshots'/x/(topic.strip('/').replace('/','__')+'.cdr');dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(b)
  v=dict(record_stamp_ns=t,relative_to_query_sec=t/1e9-r['query_wall_start'],cdr=str(dest.relative_to(OUT)),sha256=sha(dest))
  if 'costmap' in topic:
   m=deserialize_message(b,OccupancyGrid);v.update(frame=m.header.frame_id,width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],cost_counts=dict(collections.Counter(m.data)))
  else:
   m=deserialize_message(b,RosPath);v.update(frame=m.header.frame_id,points=len(m.poses),pose_xy=[[p.pose.position.x,p.pose.position.y] for p in m.poses],orientation_xyzw=[[p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w] for p in m.poses])
  snap['maps'][topic]=v
 initial.append(snap);c.close()
(OUT/'native_boundary_counterexamples.json').write_text(json.dumps(raws,indent=2)+'\n')
(OUT/'initial_public_snapshots.json').write_text(json.dumps(initial,indent=2)+'\n')
for base,label in ((BASE,'before'),(NEW,'after')):
 frozen=json.load(open(base/'frozen_source_manifest.json'));dest=OUT/'provenance'/label;dest.mkdir(parents=True,exist_ok=True);shutil.copy2(base/'frozen_source_manifest.json',dest/'frozen_source_manifest.json')
 for name in ('nav2_adapter/wildos_nav2_adapter/confirmed_approach.py','nav2_adapter/wildos_nav2_adapter/target_observation.py','nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/visible_approach.py','official_adapter/beamng_wildos_adapter/goal_standoff.py','nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py','nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py','nav2_adapter/wildos_nav2_adapter/source_control.py','nav2_adapter/wildos_nav2_adapter/continuous_path.py','nav2_adapter/config/fastdds_local_udp.xml'):
  p=base/'candidate'/name;assert sha(p)==frozen['members'][name]
  target=dest/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
files={str(p.relative_to(OUT)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in OUT.rglob('*') if p.is_file() and p.name not in ('comparison_evidence.zip','manifest.json','archive.json')}
manifest=dict(scope=__doc__,files=files,server_references=refs+[r['source_bag'] for r in reports],all_original_bag_hashes_match_previous_evidence=True,trace_previous_hash_scope='215010 and 224538 matched original sealed manifests; 225946 trace first hashed in this comparison',complete_navigation_passed=False,production_changes=False)
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
zpath=OUT/'comparison_evidence.zip'
with zipfile.ZipFile(zpath,'w',zipfile.ZIP_DEFLATED) as z:
 for name in files:z.write(OUT/name,name)
 z.write(OUT/'manifest.json','manifest.json')
info=dict(path=str(zpath),bytes=zpath.stat().st_size,sha256=sha(zpath),payloads=len(files),complete_navigation_passed=False)
(OUT/'archive.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
