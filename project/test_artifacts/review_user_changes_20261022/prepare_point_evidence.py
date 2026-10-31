"""Create an observation-only field wrapper; candidate code stays frozen v9."""
from pathlib import Path
import hashlib, json, shutil

root=Path('/home/ly/execution_continuity_review_20261022')
old=root/'v9'; new=root/'v9_point_evidence'
assert not new.exists()
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((old/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
full=new/'full';full.mkdir(parents=True)
(new/'candidate').symlink_to(old/'candidate',target_is_directory=True)
for source in (old/'full').glob('*.py'):shutil.copy2(source,full/source.name)
(full/'candidate').symlink_to(old/'candidate',target_is_directory=True)
shutil.copy2(old/'full/frozen_source_manifest.json',full/'frozen_source_manifest.json')
shutil.copy2(old/'real_ros_report.json',new/'real_ros_report.json')
config=json.loads((old/'full/config.json').read_text())
config['output_root']=str(full/'sessions')
(full/'config.json').write_text(json.dumps(config,indent=2))
addition='''
# Evidence-only recorder extension: do not alter sensor publishers or navigation.
EXTRA_POINT_TOPICS = ["/vehicles/ego/sensors/front_lidar",
    "/spot1/ouster/front/points_filtered", "/wildos/points",
    "/wildos/clearing_points", "/tf_static", "/global_costmap/costmap_raw",
    "/local_costmap/costmap_raw"]
original_popen=console.subprocess.Popen
def evidence_popen(args,*positional,**kwargs):
    if isinstance(args,(list,tuple)) and list(args[:3])==["ros2","bag","record"]:
        args=list(args)+[topic for topic in EXTRA_POINT_TOPICS if topic not in args]
    return original_popen(args,*positional,**kwargs)
console.subprocess.Popen=evidence_popen
'''
wrapper=(full/'console_graph_capture.py').read_text()
assert wrapper.count('raise SystemExit(console.main())')==1
wrapper=wrapper.replace('raise SystemExit(console.main())',addition+'\nraise SystemExit(console.main())')
compile(wrapper,'console_graph_capture.py','exec')
(full/'console_graph_capture.py').write_text(wrapper)
# Test the injected call boundary with a harmless recorder stub.
from types import SimpleNamespace
calls=[]
env={'console':SimpleNamespace(subprocess=SimpleNamespace(Popen=lambda *a,**kw:calls.append((a,kw))))}
exec(addition,env)
for repeat in range(2):
    args=['ros2','bag','record','-o','/tmp/example','/tf']
    env['evidence_popen'](args,stdout='sentinel')
    assert args==['ros2','bag','record','-o','/tmp/example','/tf']
    assert calls[-1][0][0][-7:]==env['EXTRA_POINT_TOPICS']
    assert calls[-1][1]['stdout']=='sentinel'
    plain=['python3','other.py'];env['evidence_popen'](plain)
    assert calls[-1][0][0] is plain
(new/'recorder_boundary_test.json').write_text(json.dumps({'repeats':2,'passed':True}))
shell=(old/'run_full.sh').read_text()
shell=shell.replace('cd '+str(old),'cd '+str(new))
(new/'run_full.sh').write_text(shell)
(new/'launch_trial.py').write_text((old/'launch_trial.py').read_text().replace("root=Path('"+str(old)+"')","root=Path('"+str(new)+"')"))
print(json.dumps({'candidate':str(old/'candidate'),'wrapper':str(new),'frozen_members':len(manifest['members']),'boundary_tests_passed':True}))
