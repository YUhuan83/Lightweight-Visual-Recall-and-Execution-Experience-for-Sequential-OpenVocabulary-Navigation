from pathlib import Path
import hashlib,json,shutil,zipfile
root=Path('/home/ly/execution_continuity_review_20261022')
old=root/'v9';wrapper=root/'v9_point_evidence';new=root/'v10_point_support'
assert not new.exists()
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((old/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
shutil.copytree(old/'candidate',new/'candidate',symlinks=True)
with zipfile.ZipFile(root/'incoming/point_support_delta.zip') as archive:
    assert archive.testzip() is None
    for name in archive.namelist():
        path=new/'candidate'/name
        assert path.resolve().is_relative_to((new/'candidate').resolve())
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(archive.read(name))
        manifest['members'][name]=hashlib.sha256(path.read_bytes()).hexdigest()
full=new/'full';full.mkdir()
for source in (wrapper/'full').glob('*.py'):shutil.copy2(source,full/source.name)
(full/'candidate').symlink_to(new/'candidate',target_is_directory=True)
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
config=json.loads((wrapper/'full/config.json').read_text());config['setups'].append(str(new/'install/local_setup.bash'))
config['output_root']=str(full/'sessions')
(full/'config.json').write_text(json.dumps(config,indent=2))
shell=(old/'run_full.sh').read_text().replace(str(old),str(new))
shell=shell.replace('source '+str(new/'install/local_setup.bash'),
    'source '+str(old/'install/local_setup.bash')+'\nsource '+str(new/'install/local_setup.bash'))
(new/'run_full.sh').write_text(shell)
(new/'launch_trial.py').write_text((old/'launch_trial.py').read_text().replace(str(old),str(new)))
(new/'real_ros_review.py').write_text((old/'real_ros_review.py').read_text().replace("modules = (", "modules = ('test_far_ground_support', 'test_pointcloud', "))
build=(old/'build.sh').read_text().replace(str(root/'v8/run_full.sh'),str(old/'run_full.sh')).replace('cd '+str(old),'cd '+str(new))
(new/'build.sh').write_text(build)
print(json.dumps({'candidate':str(new),'members':len(manifest['members']),'change':'pointcloud.py and far-ground tests; navigation, physics and recorder unchanged'}))
